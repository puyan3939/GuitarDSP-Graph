#pragma once

// Minimal, dependency-free parser for a SPICE-compatible netlist *subset*
// (issue #99, Layer A / "circuit simulator" step 1). Mirrors JsonValue.h's
// approach: a small hand-written parser with no third-party dependency,
// producing an in-memory document (SpiceNetlist) rather than driving
// MnaCircuitEngine directly. See SpiceNetlistLoader.h for the elaborator
// that turns a SpiceNetlist into a runnable MnaCircuitEngine circuit.
//
// Supported syntax:
//  - Element cards: R, L, C, D, Q, V, I, E (case-insensitive kind letter),
//    and X (a minimal subcircuit-call card -- see below).
//  - ".MODEL" directive, for device types "D"/"NPN"/"PNP" (as before), plus
//    three engine-macro extensions "OPAMP", "POT" and "CAP" (see below).
//  - ".PARAM name=value [name2=value2 ...]" for named numeric parameters
//    (issue #99 follow-up). Values must be plain SPICE numbers (with an
//    optional magnitude suffix); expressions are not supported in .PARAM
//    itself, only when a .PARAM name is *used* inside a `{...}` value.
//  - `{...}`-bracketed arithmetic expressions anywhere an element card would
//    otherwise take a plain numeric value (e.g. "R1 A B {100k*drive}").
//    Expression grammar/evaluation lives in SpiceExpression.h: four
//    arithmetic operators, unary minus, parentheses, and identifiers that
//    must resolve to a `.PARAM` name at elaboration time.
//  - Full-line comments starting with '*' and inline comments starting with
//    ';'.
//  - Continuation lines starting with '+', joined onto the previous card.
//  - Case-insensitive keywords, model names and node names.
//  - Numeric magnitude suffixes T/G/MEG/K/M/U/N/P/F (case-insensitive; MEG
//    is checked before the single-letter M so "1M" (milli, 1e-3) and "1MEG"
//    (mega, 1e6) are never confused), with any trailing unit text after the
//    suffix ignored (e.g. "1kOhm" == "1k", "10uF" == "10u").
//  - Ground aliasing: node names "0" and "GND" (case-insensitive) both
//    canonicalize to "0".
//
// The "X" card and the "OPAMP"/"POT" .MODEL types are this format's only
// deliberate departure from being a strict real-SPICE subset (issue #99's
// second round, "オペアンプの表現"/"ポットの表現"):
//  - "OPAMP" .MODEL parameters are named after guitardsp::hq::OpAmpSpec's
//    own fields (openLoopGainDb, gainBandwidthHz, ...), not real SPICE
//    op-amp model parameters -- this engine's op-amp macros
//    (MnaCircuitEngine::addOpAmp / addDynamicOpAmpSubcircuit) are a
//    behavioral macro, not a subcircuit expansion, so there is nothing to
//    gain by inventing a translation from real SPICE op-amp parameters and
//    real risk of the translation silently drifting from what the macro
//    actually does.
//  - "POT" .MODEL parameters (TAPER/OHMS/INVERT) describe a component
//    property (the taper law and total resistance of a potentiometer, plus
//    which electrical sense two paired resistor cards represent), not a
//    circuit equation -- see SpiceNetlistLoader.h for how a "POT"-tagged
//    resistor pair is elaborated into a single MnaCircuitEngine
//    potentiometer using the exact same
//    guitardsp::hq::PotentiometerSpec::normalizedElectricalPosition() code
//    the JSON netlist format uses, not a re-derived formula.
//  - "CAP" .MODEL parameters (currently just LEAKAGEOHMS) exist only
//    because guitardsp::hq::CapacitorSpec::leakageResistanceOhms is the one
//    capacitor field (besides capacitanceFarads itself) that actually
//    changes MnaCircuitEngineCore's stamp -- every other CapacitorSpec field
//    (tolerancePercent, voltageRatingVolts, esrOhms, dielectricAbsorption)
//    is unused by the solver, so there is nothing to gain by exposing them
//    here. A 'C' card with no model reference defaults to the same
//    leakage guitardsp::circuit::NetlistLoader.h uses for every non-
//    electrolytic part (1e9 ohms).
//  - "X" here is *not* SPICE's .SUBCKT-expanding X card: it has no
//    .SUBCKT/.ENDS to expand, and only exists so an OPAMP-typed .MODEL can
//    be instantiated (4 nodes = MnaCircuitEngine::addOpAmp's ideal op-amp;
//    6 nodes = addDynamicOpAmpSubcircuit's nonlinear macro). A real SPICE
//    reading this file would treat X as an undefined subcircuit call and
//    reject it -- there is no way to extend .MODEL-only semantics onto a
//    real SPICE X card without inventing a fictitious .SUBCKT, which would
//    be a worse compatibility break than a nonstandard-but-honest X usage.
//    Kept deliberately minimal, per issue #99: no .SUBCKT/.ENDS support.
//
// Deliberately out of scope (see issue #99): J/M/F/G/H element cards,
// .SUBCKT/.ENDS proper, and analysis cards (.TRAN/.AC/.OP). A card using any
// of these is a parse error, not a silent no-op, so a user authoring their
// own circuit gets a clear diagnostic instead of a silently-dropped
// component.
//
// Unlike real SPICE, the first non-comment line of the deck is NOT treated
// as an implicit title/comment: only lines beginning with '*' are comments.
// The issue's spec enumerates comment forms explicitly ("comment lines *",
// inline ';') without mentioning the title-line convention, and silently
// swallowing whatever a user's first line happens to contain (title or not)
// would be a worse default for a format aimed at users hand-authoring
// circuits.
//
// Real-time contract: like JsonValue.h/NetlistLoader.h, this parser only
// ever runs on the control thread while a circuit is being loaded, never
// from the audio callback path.

#include "SpiceExpression.h"

#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace guitardsp::circuit {

// One element card: kind is the uppercase reference-designator letter
// ('R','L','C','D','Q','V','I','E','X'); nodes/values are in card order.
// modelName is populated for 'D'/'Q'/'X' (always) and 'R'/'L' (only when an
// optional trailing model-name field is present, e.g. a potentiometer-pair
// resistor -- see SpiceNetlistLoader.h).
struct SpiceElement {
    char kind = '\0';
    std::string name;
    std::vector<std::string> nodes;
    std::vector<SpiceValue> values;
    std::string modelName;
    int line = 0;
};

// One ".MODEL" card. type is canonical uppercase ("D", "NPN", "PNP",
// "OPAMP" or "POT"); numeric params are canonical uppercase keys (e.g. "IS",
// "N", "RS", "OHMS"). stringParams holds any key=value pair whose value
// didn't parse as a SPICE number (e.g. "TAPER=AUDIO") -- generic rather than
// special-cased to a fixed key list, so it works for any current or future
// symbolic .MODEL parameter.
struct SpiceModel {
    std::string name;
    std::string type;
    std::unordered_map<std::string, double> params;
    std::unordered_map<std::string, std::string> stringParams;
    int line = 0;
};

struct SpiceNetlist {
    std::vector<SpiceElement> elements;
    std::unordered_map<std::string, SpiceModel> models; // keyed by canonical uppercase name
    std::unordered_map<std::string, double> params;      // .PARAM values, keyed by canonical uppercase name
};

namespace spice_detail {

struct LogicalLine {
    int line = 0;
    std::string text;
};

// Splits the deck into logical lines: strips inline ';' comments, drops
// full-line '*' comments, and joins '+' continuation lines onto the
// preceding card. Each result entry's `line` is the 1-based source line the
// card started on.
inline std::vector<LogicalLine> joinLogicalLines(std::string_view text) {
    std::vector<LogicalLine> result;
    std::size_t pos = 0;
    int lineNo = 0;
    while (pos <= text.size()) {
        const std::size_t nl = text.find('\n', pos);
        std::string_view rawLine = (nl == std::string_view::npos) ? text.substr(pos) : text.substr(pos, nl - pos);
        ++lineNo;
        if (!rawLine.empty() && rawLine.back() == '\r') rawLine.remove_suffix(1);

        std::string cleaned(rawLine);
        const std::size_t semi = cleaned.find(';');
        if (semi != std::string::npos) cleaned.erase(semi);
        const std::string trimmed = trimCopy(cleaned);

        if (trimmed.empty() || trimmed[0] == '*') {
            // blank or full-line comment: nothing to append.
        } else if (trimmed[0] == '+') {
            if (result.empty()) {
                throw SpiceParseError(lineNo, std::string(rawLine),
                                       "continuation line ('+') with no preceding card");
            }
            const std::string continuation = trimCopy(trimmed.substr(1));
            if (!continuation.empty()) {
                result.back().text += ' ';
                result.back().text += continuation;
            }
        } else {
            result.push_back(LogicalLine{lineNo, trimmed});
        }

        if (nl == std::string_view::npos) break;
        pos = nl + 1;
    }
    return result;
}

// Replaces every `{...}` span in `text` with a compact placeholder token
// ("@EXPR0@", "@EXPR1@", ...) and appends the span's inner text (braces
// stripped) to `outExprs` at the matching index. Runs before tokenize() so
// the outer card tokenizer never has to know about expression syntax (which
// has its own '(' / ')' handling -- see SpiceExpression.h's tokenizeExpr).
inline std::string extractBraceExpressions(const std::string& text, int line,
                                            std::vector<std::string>& outExprs) {
    std::string result;
    result.reserve(text.size());
    std::size_t i = 0;
    while (i < text.size()) {
        if (text[i] == '{') {
            const std::size_t close = text.find('}', i + 1);
            if (close == std::string::npos) throw SpiceParseError(line, text, "unterminated '{' expression");
            result += "@EXPR";
            result += std::to_string(outExprs.size());
            result += '@';
            outExprs.push_back(text.substr(i + 1, close - i - 1));
            i = close + 1;
        } else if (text[i] == '}') {
            throw SpiceParseError(line, text, "unmatched '}' with no preceding '{'");
        } else {
            result += text[i];
            ++i;
        }
    }
    return result;
}

// Returns true and sets `index` if `tok` is an "@EXPRn@" placeholder
// produced by extractBraceExpressions().
inline bool tryParseExprPlaceholder(const std::string& tok, std::size_t& index) {
    if (tok.size() < 7 || tok.compare(0, 5, "@EXPR") != 0 || tok.back() != '@') return false;
    const std::string digits = tok.substr(5, tok.size() - 6);
    if (digits.empty()) return false;
    for (char c : digits) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    index = static_cast<std::size_t>(std::strtoul(digits.c_str(), nullptr, 10));
    return true;
}

// Splits a logical line's text into tokens on whitespace, treating '(' and
// ')' as their own tokens even when not separated by whitespace (so
// ".model D1 D(IS=1n)" and ".model D1 D (IS=1n)" tokenize identically).
inline std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> tokens;
    std::string current;
    const auto flush = [&]() {
        if (!current.empty()) { tokens.push_back(current); current.clear(); }
    };
    for (char c : line) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            flush();
        } else if (c == '(' || c == ')') {
            flush();
            tokens.push_back(std::string(1, c));
        } else {
            current.push_back(c);
        }
    }
    flush();
    return tokens;
}

inline void parseParamCard(const std::vector<std::string>& tokens, const LogicalLine& ll, SpiceNetlist& netlist) {
    const auto fail = [&](const std::string& message) -> void {
        throw SpiceParseError(ll.line, ll.text, message);
    };
    if (tokens.size() < 2) fail("'.param name=value [...]' requires at least one assignment");
    for (std::size_t i = 1; i < tokens.size(); ++i) {
        const std::string& tok = tokens[i];
        const std::size_t eq = tok.find('=');
        if (eq == std::string::npos) fail("expected 'name=value' .param assignment, got '" + tok + "'");
        const std::string name = toUpperCopy(tok.substr(0, eq));
        if (name.empty()) fail("'.param' assignment is missing a name");
        bool ok = false;
        const double value = parseSpiceNumber(tok.substr(eq + 1), ok);
        if (!ok) {
            fail("invalid numeric value for .param '" + name +
                 "' (expressions are not supported inside .PARAM itself, only when a "
                 ".PARAM name is referenced inside a '{...}' element value)");
        }
        if (netlist.params.count(name) != 0) fail("duplicate .PARAM name '" + name + "'");
        netlist.params[name] = value;
    }
}

inline void parseModelCard(const std::vector<std::string>& tokens, const LogicalLine& ll, SpiceNetlist& netlist) {
    const auto fail = [&](const std::string& message) -> void {
        throw SpiceParseError(ll.line, ll.text, message);
    };
    if (tokens.size() < 3) fail("'.model name type(...)' requires a name and a device type");

    SpiceModel model;
    model.name = toUpperCopy(tokens[1]);
    model.type = toUpperCopy(tokens[2]);
    if (model.type != "D" && model.type != "NPN" && model.type != "PNP" &&
        model.type != "OPAMP" && model.type != "POT" && model.type != "CAP") {
        fail("unsupported .model type '" + tokens[2] +
             "' (only D, NPN/PNP, OPAMP, POT and CAP are implemented in this SPICE subset)");
    }
    for (std::size_t i = 3; i < tokens.size(); ++i) {
        const std::string& tok = tokens[i];
        if (tok == "(" || tok == ")") continue;
        const std::size_t eq = tok.find('=');
        if (eq == std::string::npos) fail("expected 'key=value' model parameter, got '" + tok + "'");
        const std::string key = toUpperCopy(tok.substr(0, eq));
        const std::string rawValue = tok.substr(eq + 1);
        bool ok = false;
        const double value = parseSpiceNumber(rawValue, ok);
        if (ok) {
            model.params[key] = value;
        } else {
            // Not a number: a symbolic parameter value (e.g. TAPER=AUDIO).
            // Stored generically rather than special-cased to one key, so
            // any current or future symbolic .MODEL parameter works the
            // same way.
            model.stringParams[key] = toUpperCopy(rawValue);
        }
    }
    model.line = ll.line;

    if (netlist.models.count(model.name) != 0) fail("duplicate .model name '" + tokens[1] + "'");
    netlist.models.emplace(model.name, std::move(model));
}

inline void parseElementCard(const std::vector<std::string>& tokens, const LogicalLine& ll,
                              const std::vector<std::string>& exprTexts, SpiceNetlist& netlist) {
    const auto fail = [&](const std::string& message) -> void {
        throw SpiceParseError(ll.line, ll.text, message);
    };
    const auto val = [&](const std::string& tok) -> SpiceValue {
        std::size_t index = 0;
        if (tryParseExprPlaceholder(tok, index)) {
            if (index >= exprTexts.size()) fail("internal error resolving expression placeholder '" + tok + "'");
            SpiceValue v;
            v.expr = parseSpiceExpr(exprTexts[index], ll.line, ll.text);
            return v;
        }
        bool ok = false;
        const double value = parseSpiceNumber(tok, ok);
        if (!ok) fail("invalid numeric value '" + tok + "'");
        SpiceValue v;
        v.literal = value;
        return v;
    };
    const auto node = [&](const std::string& tok) -> std::string {
        const std::string canonical = canonicalNode(tok);
        if (!isValidNodeName(canonical)) fail("invalid node name '" + tok + "' (node names must be alphanumeric)");
        return canonical;
    };

    const std::string& name = tokens[0];
    const char kind = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));

    SpiceElement element;
    element.name = name;
    element.kind = kind;
    element.line = ll.line;

    switch (kind) {
        case 'R':
        case 'L': {
            if (tokens.size() != 4 && tokens.size() != 5) {
                fail(std::string(kind == 'R' ? "resistor" : "inductor") +
                     " card 'Xxx n1 n2 value [modelName]' requires exactly 2 nodes, a value, "
                     "and an optional trailing model reference");
            }
            element.nodes = {node(tokens[1]), node(tokens[2])};
            element.values = {val(tokens[3])};
            if (tokens.size() == 5) element.modelName = toUpperCopy(tokens[4]);
            break;
        }
        case 'C': {
            if (tokens.size() != 4 && tokens.size() != 5) {
                fail("capacitor card 'Cxx n1 n2 value [modelName]' requires exactly 2 nodes, a value, "
                     "and an optional trailing model reference (a CAP-typed .MODEL)");
            }
            element.nodes = {node(tokens[1]), node(tokens[2])};
            element.values = {val(tokens[3])};
            if (tokens.size() == 5) element.modelName = toUpperCopy(tokens[4]);
            break;
        }
        case 'D': {
            if (tokens.size() != 4) fail("diode card 'Dxx anode cathode modelName' requires exactly 2 nodes and a model name");
            element.nodes = {node(tokens[1]), node(tokens[2])};
            element.modelName = toUpperCopy(tokens[3]);
            break;
        }
        case 'Q': {
            if (tokens.size() == 5) {
                element.nodes = {node(tokens[1]), node(tokens[2]), node(tokens[3])};
                element.modelName = toUpperCopy(tokens[4]);
            } else if (tokens.size() == 6) {
                element.nodes = {node(tokens[1]), node(tokens[2]), node(tokens[3]), node(tokens[4])};
                element.modelName = toUpperCopy(tokens[5]);
            } else {
                fail("transistor card 'Qxx collector base emitter [substrate] modelName' requires 3 or 4 nodes and a model name");
            }
            break;
        }
        case 'V':
        case 'I': {
            if (tokens.size() == 4) {
                element.nodes = {node(tokens[1]), node(tokens[2])};
                element.values = {val(tokens[3])};
            } else if (tokens.size() == 5 && toUpperCopy(tokens[3]) == "DC") {
                element.nodes = {node(tokens[1]), node(tokens[2])};
                element.values = {val(tokens[4])};
            } else {
                fail(std::string(kind == 'V' ? "voltage" : "current") +
                     " source card 'Xxx n+ n- [DC] value' requires 2 nodes and a value");
            }
            break;
        }
        case 'E': {
            if (tokens.size() != 6) fail("VCVS card 'Exx n+ n- nc+ nc- gain' requires exactly 4 nodes and a gain value");
            element.nodes = {node(tokens[1]), node(tokens[2]), node(tokens[3]), node(tokens[4])};
            element.values = {val(tokens[5])};
            break;
        }
        case 'X': {
            if (tokens.size() < 3) {
                fail("'X' card 'Xxx node1 [node2 ...] modelName' requires at least 1 node and a model name "
                     "(this SPICE subset's X only instantiates an OPAMP-typed .MODEL -- see SpiceNetlistParser.h)");
            }
            for (std::size_t i = 1; i + 1 < tokens.size(); ++i) element.nodes.push_back(node(tokens[i]));
            element.modelName = toUpperCopy(tokens.back());
            break;
        }
        case 'J':
        case 'M':
        case 'F':
        case 'G':
        case 'H':
            fail(std::string("element type '") + kind +
                 "' is not implemented in this SPICE subset (J/M/F/G/H are out of scope; see issue #99)");
            break;
        default:
            fail(std::string("unrecognized element type '") + kind + "'");
    }

    netlist.elements.push_back(std::move(element));
}

} // namespace spice_detail

// Parses SPICE-subset netlist text into an in-memory document. Throws
// SpiceParseError on any syntax error, with the offending line number and
// content attached.
inline SpiceNetlist parseSpiceNetlist(std::string_view text) {
    SpiceNetlist netlist;
    const std::vector<spice_detail::LogicalLine> logicalLines = spice_detail::joinLogicalLines(text);
    for (const spice_detail::LogicalLine& ll : logicalLines) {
        std::vector<std::string> exprTexts;
        const std::string substituted = spice_detail::extractBraceExpressions(ll.text, ll.line, exprTexts);
        const std::vector<std::string> tokens = spice_detail::tokenize(substituted);
        if (tokens.empty()) continue;

        if (tokens[0][0] == '.') {
            const std::string directive = spice_detail::toUpperCopy(tokens[0]);
            if (directive == ".MODEL") {
                spice_detail::parseModelCard(tokens, ll, netlist);
            } else if (directive == ".PARAM") {
                spice_detail::parseParamCard(tokens, ll, netlist);
            } else {
                throw SpiceParseError(ll.line, ll.text,
                    "unsupported directive '" + tokens[0] +
                    "' (only .MODEL and .PARAM are implemented in this SPICE subset; "
                    ".SUBCKT/.TRAN/.AC/.OP etc. are out of scope, see issue #99)");
            }
            continue;
        }

        spice_detail::parseElementCard(tokens, ll, exprTexts, netlist);
    }
    return netlist;
}

// Non-throwing convenience wrapper, mirroring NetlistCircuit::loadFromJson's
// bool+error* style.
inline bool parseSpiceNetlist(std::string_view text, SpiceNetlist& out, std::string* error) {
    try {
        out = parseSpiceNetlist(text);
        return true;
    } catch (const SpiceParseError& e) {
        if (error != nullptr) *error = e.what();
        return false;
    }
}

} // namespace guitardsp::circuit
