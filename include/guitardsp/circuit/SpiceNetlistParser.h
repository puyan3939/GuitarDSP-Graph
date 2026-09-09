#pragma once

// Minimal, dependency-free parser for a SPICE-compatible netlist *subset*
// (issue #99, Layer A / "circuit simulator" step 1). Mirrors JsonValue.h's
// approach: a small hand-written parser with no third-party dependency,
// producing an in-memory document (SpiceNetlist) rather than driving
// MnaCircuitEngine directly.
//
// Supported syntax:
//  - Element cards: R, L, C, D, Q, V, I, E (case-insensitive kind letter).
//  - ".MODEL" directive, for device types "D" and "NPN"/"PNP" only.
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
// Deliberately out of scope (see issue #99): J/M/F/G/H element cards,
// .SUBCKT/.ENDS/X subcircuit calls, .PARAM and expression evaluation, and
// analysis cards (.TRAN/.AC/.OP). A card using any of these is a parse
// error, not a silent no-op, so a user authoring their own circuit gets a
// clear diagnostic instead of a silently-dropped component.
//
// Unlike real SPICE, the first non-comment line of the deck is NOT treated
// as an implicit title/comment: only lines beginning with '*' are comments.
// The issue's spec enumerates comment forms explicitly ("comment lines *",
// inline ';') without mentioning the title-line convention, and silently
// swallowing whatever a user's first line happens to contain (title or not)
// would be a worse default for a format aimed at users hand-authoring
// circuits.
//
// This header only builds the parsed SpiceNetlist structure; it does not
// elaborate a netlist into an MnaCircuitEngine circuit. See the note in the
// issue #99 writeup for why TS808 elaboration is not implemented here yet:
// TS808's clipping stage relies on the "dynamicOpAmp" macro (saturation,
// slew-rate limiting -- not a plain linear VCVS) and its three controls are
// potentiometers, neither of which is representable with a bare
// R/L/C/D/Q/V/I/E element set without extending the format beyond real
// SPICE semantics.
//
// Real-time contract: like JsonValue.h/NetlistLoader.h, this parser only
// ever runs on the control thread while a circuit is being loaded, never
// from the audio callback path.

#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace guitardsp::circuit {

// One element card: kind is the uppercase reference-designator letter
// ('R','L','C','D','Q','V','I','E'); nodes/values are in card order.
// modelName (canonical uppercase) is only populated for 'D'/'Q'.
struct SpiceElement {
    char kind = '\0';
    std::string name;
    std::vector<std::string> nodes;
    std::vector<double> values;
    std::string modelName;
    int line = 0;
};

// One ".MODEL" card. type is canonical uppercase ("D", "NPN" or "PNP");
// params keys are canonical uppercase (e.g. "IS", "N", "RS", "BF", "VAF").
struct SpiceModel {
    std::string name;
    std::string type;
    std::unordered_map<std::string, double> params;
    int line = 0;
};

struct SpiceNetlist {
    std::vector<SpiceElement> elements;
    std::unordered_map<std::string, SpiceModel> models; // keyed by canonical uppercase name
};

// Thrown for malformed SPICE text. Always carries the 1-based source line
// number and that logical line's (comment-stripped, continuation-joined)
// content, per issue #99's requirement that syntax errors be traceable back
// to the offending line for a user hand-authoring a circuit.
class SpiceParseError : public std::runtime_error {
public:
    SpiceParseError(int line, std::string lineText, const std::string& message)
        : std::runtime_error("line " + std::to_string(line) + ": '" + lineText + "' -- " + message),
          line_(line), lineText_(std::move(lineText)) {}

    int line() const noexcept { return line_; }
    const std::string& lineText() const noexcept { return lineText_; }

private:
    int line_;
    std::string lineText_;
};

namespace spice_detail {

inline std::string toUpperCopy(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

inline std::string trimCopy(std::string_view s) {
    std::size_t b = 0;
    std::size_t e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return std::string(s.substr(b, e - b));
}

inline bool isValidNodeName(const std::string& canonical) noexcept {
    if (canonical.empty()) return false;
    for (char c : canonical) {
        if (!std::isalnum(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

inline std::string canonicalNode(std::string_view raw) {
    std::string upper = toUpperCopy(raw);
    if (upper == "0" || upper == "GND") return "0";
    return upper;
}

// Parses a SPICE numeric literal with an optional trailing magnitude suffix
// (T/G/MEG/K/M/U/N/P/F, case-insensitive) followed by ignored trailing unit
// text. Delegates the mantissa/exponent scan to strtod so "2.2e-9", "4.7k",
// "1MEG" and "10uF" all parse the same way real SPICE would.
inline double parseSpiceNumber(const std::string& token, bool& ok) {
    if (token.empty()) { ok = false; return 0.0; }
    char* end = nullptr;
    const double mantissa = std::strtod(token.c_str(), &end);
    if (end == token.c_str()) { ok = false; return 0.0; }
    ok = true;

    const std::string rest(end);
    double multiplier = 1.0;
    if (rest.size() >= 3 && toUpperCopy(rest.substr(0, 3)) == "MEG") {
        multiplier = 1.0e6;
    } else if (!rest.empty()) {
        switch (std::toupper(static_cast<unsigned char>(rest[0]))) {
            case 'T': multiplier = 1.0e12; break;
            case 'G': multiplier = 1.0e9; break;
            case 'K': multiplier = 1.0e3; break;
            case 'M': multiplier = 1.0e-3; break;
            case 'U': multiplier = 1.0e-6; break;
            case 'N': multiplier = 1.0e-9; break;
            case 'P': multiplier = 1.0e-12; break;
            case 'F': multiplier = 1.0e-15; break;
            default: multiplier = 1.0; break;
        }
    }
    return mantissa * multiplier;
}

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

inline void parseModelCard(const std::vector<std::string>& tokens, const LogicalLine& ll, SpiceNetlist& netlist) {
    const auto fail = [&](const std::string& message) -> void {
        throw SpiceParseError(ll.line, ll.text, message);
    };
    if (tokens.size() < 3) fail("'.model name type(...)' requires a name and a device type");

    SpiceModel model;
    model.name = toUpperCopy(tokens[1]);
    model.type = toUpperCopy(tokens[2]);
    if (model.type != "D" && model.type != "NPN" && model.type != "PNP") {
        fail("unsupported .model type '" + tokens[2] +
             "' (only D and NPN/PNP are implemented in this SPICE subset)");
    }
    for (std::size_t i = 3; i < tokens.size(); ++i) {
        const std::string& tok = tokens[i];
        if (tok == "(" || tok == ")") continue;
        const std::size_t eq = tok.find('=');
        if (eq == std::string::npos) fail("expected 'key=value' model parameter, got '" + tok + "'");
        const std::string key = toUpperCopy(tok.substr(0, eq));
        bool ok = false;
        const double value = parseSpiceNumber(tok.substr(eq + 1), ok);
        if (!ok) fail("invalid numeric value for model parameter '" + key + "'");
        model.params[key] = value;
    }
    model.line = ll.line;

    if (netlist.models.count(model.name) != 0) fail("duplicate .model name '" + tokens[1] + "'");
    netlist.models.emplace(model.name, std::move(model));
}

inline void parseElementCard(const std::vector<std::string>& tokens, const LogicalLine& ll, SpiceNetlist& netlist) {
    const auto fail = [&](const std::string& message) -> void {
        throw SpiceParseError(ll.line, ll.text, message);
    };
    const auto num = [&](const std::string& tok) -> double {
        bool ok = false;
        const double value = parseSpiceNumber(tok, ok);
        if (!ok) fail("invalid numeric value '" + tok + "'");
        return value;
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
            if (tokens.size() != 4) {
                fail(std::string(kind == 'R' ? "resistor" : "inductor") +
                     " card 'Xxx n1 n2 value' requires exactly 2 nodes and a value");
            }
            element.nodes = {node(tokens[1]), node(tokens[2])};
            element.values = {num(tokens[3])};
            break;
        }
        case 'C': {
            if (tokens.size() != 4) fail("capacitor card 'Cxx n1 n2 value' requires exactly 2 nodes and a value");
            element.nodes = {node(tokens[1]), node(tokens[2])};
            element.values = {num(tokens[3])};
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
                element.values = {num(tokens[3])};
            } else if (tokens.size() == 5 && toUpperCopy(tokens[3]) == "DC") {
                element.nodes = {node(tokens[1]), node(tokens[2])};
                element.values = {num(tokens[4])};
            } else {
                fail(std::string(kind == 'V' ? "voltage" : "current") +
                     " source card 'Xxx n+ n- [DC] value' requires 2 nodes and a value");
            }
            break;
        }
        case 'E': {
            if (tokens.size() != 6) fail("VCVS card 'Exx n+ n- nc+ nc- gain' requires exactly 4 nodes and a gain value");
            element.nodes = {node(tokens[1]), node(tokens[2]), node(tokens[3]), node(tokens[4])};
            element.values = {num(tokens[5])};
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
        const std::vector<std::string> tokens = spice_detail::tokenize(ll.text);
        if (tokens.empty()) continue;

        if (tokens[0][0] == '.') {
            const std::string directive = spice_detail::toUpperCopy(tokens[0]);
            if (directive == ".MODEL") {
                spice_detail::parseModelCard(tokens, ll, netlist);
            } else {
                throw SpiceParseError(ll.line, ll.text,
                    "unsupported directive '" + tokens[0] +
                    "' (only .MODEL is implemented in this SPICE subset; "
                    ".SUBCKT/.PARAM/.TRAN/.AC/.OP etc. are out of scope, see issue #99)");
            }
            continue;
        }

        spice_detail::parseElementCard(tokens, ll, netlist);
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
