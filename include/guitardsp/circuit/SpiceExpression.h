#pragma once

// Shared foundation for the SPICE-subset netlist format (issue #99): the
// magnitude-suffix numeric parser, the SpiceParseError type, and a small
// dependency-free arithmetic expression AST/parser/evaluator for `.PARAM`
// and `{...}`-bracketed element values.
//
// Scope of the expression language (issue #99, "ノブの扱い" follow-up):
// four arithmetic operators (+ - * /), unary minus, parentheses and
// identifiers referencing a `.PARAM` name. No functions (pow/sin/exp/...).
// This is deliberately minimal: every pedal control expression seen in this
// codebase so far (`{100k*drive}`, `{500k*(1-drive)}`) only needs this much,
// and a taper curve like a pot's audio-log law is a *component* property
// (see PotentiometerSpec::normalizedElectricalPosition() in
// guitardsp/hq/ComponentCatalog.h), not a circuit expression -- so it is
// intentionally kept out of the expression evaluator and applied by the
// elaborator instead (see SpiceNetlistLoader.h's pot-pair handling).
//
// Real-time contract: like SpiceNetlistParser.h, everything here only ever
// runs on the control thread while a circuit is being loaded/edited, never
// from the audio callback path.

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace guitardsp::circuit {

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

} // namespace spice_detail

// --- Expression AST -------------------------------------------------------

struct SpiceExprNode {
    enum class Kind { Number, Param, Add, Sub, Mul, Div, Neg };
    Kind kind;
    double number = 0.0;
    std::string paramName;
    std::shared_ptr<SpiceExprNode> lhs;
    std::shared_ptr<SpiceExprNode> rhs;
};
using SpiceExprPtr = std::shared_ptr<SpiceExprNode>;

// Evaluates `node` against `params` (by canonical uppercase name). Throws
// std::runtime_error if an identifier has no entry in `params` -- this is a
// control-thread-only load/param-update error path, not something the
// audio-rate code below ever needs to handle.
inline double evaluateSpiceExpr(const SpiceExprPtr& node,
                                 const std::unordered_map<std::string, double>& params) {
    using Kind = SpiceExprNode::Kind;
    switch (node->kind) {
        case Kind::Number: return node->number;
        case Kind::Param: {
            const auto it = params.find(node->paramName);
            if (it == params.end())
                throw std::runtime_error("expression references undefined .PARAM '" + node->paramName + "'");
            return it->second;
        }
        case Kind::Add: return evaluateSpiceExpr(node->lhs, params) + evaluateSpiceExpr(node->rhs, params);
        case Kind::Sub: return evaluateSpiceExpr(node->lhs, params) - evaluateSpiceExpr(node->rhs, params);
        case Kind::Mul: return evaluateSpiceExpr(node->lhs, params) * evaluateSpiceExpr(node->rhs, params);
        case Kind::Div: return evaluateSpiceExpr(node->lhs, params) / evaluateSpiceExpr(node->rhs, params);
        case Kind::Neg: return -evaluateSpiceExpr(node->lhs, params);
    }
    return 0.0;
}

// Collects every distinct .PARAM name the expression depends on, so a
// loader can build a param-name -> dependent-component index once at load
// time (see SpiceNetlistLoader.h's ParamBinding) instead of re-parsing on
// every knob movement.
inline void collectSpiceExprParams(const SpiceExprPtr& node, std::vector<std::string>& out) {
    using Kind = SpiceExprNode::Kind;
    if (!node) return;
    switch (node->kind) {
        case Kind::Number: return;
        case Kind::Param:
            if (std::find(out.begin(), out.end(), node->paramName) == out.end())
                out.push_back(node->paramName);
            return;
        case Kind::Neg:
            collectSpiceExprParams(node->lhs, out);
            return;
        default:
            collectSpiceExprParams(node->lhs, out);
            collectSpiceExprParams(node->rhs, out);
            return;
    }
}

namespace spice_expr_detail {

struct ExprToken {
    enum class Kind { Number, Ident, Plus, Minus, Star, Slash, LParen, RParen, End };
    Kind kind;
    double number = 0.0;
    std::string ident;
};

// Tokenizes the text found inside a `{...}` expression (braces already
// stripped by the caller). Deliberately its own scanner rather than reusing
// SpiceNetlistParser.h's whitespace/paren tokenizer: an expression's numeric
// suffix must stop at the *first* unrecognized letter run so "500k*drive"
// splits into "500k" (=500e3) and the identifier "drive", not one run-on
// token -- unlike a bare element value, where trailing unit text after a
// suffix is deliberately ignored (see parseSpiceNumber's docs above).
inline std::vector<ExprToken> tokenizeExpr(const std::string& text, int line, const std::string& lineText) {
    std::vector<ExprToken> tokens;
    std::size_t i = 0;
    const std::size_t n = text.size();
    const auto fail = [&](const std::string& message) -> void {
        throw SpiceParseError(line, lineText, message);
    };
    while (i < n) {
        const char c = text[i];
        if (std::isspace(static_cast<unsigned char>(c))) { ++i; continue; }
        if (c == '+') { tokens.push_back({ExprToken::Kind::Plus}); ++i; continue; }
        if (c == '-') { tokens.push_back({ExprToken::Kind::Minus}); ++i; continue; }
        if (c == '*') { tokens.push_back({ExprToken::Kind::Star}); ++i; continue; }
        if (c == '/') { tokens.push_back({ExprToken::Kind::Slash}); ++i; continue; }
        if (c == '(') { tokens.push_back({ExprToken::Kind::LParen}); ++i; continue; }
        if (c == ')') { tokens.push_back({ExprToken::Kind::RParen}); ++i; continue; }
        if (std::isdigit(static_cast<unsigned char>(c)) ||
            (c == '.' && i + 1 < n && std::isdigit(static_cast<unsigned char>(text[i + 1])))) {
            std::size_t j = i;
            while (j < n && std::isdigit(static_cast<unsigned char>(text[j]))) ++j;
            if (j < n && text[j] == '.') {
                ++j;
                while (j < n && std::isdigit(static_cast<unsigned char>(text[j]))) ++j;
            }
            if (j < n && (text[j] == 'e' || text[j] == 'E')) {
                std::size_t k = j + 1;
                if (k < n && (text[k] == '+' || text[k] == '-')) ++k;
                if (k < n && std::isdigit(static_cast<unsigned char>(text[k]))) {
                    j = k;
                    while (j < n && std::isdigit(static_cast<unsigned char>(text[j]))) ++j;
                }
            }
            // Optional single magnitude suffix (MEG checked before the bare
            // 'M' so "1MEG" and "1M" are never confused -- same trap as the
            // element-value parser).
            std::size_t suffixEnd = j;
            if (j + 3 <= n && spice_detail::toUpperCopy(text.substr(j, 3)) == "MEG") {
                suffixEnd = j + 3;
            } else if (j < n && std::isalpha(static_cast<unsigned char>(text[j]))) {
                static const std::string kSuffixLetters = "TGKMUNPF";
                if (kSuffixLetters.find(static_cast<char>(std::toupper(static_cast<unsigned char>(text[j])))) !=
                    std::string::npos) {
                    suffixEnd = j + 1;
                }
            }
            bool ok = false;
            const double value = spice_detail::parseSpiceNumber(text.substr(i, suffixEnd - i), ok);
            if (!ok) fail("invalid numeric literal in expression: '" + text.substr(i, suffixEnd - i) + "'");
            tokens.push_back({ExprToken::Kind::Number, value, {}});
            i = suffixEnd;
            continue;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            std::size_t j = i;
            while (j < n && (std::isalnum(static_cast<unsigned char>(text[j])) || text[j] == '_')) ++j;
            tokens.push_back({ExprToken::Kind::Ident, 0.0, spice_detail::toUpperCopy(text.substr(i, j - i))});
            i = j;
            continue;
        }
        fail(std::string("unexpected character '") + c + "' in expression");
    }
    tokens.push_back({ExprToken::Kind::End});
    return tokens;
}

// Recursive-descent parser: expr := term (('+'|'-') term)*
//                            term := factor (('*'|'/') factor)*
//                            factor := '-' factor | number | ident | '(' expr ')'
class ExprParser {
public:
    ExprParser(std::vector<ExprToken> tokens, int line, const std::string& lineText)
        : tokens_(std::move(tokens)), line_(line), lineText_(lineText) {}

    SpiceExprPtr parseFull() {
        SpiceExprPtr node = parseExpr();
        if (peek().kind != ExprToken::Kind::End) fail("unexpected trailing content in expression");
        return node;
    }

private:
    const ExprToken& peek() const { return tokens_[pos_]; }
    ExprToken take() { return tokens_[pos_++]; }
    [[noreturn]] void fail(const std::string& message) const { throw SpiceParseError(line_, lineText_, message); }

    SpiceExprPtr makeBinary(SpiceExprNode::Kind kind, SpiceExprPtr lhs, SpiceExprPtr rhs) {
        auto node = std::make_shared<SpiceExprNode>();
        node->kind = kind;
        node->lhs = std::move(lhs);
        node->rhs = std::move(rhs);
        return node;
    }

    SpiceExprPtr parseExpr() {
        SpiceExprPtr node = parseTerm();
        for (;;) {
            if (peek().kind == ExprToken::Kind::Plus) { take(); node = makeBinary(SpiceExprNode::Kind::Add, node, parseTerm()); }
            else if (peek().kind == ExprToken::Kind::Minus) { take(); node = makeBinary(SpiceExprNode::Kind::Sub, node, parseTerm()); }
            else break;
        }
        return node;
    }

    SpiceExprPtr parseTerm() {
        SpiceExprPtr node = parseFactor();
        for (;;) {
            if (peek().kind == ExprToken::Kind::Star) { take(); node = makeBinary(SpiceExprNode::Kind::Mul, node, parseFactor()); }
            else if (peek().kind == ExprToken::Kind::Slash) { take(); node = makeBinary(SpiceExprNode::Kind::Div, node, parseFactor()); }
            else break;
        }
        return node;
    }

    SpiceExprPtr parseFactor() {
        if (peek().kind == ExprToken::Kind::Minus) {
            take();
            auto node = std::make_shared<SpiceExprNode>();
            node->kind = SpiceExprNode::Kind::Neg;
            node->lhs = parseFactor();
            return node;
        }
        if (peek().kind == ExprToken::Kind::Number) {
            auto node = std::make_shared<SpiceExprNode>();
            node->kind = SpiceExprNode::Kind::Number;
            node->number = take().number;
            return node;
        }
        if (peek().kind == ExprToken::Kind::Ident) {
            auto node = std::make_shared<SpiceExprNode>();
            node->kind = SpiceExprNode::Kind::Param;
            node->paramName = take().ident;
            return node;
        }
        if (peek().kind == ExprToken::Kind::LParen) {
            take();
            SpiceExprPtr node = parseExpr();
            if (peek().kind != ExprToken::Kind::RParen) fail("expected ')' in expression");
            take();
            return node;
        }
        fail("expected a number, identifier or '(' in expression");
    }

    std::vector<ExprToken> tokens_;
    std::size_t pos_ = 0;
    int line_;
    std::string lineText_;
};

} // namespace spice_expr_detail

// Parses the text found inside a `{...}` expression (braces already
// stripped). `line`/`lineText` are only used to attach a source location to
// a SpiceParseError.
inline SpiceExprPtr parseSpiceExpr(const std::string& text, int line, const std::string& lineText) {
    const auto tokens = spice_expr_detail::tokenizeExpr(text, line, lineText);
    spice_expr_detail::ExprParser parser(tokens, line, lineText);
    return parser.parseFull();
}

// A single element/model numeric field: either a plain literal (the common
// case, and the only case this SPICE subset supported before .PARAM/{...}
// expressions were added) or a `{...}` expression referencing one or more
// `.PARAM`s. Implicitly convertible to double as the literal value so
// existing call sites that only ever used plain-literal cards (e.g.
// tests/SpiceNetlistParserTests.cpp's `nearlyEqual(element.values[0], ...)`)
// keep compiling unchanged; code that needs to handle a knob-dependent value
// must check isExpression() explicitly.
struct SpiceValue {
    double literal = 0.0;
    SpiceExprPtr expr;

    bool isExpression() const noexcept { return static_cast<bool>(expr); }
    double evaluate(const std::unordered_map<std::string, double>& params) const {
        return isExpression() ? evaluateSpiceExpr(expr, params) : literal;
    }
    operator double() const noexcept { return literal; }
};

} // namespace guitardsp::circuit
