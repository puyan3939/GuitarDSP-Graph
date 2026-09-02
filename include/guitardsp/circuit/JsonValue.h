#pragma once

// Minimal, dependency-free JSON reader used to load data-driven circuit
// netlists (see NetlistLoader.h and docs/CIRCUIT_NETLIST_FORMAT.md).
//
// This parser is intentionally small: it supports exactly the JSON subset
// needed for netlist documents (objects, arrays, strings, numbers, bools,
// null) and nothing else (no comments, no trailing commas). It must only run
// on the control thread while a circuit is being prepared/loaded, never from
// the real-time audio callback path.

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace guitardsp::circuit {

class JsonValue {
public:
    enum class Type { null, boolean, number, string, array, object };

    JsonValue() : type_(Type::null) {}
    static JsonValue makeBool(bool value) {
        JsonValue v;
        v.type_ = Type::boolean;
        v.bool_ = value;
        return v;
    }
    static JsonValue makeNumber(double value) {
        JsonValue v;
        v.type_ = Type::number;
        v.number_ = value;
        return v;
    }
    static JsonValue makeString(std::string value) {
        JsonValue v;
        v.type_ = Type::string;
        v.string_ = std::move(value);
        return v;
    }
    static JsonValue makeArray() {
        JsonValue v;
        v.type_ = Type::array;
        return v;
    }
    static JsonValue makeObject() {
        JsonValue v;
        v.type_ = Type::object;
        return v;
    }

    Type type() const noexcept { return type_; }
    bool isNull() const noexcept { return type_ == Type::null; }
    bool isObject() const noexcept { return type_ == Type::object; }
    bool isArray() const noexcept { return type_ == Type::array; }
    bool isString() const noexcept { return type_ == Type::string; }
    bool isNumber() const noexcept { return type_ == Type::number; }

    double asNumber(double fallback = 0.0) const noexcept {
        return type_ == Type::number ? number_ : fallback;
    }
    float asFloat(float fallback = 0.0f) const noexcept {
        return type_ == Type::number ? static_cast<float>(number_) : fallback;
    }
    int asInt(int fallback = 0) const noexcept {
        return type_ == Type::number ? static_cast<int>(number_) : fallback;
    }
    bool asBool(bool fallback = false) const noexcept {
        return type_ == Type::boolean ? bool_ : fallback;
    }
    const std::string& asString(const std::string& fallback = {}) const noexcept {
        return type_ == Type::string ? string_ : fallback;
    }

    const std::vector<JsonValue>& items() const noexcept { return array_; }
    const std::map<std::string, JsonValue>& entries() const noexcept { return object_; }

    bool has(const std::string& key) const noexcept {
        return type_ == Type::object && object_.count(key) != 0;
    }
    const JsonValue& operator[](const std::string& key) const {
        static const JsonValue nullValue{};
        if (type_ != Type::object) return nullValue;
        const auto it = object_.find(key);
        return it == object_.end() ? nullValue : it->second;
    }

    void push(JsonValue value) { array_.push_back(std::move(value)); }
    void set(std::string key, JsonValue value) { object_[std::move(key)] = std::move(value); }

private:
    Type type_ = Type::null;
    bool bool_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<JsonValue> array_;
    std::map<std::string, JsonValue> object_;
};

// Thrown for malformed JSON text. Only ever constructed while parsing on the
// control thread (see the real-time contract note above).
class JsonParseError : public std::runtime_error {
public:
    explicit JsonParseError(const std::string& message) : std::runtime_error(message) {}
};

namespace json_detail {

class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}

    JsonValue parseDocument() {
        skipWhitespace();
        JsonValue value = parseValue();
        skipWhitespace();
        if (pos_ != text_.size()) fail("trailing data after JSON document");
        return value;
    }

private:
    std::string_view text_;
    std::size_t pos_ = 0;

    [[noreturn]] void fail(const char* message) const {
        throw JsonParseError(std::string("JSON parse error at offset ") +
                             std::to_string(pos_) + ": " + message);
    }

    char peek() const { return pos_ < text_.size() ? text_[pos_] : '\0'; }
    char advance() { return text_[pos_++]; }
    bool eof() const { return pos_ >= text_.size(); }

    void skipWhitespace() {
        while (!eof()) {
            const char c = peek();
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { ++pos_; continue; }
            break;
        }
    }

    void expect(char c) {
        if (eof() || peek() != c) fail("unexpected character");
        ++pos_;
    }

    bool consumeLiteral(std::string_view literal) {
        if (text_.substr(pos_, literal.size()) != literal) return false;
        pos_ += literal.size();
        return true;
    }

    JsonValue parseValue() {
        if (eof()) fail("unexpected end of input");
        const char c = peek();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == '"') return JsonValue::makeString(parseString());
        if (c == 't' && consumeLiteral("true")) return JsonValue::makeBool(true);
        if (c == 'f' && consumeLiteral("false")) return JsonValue::makeBool(false);
        if (c == 'n' && consumeLiteral("null")) return JsonValue{};
        if (c == '-' || (c >= '0' && c <= '9')) return parseNumber();
        fail("unexpected token");
    }

    JsonValue parseObject() {
        expect('{');
        JsonValue value = JsonValue::makeObject();
        skipWhitespace();
        if (peek() == '}') { ++pos_; return value; }
        while (true) {
            skipWhitespace();
            if (peek() != '"') fail("expected string key");
            std::string key = parseString();
            skipWhitespace();
            expect(':');
            skipWhitespace();
            value.set(std::move(key), parseValue());
            skipWhitespace();
            if (peek() == ',') { ++pos_; continue; }
            if (peek() == '}') { ++pos_; break; }
            fail("expected ',' or '}' in object");
        }
        return value;
    }

    JsonValue parseArray() {
        expect('[');
        JsonValue value = JsonValue::makeArray();
        skipWhitespace();
        if (peek() == ']') { ++pos_; return value; }
        while (true) {
            skipWhitespace();
            value.push(parseValue());
            skipWhitespace();
            if (peek() == ',') { ++pos_; continue; }
            if (peek() == ']') { ++pos_; break; }
            fail("expected ',' or ']' in array");
        }
        return value;
    }

    std::string parseString() {
        expect('"');
        std::string result;
        while (true) {
            if (eof()) fail("unterminated string");
            const char c = advance();
            if (c == '"') break;
            if (c == '\\') {
                if (eof()) fail("unterminated escape sequence");
                const char escape = advance();
                switch (escape) {
                    case '"': result.push_back('"'); break;
                    case '\\': result.push_back('\\'); break;
                    case '/': result.push_back('/'); break;
                    case 'b': result.push_back('\b'); break;
                    case 'f': result.push_back('\f'); break;
                    case 'n': result.push_back('\n'); break;
                    case 'r': result.push_back('\r'); break;
                    case 't': result.push_back('\t'); break;
                    case 'u': {
                        if (pos_ + 4 > text_.size()) fail("truncated unicode escape");
                        unsigned codepoint = 0;
                        for (int i = 0; i < 4; ++i) {
                            const char hex = advance();
                            codepoint <<= 4;
                            if (hex >= '0' && hex <= '9') codepoint |= static_cast<unsigned>(hex - '0');
                            else if (hex >= 'a' && hex <= 'f') codepoint |= static_cast<unsigned>(hex - 'a' + 10);
                            else if (hex >= 'A' && hex <= 'F') codepoint |= static_cast<unsigned>(hex - 'A' + 10);
                            else fail("invalid unicode escape");
                        }
                        // Netlist documents only need ASCII identifiers/names; encode
                        // as UTF-8 for completeness without pulling in a full codec.
                        if (codepoint < 0x80) {
                            result.push_back(static_cast<char>(codepoint));
                        } else if (codepoint < 0x800) {
                            result.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
                            result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
                        } else {
                            result.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
                            result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
                            result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
                        }
                        break;
                    }
                    default: fail("invalid escape sequence");
                }
                continue;
            }
            result.push_back(c);
        }
        return result;
    }

    JsonValue parseNumber() {
        const std::size_t start = pos_;
        if (peek() == '-') ++pos_;
        while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        if (!eof() && peek() == '.') {
            ++pos_;
            while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        }
        if (!eof() && (peek() == 'e' || peek() == 'E')) {
            ++pos_;
            if (!eof() && (peek() == '+' || peek() == '-')) ++pos_;
            while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        }
        const std::string token(text_.substr(start, pos_ - start));
        if (token.empty() || token == "-") fail("invalid number literal");
        char* end = nullptr;
        const double value = std::strtod(token.c_str(), &end);
        if (end != token.c_str() + token.size()) fail("invalid number literal");
        return JsonValue::makeNumber(value);
    }
};

} // namespace json_detail

inline JsonValue parseJson(std::string_view text) {
    json_detail::Parser parser(text);
    return parser.parseDocument();
}

} // namespace guitardsp::circuit
