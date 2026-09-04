#pragma once

// float32 <-> 8-hex-digit text conversion for the golden reference file
// format (docs/GOLDEN_REFERENCE.md section 4). Golden files store one
// sample per line as the exact IEEE-754 bit pattern in hex, not decimal
// text, so a golden file's bytes never depend on locale, rounding mode or
// printf precision.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace guitardsp::golden {

inline std::uint32_t floatToBits(float value) noexcept {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

inline float bitsToFloat(std::uint32_t bits) noexcept {
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline std::string toHexLine(float value) {
    char buf[9];
    std::snprintf(buf, sizeof(buf), "%08x", floatToBits(value));
    return std::string(buf, 8);
}

// Parses one "%08x" line. Returns false (leaving value unchanged) if the
// line is not exactly 8 hex digits.
inline bool parseHexLine(const std::string& line, float* outValue) {
    if (line.size() != 8) return false;
    std::uint32_t bits = 0;
    for (char c : line) {
        bits <<= 4;
        if (c >= '0' && c <= '9') bits |= static_cast<std::uint32_t>(c - '0');
        else if (c >= 'a' && c <= 'f') bits |= static_cast<std::uint32_t>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') bits |= static_cast<std::uint32_t>(c - 'A' + 10);
        else return false;
    }
    *outValue = bitsToFloat(bits);
    return true;
}

} // namespace guitardsp::golden
