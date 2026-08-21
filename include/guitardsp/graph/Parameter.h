#pragma once

#include <algorithm>
#include <cstddef>
#include <string_view>

namespace guitardsp::graph {

enum class ParameterUnit { generic, decibels, hertz, milliseconds, percent, semitones };

struct ParameterDescriptor {
    std::string_view id;
    std::string_view name;
    float minimum = 0.0f;
    float maximum = 1.0f;
    float defaultValue = 0.0f;
    ParameterUnit unit = ParameterUnit::generic;
    float skew = 1.0f;
};

inline float clampParameter(const ParameterDescriptor& descriptor, float value) noexcept {
    return std::clamp(value, descriptor.minimum, descriptor.maximum);
}

} // namespace guitardsp::graph
