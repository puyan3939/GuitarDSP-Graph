#pragma once

#include <algorithm>
#include <cmath>

namespace guitardsp::dsp {

class OnePoleState {
public:
    void prepare(double sampleRate) noexcept { sampleRate_ = sampleRate; reset(); }
    void reset() noexcept { z_ = 0.0f; }
    float lowPass(float x, float cutoffHz) noexcept {
        const float hz = std::clamp(cutoffHz, 5.0f, static_cast<float>(0.45 * sampleRate_));
        const float a = std::exp(-2.0f * pi * hz / static_cast<float>(sampleRate_));
        z_ = (1.0f - a) * x + a * z_;
        return z_;
    }
    float highPass(float x, float cutoffHz) noexcept { return x - lowPass(x, cutoffHz); }
private:
    static constexpr float pi = 3.14159265358979323846f;
    double sampleRate_ = 48000.0;
    float z_ = 0.0f;
};

class DcBlocker {
public:
    void prepare(double sampleRate, float cutoffHz = 15.0f) noexcept {
        const float x = std::exp(-2.0f * pi * cutoffHz / static_cast<float>(sampleRate));
        r_ = std::clamp(x, 0.0f, 0.99999f); reset();
    }
    void reset() noexcept { x1_ = y1_ = 0.0f; }
    float process(float x) noexcept {
        const float y = x - x1_ + r_ * y1_;
        x1_ = x; y1_ = y; return y;
    }
private:
    static constexpr float pi = 3.14159265358979323846f;
    float r_ = 0.995f, x1_ = 0.0f, y1_ = 0.0f;
};

inline float solveAntiparallelDiodes(float sourceV, float seriesResistance,
                                     float saturationCurrent = 2.0e-9f,
                                     float thermalVoltageTimesN = 0.052f) noexcept {
    const float rs = std::max(1.0f, seriesResistance);
    const float vt = std::max(0.01f, thermalVoltageTimesN);
    float v = std::clamp(sourceV, -1.2f, 1.2f);
    for (int iteration = 0; iteration < 5; ++iteration) {
        const float arg = std::clamp(v / vt, -20.0f, 20.0f);
        const float sinhArg = std::sinh(arg);
        const float coshArg = std::cosh(arg);
        const float current = 2.0f * saturationCurrent * sinhArg;
        const float f = v + rs * current - sourceV;
        const float derivative = 1.0f + rs * 2.0f * saturationCurrent * coshArg / vt;
        v -= f / std::max(1.0e-5f, derivative);
    }
    return v;
}

} // namespace guitardsp::dsp
