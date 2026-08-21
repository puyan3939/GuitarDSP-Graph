#pragma once
#include "ADAA.h"
#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

class OnePole {
public:
    void prepare(double sampleRate) noexcept { sampleRate_ = std::max(1.0, sampleRate); setLowpass(1000.0f); }
    void reset(float value = 0.0f) noexcept { z_ = value; }
    void setLowpass(float hz) noexcept {
        const double f = std::clamp<double>(hz, 1.0, 0.49 * sampleRate_);
        a_ = static_cast<float>(std::exp(-2.0 * 3.14159265358979323846 * f / sampleRate_));
    }
    float processLowpass(float x) noexcept { z_ = (1.0f - a_) * x + a_ * z_; return z_; }
    float processHighpass(float x) noexcept { return x - processLowpass(x); }
private:
    double sampleRate_ = 48000.0;
    float a_ = 0.9f, z_ = 0.0f;
};

// Compact dynamic triode building block. This is not a named tube fit yet;
// it models asymmetric transfer, bias memory and cathode/supply compression
// separately so measured tube fits can replace each term later.
class DynamicTriodeStage {
public:
    void prepare(double sampleRate) noexcept {
        biasMemory_.prepare(sampleRate); biasMemory_.setLowpass(7.0f);
        envelope_.prepare(sampleRate); envelope_.setLowpass(18.0f);
        reset();
    }
    void reset() noexcept { biasMemory_.reset(); envelope_.reset(); positive_.reset(); negative_.reset(); }
    void setDrive(float v) noexcept { drive_ = std::clamp(v, 0.1f, 20.0f); }
    void setAsymmetry(float v) noexcept { asymmetry_ = std::clamp(v, -0.8f, 0.8f); }
    void setSag(float v) noexcept { sag_ = std::clamp(v, 0.0f, 0.8f); }

    float process(float x) noexcept {
        const float memory = biasMemory_.processLowpass(x);
        const float env = envelope_.processLowpass(std::abs(x));
        const float supply = std::max(0.2f, 1.0f - sag_ * env);
        const float bias = asymmetry_ * (0.12f + 0.08f * memory);
        const float v = drive_ * x / supply + bias;
        const float pos = positive_.process(v);
        const float neg = negative_.process(-0.92f * v);
        return supply * 0.52f * (pos - neg);
    }

private:
    OnePole biasMemory_, envelope_;
    ADAATanh positive_, negative_;
    float drive_ = 1.0f, asymmetry_ = 0.1f, sag_ = 0.05f;
};

class TransformerStage {
public:
    void prepare(double sampleRate) noexcept { low_.prepare(sampleRate); low_.setLowpass(120.0f); reset(); }
    void reset() noexcept { low_.reset(); saturation_.reset(); }
    void setSaturation(float value) noexcept { saturationAmount_ = std::clamp(value, 0.0f, 1.0f); }
    float process(float x) noexcept {
        const float lf = low_.processLowpass(x);
        const float shaped = x + 0.10f * saturationAmount_ * lf;
        const float drive = 1.0f + 3.0f * saturationAmount_;
        const float sat = saturation_.process(shaped * drive) / drive;
        return (1.0f - 0.35f * saturationAmount_) * shaped + 0.35f * saturationAmount_ * sat;
    }
private:
    OnePole low_;
    ADAATanh saturation_;
    float saturationAmount_ = 0.2f;
};

} // namespace guitardsp::hq
