#pragma once
#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

// First-order antiderivative anti-aliasing. The primitive is intentionally
// stateless apart from x[n-1], so it can be embedded inside oversampled stages.
class ADAATanh {
public:
    void reset(float x = 0.0f) noexcept { previous_ = x; initialized_ = false; }

    float process(float x) noexcept {
        if (!initialized_) {
            previous_ = x;
            initialized_ = true;
            return std::tanh(x);
        }
        const float dx = x - previous_;
        float y;
        if (std::abs(dx) < 1.0e-5f) {
            const float mid = 0.5f * (x + previous_);
            y = std::tanh(mid);
        } else {
            y = (antiderivative(x) - antiderivative(previous_)) / dx;
        }
        previous_ = x;
        return y;
    }

private:
    static float antiderivative(float x) noexcept {
        const float ax = std::abs(x);
        return ax > 18.0f ? ax : std::log(std::cosh(x));
    }

    float previous_ = 0.0f;
    bool initialized_ = false;
};

class ADAASoftClip {
public:
    void reset(float x = 0.0f) noexcept { previous_ = x; initialized_ = false; }

    float process(float x) noexcept {
        if (!initialized_) {
            previous_ = x;
            initialized_ = true;
            return shape(x);
        }
        const float dx = x - previous_;
        float y;
        if (std::abs(dx) < 1.0e-5f) {
            y = shape(0.5f * (x + previous_));
        } else {
            y = (primitive(x) - primitive(previous_)) / dx;
        }
        previous_ = x;
        return y;
    }

private:
    static float shape(float x) noexcept {
        if (x <= -1.0f) return -2.0f / 3.0f;
        if (x >= 1.0f) return 2.0f / 3.0f;
        return x - (x * x * x) / 3.0f;
    }

    static float primitive(float x) noexcept {
        if (x <= -1.0f) return -(2.0f / 3.0f) * x - 0.25f;
        if (x >= 1.0f) return (2.0f / 3.0f) * x - 0.25f;
        return 0.5f * x * x - (x * x * x * x) / 12.0f;
    }

    float previous_ = 0.0f;
    bool initialized_ = false;
};

} // namespace guitardsp::hq
