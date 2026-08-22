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
            // log(cosh(x)) is O(x^2). Evaluating it in float first rounds
            // cosh(x) to one for quiet audio, then dividing two quantized
            // primitives by dx turns that cancellation into audible noise.
            // Keep the complete divided difference in double precision. The
            // small-signal series is both more accurate and cheaper than two
            // log/cosh evaluations in the region where the old path failed.
            const double current = static_cast<double>(x);
            const double previous = static_cast<double>(previous_);
            y = static_cast<float>((antiderivative(current) - antiderivative(previous))
                                   / (current - previous));
        }
        previous_ = x;
        return y;
    }

private:
    static double antiderivative(double x) noexcept {
        const double ax = std::abs(x);
        if (ax < 0.25) {
            const double square = x * x;
            return square * (0.5 + square * (-1.0 / 12.0
                + square * (1.0 / 45.0 + square * (-17.0 / 2520.0
                + square * (31.0 / 14175.0 - square * 691.0 / 935550.0)))));
        }
        return ax > 18.0 ? ax : std::log(std::cosh(x));
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
