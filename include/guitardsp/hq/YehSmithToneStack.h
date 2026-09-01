#pragma once

#include "ToneStackFamilies.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace guitardsp::hq {

// Exact third-order FMV/Bassman-family tone-stack transfer function based on
// Yeh & Smith, DAFx-06, discretized with the bilinear transform. Component
// values are supplied by ToneStackNominalValues so British/American families
// can share the exact symbolic network while retaining distinct parts values.
class YehSmithToneStack {
public:
    void prepare(double sampleRate, ToneStackFamily family = ToneStackFamily::reference) noexcept {
        sampleRate_ = std::max(1.0, sampleRate);
        family_ = family;
        updateCoefficients();
        reset();
    }

    void reset() noexcept {
        x_.fill(0.0);
        y_.fill(0.0);
    }

    void setFamily(ToneStackFamily family) noexcept {
        if (family_ == family) return;
        family_ = family;
        updateCoefficients();
    }

    void setControls(float bass, float mid, float treble) noexcept {
        const float b = std::clamp(bass, 0.0f, 1.0f);
        const float m = std::clamp(mid, 0.0f, 1.0f);
        const float t = std::clamp(treble, 0.0f, 1.0f);
        if (b == bass_ && m == mid_ && t == treble_) return;
        bass_ = b; mid_ = m; treble_ = t;
        updateCoefficients();
    }

    float process(float input) noexcept {
        const double x0 = static_cast<double>(input);
        const double y0 = b_[0] * x0 + b_[1] * x_[0] + b_[2] * x_[1] + b_[3] * x_[2]
                        - a_[1] * y_[0] - a_[2] * y_[1] - a_[3] * y_[2];
        x_[2] = x_[1]; x_[1] = x_[0]; x_[0] = x0;
        y_[2] = y_[1]; y_[1] = y_[0]; y_[0] = std::isfinite(y0) ? y0 : 0.0;
        return static_cast<float>(y_[0]);
    }

    [[nodiscard]] ToneStackFamily family() const noexcept { return family_; }
    [[nodiscard]] const std::array<double, 4>& numerator() const noexcept { return b_; }
    [[nodiscard]] const std::array<double, 4>& denominator() const noexcept { return a_; }

private:
    struct AnalogCoefficients {
        double b1{}, b2{}, b3{};
        double a0{1.0}, a1{}, a2{}, a3{};
    };

    [[nodiscard]] AnalogCoefficients analogCoefficients() const noexcept {
        const auto v = nominalToneStack(family_);
        const double C1 = static_cast<double>(v.trebleCapFarads);
        const double C2 = static_cast<double>(v.bassCapFarads);
        const double C3 = static_cast<double>(v.midCapFarads);
        const double R1 = static_cast<double>(v.treblePotOhms);
        const double R2 = static_cast<double>(v.bassPotOhms);
        const double R3 = static_cast<double>(v.midPotOhms);
        const double R4 = static_cast<double>(v.slopeResistanceOhms);
        const double t = static_cast<double>(treble_);
        const double m = static_cast<double>(mid_);
        // The paper defines l as the bass-pot resistance fraction. A quadratic
        // control mapping is a useful audio-taper starting point and is kept
        // explicit so measured fitting can replace it later.
        const double l = static_cast<double>(bass_) * static_cast<double>(bass_);

        AnalogCoefficients c;
        c.b1 = t*C1*R1 + m*C3*R3 + l*(C1*R2 + C2*R2) + (C1*R3 + C2*R3);
        c.b2 = t*(C1*C2*R1*R4 + C1*C3*R1*R4)
             - m*m*(C1*C3*R3*R3 + C2*C3*R3*R3)
             + m*(C1*C3*R1*R3 + C1*C3*R3*R3 + C2*C3*R3*R3)
             + l*(C1*C2*R1*R2 + C1*C2*R2*R4 + C1*C3*R2*R4)
             + l*m*(C1*C3*R2*R3 + C2*C3*R2*R3)
             + (C1*C2*R1*R3 + C1*C2*R3*R4 + C1*C3*R3*R4);
        c.b3 = l*m*(C1*C2*C3*R1*R2*R3 + C1*C2*C3*R2*R3*R4)
             - m*m*(C1*C2*C3*R1*R3*R3 + C1*C2*C3*R3*R3*R4)
             + m*(C1*C2*C3*R1*R3*R3 + C1*C2*C3*R3*R3*R4)
             + t*C1*C2*C3*R1*R3*R4 - t*m*C1*C2*C3*R1*R3*R4
             + t*l*C1*C2*C3*R1*R2*R4;

        c.a1 = (C1*R1 + C1*R3 + C2*R3 + C2*R4 + C3*R4) + m*C3*R3 + l*(C1*R2 + C2*R2);
        c.a2 = m*(C1*C3*R1*R3 - C2*C3*R3*R4 + C1*C3*R3*R3 + C2*C3*R3*R3)
             + l*m*(C1*C3*R2*R3 + C2*C3*R2*R3)
             - m*m*(C1*C3*R3*R3 + C2*C3*R3*R3)
             + l*(C1*C2*R2*R4 + C1*C2*R1*R2 + C1*C3*R2*R4 + C2*C3*R2*R4)
             + (C1*C2*R1*R4 + C1*C3*R1*R4 + C1*C2*R3*R4 + C1*C2*R1*R3 + C1*C3*R3*R4 + C2*C3*R3*R4);
        c.a3 = l*m*(C1*C2*C3*R1*R2*R3 + C1*C2*C3*R2*R3*R4)
             - m*m*(C1*C2*C3*R1*R3*R3 + C1*C2*C3*R3*R3*R4)
             + m*(C1*C2*C3*R3*R3*R4 + C1*C2*C3*R1*R3*R3 - C1*C2*C3*R1*R3*R4)
             + l*C1*C2*C3*R1*R2*R4 + C1*C2*C3*R1*R3*R4;
        return c;
    }

    void updateCoefficients() noexcept {
        const auto c = analogCoefficients();
        const double k = 2.0 * sampleRate_;
        const double k2 = k*k;
        const double k3 = k2*k;

        // Expand H(s) under s = k(1-z^-1)/(1+z^-1). This is algebraically
        // equivalent to the DAFx-06 bilinear coefficient form but normalized
        // here to a0 = 1 for direct-form processing.
        const double B0 = c.b1*k + c.b2*k2 + c.b3*k3;
        const double B1 = c.b1*k - c.b2*k2 - 3.0*c.b3*k3;
        const double B2 = -c.b1*k - c.b2*k2 + 3.0*c.b3*k3;
        const double B3 = -c.b1*k + c.b2*k2 - c.b3*k3;
        const double A0 = c.a0 + c.a1*k + c.a2*k2 + c.a3*k3;
        const double A1 = 3.0*c.a0 + c.a1*k - c.a2*k2 - 3.0*c.a3*k3;
        const double A2 = 3.0*c.a0 - c.a1*k - c.a2*k2 + 3.0*c.a3*k3;
        const double A3 = c.a0 - c.a1*k + c.a2*k2 - c.a3*k3;
        const double norm = std::abs(A0) > 1.0e-30 ? 1.0 / A0 : 1.0;
        b_ = {B0*norm, B1*norm, B2*norm, B3*norm};
        a_ = {1.0, A1*norm, A2*norm, A3*norm};
    }

    double sampleRate_ = 48000.0;
    ToneStackFamily family_ = ToneStackFamily::reference;
    float bass_ = 0.5f, mid_ = 0.5f, treble_ = 0.5f;
    std::array<double, 4> b_{};
    std::array<double, 4> a_{{1.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> x_{};
    std::array<double, 3> y_{};
};

} // namespace guitardsp::hq
