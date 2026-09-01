#pragma once
#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

struct DiodePairModel {
    float saturationCurrent = 2.0e-9f;
    float thermalVoltage = 0.026f;
    float seriesResistance = 1000.0f;
    float positiveScale = 1.0f;
    float negativeScale = 1.0f;
};

class ImplicitDiodeClipper {
public:
    void setModel(DiodePairModel model) noexcept { model_ = model; }
    void reset(float y = 0.0f) noexcept { previous_ = y; }

    float process(float input) noexcept {
        const float r = std::max(1.0f, model_.seriesResistance);
        const float vt = std::max(0.005f, model_.thermalVoltage);
        const float is = std::max(1.0e-15f, model_.saturationCurrent);
        float y = std::clamp(previous_, -4.0f, 4.0f);

        for (int iteration = 0; iteration < 8; ++iteration) {
            const float ep = safeExp(y / (vt * std::max(0.1f, model_.positiveScale)));
            const float en = safeExp(-y / (vt * std::max(0.1f, model_.negativeScale)));
            const float diodeCurrent = is * ((ep - 1.0f) - (en - 1.0f));
            const float f = y + r * diodeCurrent - input;
            const float derivative = 1.0f + r * is * (
                ep / (vt * std::max(0.1f, model_.positiveScale)) +
                en / (vt * std::max(0.1f, model_.negativeScale)));
            const float step = f / std::max(1.0e-6f, derivative);
            y -= std::clamp(step, -0.5f, 0.5f);
            if (std::abs(step) < 1.0e-6f) break;
        }

        previous_ = std::clamp(y, -8.0f, 8.0f);
        return previous_;
    }

private:
    static float safeExp(float x) noexcept { return std::exp(std::clamp(x, -20.0f, 20.0f)); }
    DiodePairModel model_{};
    float previous_ = 0.0f;
};

} // namespace guitardsp::hq
