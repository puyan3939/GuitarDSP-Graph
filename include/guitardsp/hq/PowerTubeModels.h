#pragma once

#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

enum class PowerTubeType { el34, sixL6GC, kt88 };

struct PowerTubeModel {
    float transconductance = 0.0105f;
    float kneeVoltage = 0.22f;
    float saturation = 1.0f;
    float asymmetry = 0.04f;
    float screenCompression = 0.10f;

    static PowerTubeModel forType(PowerTubeType type) noexcept {
        switch (type) {
            case PowerTubeType::sixL6GC:
                return {0.0088f, 0.18f, 1.12f, 0.025f, 0.075f};
            case PowerTubeType::kt88:
                return {0.0122f, 0.16f, 1.30f, 0.018f, 0.060f};
            case PowerTubeType::el34:
            default:
                return {0.0105f, 0.22f, 1.00f, 0.045f, 0.115f};
        }
    }

    float transfer(float gridDrive, float supplyScale) const noexcept {
        const float supply = std::clamp(supplyScale, 0.20f, 1.20f);
        const float polarityScale = gridDrive >= 0.0f ? 1.0f + asymmetry : 1.0f - asymmetry;
        const float drive = transconductance * 120.0f * polarityScale * gridDrive;
        const float compressed = drive / (1.0f + screenCompression * std::abs(drive) / supply);
        const float knee = std::max(0.02f, kneeVoltage * supply);
        const float shaped = std::tanh(compressed / knee) * knee;
        return saturation * supply * shaped;
    }
};

} // namespace guitardsp::hq
