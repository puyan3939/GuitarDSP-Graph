#pragma once

#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

enum class DiodeType { silicon, germanium, led };

struct DiodeModel {
    float saturationCurrent = 2.0e-9f;
    float emission = 1.9f;
    float thermalVoltage = 0.02585f;

    static DiodeModel forType(DiodeType type) noexcept {
        switch (type) {
            case DiodeType::germanium: return {2.0e-6f, 1.1f, 0.02585f};
            case DiodeType::led: return {1.0e-12f, 2.2f, 0.02585f};
            case DiodeType::silicon:
            default: return {2.0e-9f, 1.9f, 0.02585f};
        }
    }

    float current(float voltage) const noexcept {
        const float denom = std::max(1.0e-6f, emission * thermalVoltage);
        const float arg = std::clamp(voltage / denom, -40.0f, 40.0f);
        return saturationCurrent * (std::exp(arg) - 1.0f);
    }

    float conductance(float voltage) const noexcept {
        const float denom = std::max(1.0e-6f, emission * thermalVoltage);
        const float arg = std::clamp(voltage / denom, -40.0f, 40.0f);
        return saturationCurrent * std::exp(arg) / denom;
    }
};

struct BJTModel {
    float beta = 180.0f;
    float vbe = 0.62f;
    float thermalVoltage = 0.02585f;
    float saturationVoltage = 0.18f;

    float collectorCurrent(float baseEmitterVoltage, float emitterDegenerationOhms = 0.0f) const noexcept {
        const float excess = std::max(0.0f, baseEmitterVoltage - vbe);
        if (excess <= 0.0f) return 0.0f;
        const float gmCurrent = excess / std::max(1.0f, emitterDegenerationOhms + thermalVoltage * 1000.0f / std::max(1.0f, beta));
        return std::max(0.0f, gmCurrent);
    }

    float saturateCollector(float collectorVoltage) const noexcept {
        return std::max(saturationVoltage, collectorVoltage);
    }
};

struct TriodeModel {
    float mu = 100.0f;
    float kg1 = 1060.0f;
    float kp = 600.0f;
    float kvb = 300.0f;
    float exponent = 1.4f;

    static TriodeModel twelveAX7() noexcept { return {}; }
    static TriodeModel twelveAT7() noexcept { return {60.0f, 900.0f, 420.0f, 260.0f, 1.35f}; }

    float plateCurrent(float gridVoltage, float plateVoltage) const noexcept {
        const float safePlate = std::max(0.0f, plateVoltage);
        const float root = std::sqrt(kvb + safePlate * safePlate);
        const float inner = std::max(0.0f, kp * (1.0f / std::max(1.0f, mu) + gridVoltage / std::max(1.0f, root)));
        const float logTerm = std::log1p(std::exp(std::clamp(inner, -30.0f, 30.0f)));
        return 2.0f * std::pow(std::max(0.0f, logTerm), exponent) / std::max(1.0f, kg1);
    }
};

struct CathodeBiasState {
    float voltage = 0.0f;
    float coefficient = 0.001f;

    void setTimeConstant(double sampleRate, float ms) noexcept {
        const float seconds = std::max(1.0e-5f, ms * 0.001f);
        coefficient = 1.0f - std::exp(-1.0f / static_cast<float>(sampleRate * seconds));
    }

    float process(float target) noexcept {
        voltage += coefficient * (target - voltage);
        return voltage;
    }
};

} // namespace guitardsp::hq
