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
    struct PlateLinearization {
        float current = 0.0f;
        float conductance = 0.0f;
    };

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
        // Keep the signed grid term: softplus already guarantees positive plate
        // current. Clamping its input to zero makes every sufficiently negative
        // grid voltage produce the same current, eliminating transconductance
        // after the cathode self-bias settles and silencing the whole amplifier.
        const float inner = kp * (1.0f / std::max(1.0f, mu)
                                + gridVoltage / std::max(1.0f, root));
        const float logTerm = std::log1p(std::exp(std::clamp(inner, -30.0f, 30.0f)));
        return 2.0f * std::pow(std::max(0.0f, logTerm), exponent) / std::max(1.0f, kg1);
    }

    PlateLinearization linearizePlate(float gridVoltage, float plateVoltage) const noexcept {
        const float safePlate = std::max(0.0f, plateVoltage);
        const float root = std::sqrt(kvb + safePlate * safePlate);
        const float safeRoot = std::max(1.0f, root);
        const float inner = kp * (1.0f / std::max(1.0f, mu)
                                + gridVoltage / safeRoot);
        const float limited = std::clamp(inner, -30.0f, 30.0f);
        const float exponential = std::exp(limited);
        const float logTerm = std::log1p(exponential);
        const float current = 2.0f * std::pow(std::max(0.0f, logTerm), exponent)
            / std::max(1.0f, kg1);

        float conductance = 0.0f;
        if (plateVoltage > 0.0f && root > 1.0f
            && inner > -30.0f && inner < 30.0f && logTerm > 0.0f) {
            const float innerSlope = -kp * gridVoltage * safePlate
                / (root * root * root);
            const float softplusSlope = exponential / (1.0f + exponential);
            conductance = exponent * current / logTerm * softplusSlope * innerSlope;
        }
        return {current, conductance};
    }
};

// Koren-style pentode/beam-tetrode power tube model, extending TriodeModel's
// softplus + power-law structure to the 4-terminal plate/grid/screen/cathode
// device. Reference: N. Koren, "Improved Vacuum-Tube Models for SPICE
// Simulations", audioXpress / Vacuum Tube Valley, 1996
// (http://www.normankoren.com/Audio/Tubemodspice_article.html). Koren's
// pentode variant replaces the triode's plate-voltage drive term with the
// screen voltage (plate current becomes largely independent of plate
// voltage, the defining pentode trait); the plate-voltage dependence that
// remains is a soft "knee" governed by kvb, and screen current is a second,
// separately-partitioned tap on the same space-charge term. As with
// TriodeModel::plateCurrent, the presets here are engineering-approximate
// starting points for circuit modelling, not manufacturer datasheet fits.
struct PentodeModel {
    float mu = 4.0f;
    float kg1 = 1100.0f;
    float kg2 = 4200.0f;
    float kp = 60.0f;
    float kvb = 24.0f;
    float kvb1 = 300.0f;
    float exponent = 1.35f;

    static PentodeModel el34() noexcept { return {}; }
    static PentodeModel sixL6GC() noexcept { return {3.5f, 1250.0f, 5200.0f, 48.0f, 18.0f, 260.0f, 1.35f}; }
    static PentodeModel kt88() noexcept { return {4.5f, 980.0f, 3800.0f, 52.0f, 20.0f, 320.0f, 1.35f}; }

    // Shared space-charge drive term (Koren's "E1"), softplus of the grid
    // drive normalized by the screen voltage instead of the plate voltage.
    float driveTerm(float gridToCathode, float screenToCathode) const noexcept {
        const float safeScreen = std::max(0.0f, screenToCathode);
        // sqrt(kvb1 + safeScreen^2) protects against dividing by a collapsed
        // screen voltage exactly like TriodeModel::plateCurrent's
        // sqrt(kvb + safePlate^2) protects against a collapsed plate voltage.
        const float root = std::sqrt(kvb1 + safeScreen * safeScreen);
        const float inner = kp * (1.0f / std::max(1.0f, mu)
                                + gridToCathode / std::max(1.0f, root));
        return std::log1p(std::exp(std::clamp(inner, -30.0f, 30.0f)));
    }

    float plateCurrent(float gridToCathode, float plateToCathode, float screenToCathode) const noexcept {
        const float logTerm = driveTerm(gridToCathode, screenToCathode);
        const float spaceCharge = 2.0f * std::pow(std::max(0.0f, logTerm), exponent) / std::max(1.0f, kg1);
        // Pentode plate current collapses to zero as Vpk -> 0 (no field to
        // draw electrons across the screen) and saturates once Vpk is a few
        // multiples of kvb, essentially independent of further plate voltage
        // increases. atan(..) * 2/pi reproduces that 0 -> 1 soft knee shape.
        constexpr float twoOverPi = 0.63661977236758134f;
        const float safePlate = std::max(0.0f, plateToCathode);
        const float knee = std::atan(safePlate / std::max(1.0e-3f, kvb)) * twoOverPi;
        return spaceCharge * knee;
    }

    float screenCurrent(float gridToCathode, float screenToCathode) const noexcept {
        const float logTerm = driveTerm(gridToCathode, screenToCathode);
        // The screen grid intercepts a roughly plate-voltage-independent
        // fraction of the same space-charge beam, so screen current reuses
        // driveTerm but is partitioned by its own perveance kg2 rather than
        // gated by the plate-voltage knee used for plate current.
        return 2.0f * std::pow(std::max(0.0f, logTerm), exponent) / std::max(1.0f, kg2);
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
