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

    // Plate current plus its exact partials w.r.t. grid and plate voltage,
    // derived in closed form from plateCurrent()'s softplus/power-law chain
    // (see docs/MNA_ACCELERATION.md). Reuses the exp/log1p/pow already needed
    // for the current value itself -- no extra transcendental evaluations --
    // so this replaces stampTriode's two-sided central-difference gm/gp with
    // a single analytic pass. gp's formula mirrors linearizePlate()'s
    // conductance above; gm is the same chain-rule step through d(inner)/dVg
    // instead of d(inner)/dVp. Current is intentionally left unclamped to the
    // device's operating envelope here -- stampTriode applies that clamp (and
    // zeroes the Jacobian outside it, matching what a central difference
    // straddling the clamp would settle to) since the clamp bound is a stamp
    // concern, not a device-model one.
    struct PlateJacobian {
        float current = 0.0f;
        float gm = 0.0f; // dI/dVg
        float gp = 0.0f; // dI/dVp
    };

    PlateJacobian plateCurrentJacobian(float gridVoltage, float plateVoltage) const noexcept {
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

        float gm = 0.0f;
        float gp = 0.0f;
        if (inner > -30.0f && inner < 30.0f && logTerm > 0.0f) {
            const float softplusSlope = exponential / (1.0f + exponential);
            const float base = exponent * current / logTerm * softplusSlope;
            gm = base * (kp / safeRoot);
            if (plateVoltage > 0.0f && root > 1.0f) {
                const float innerSlope = -kp * gridVoltage * safePlate
                    / (root * root * root);
                gp = base * innerSlope;
            }
        }
        return {current, gm, gp};
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

    // Plate current, screen current, and all five partials (dPlate/d{Vg,Vp,Vs},
    // dScreen/d{Vg,Vs}) from one closed-form evaluation, replacing
    // stampPentode's 12 central-difference calls into plateCurrent()/
    // screenCurrent() (each of which recomputes driveTerm's exp/log1p) with a
    // single shared driveTerm evaluation plus the two pow() calls the current
    // values need anyway. Derived by chain rule through driveTerm's softplus
    // (dL/dVg, dL/dVs mirror TriodeModel::plateCurrentJacobian's gm/gp, with
    // plate voltage replaced by screen voltage) and, for plate current, the
    // atan "knee" term's own derivative w.r.t. plate voltage. See
    // docs/MNA_ACCELERATION.md for the full derivation. Currents are left
    // unclamped to the device envelope here; stampPentode applies that clamp.
    struct CurrentJacobian {
        float plateCurrent = 0.0f;
        float screenCurrent = 0.0f;
        float dPlateDVg = 0.0f;
        float dPlateDVp = 0.0f;
        float dPlateDVs = 0.0f;
        float dScreenDVg = 0.0f;
        float dScreenDVs = 0.0f;
    };

    CurrentJacobian currentsAndJacobian(float gridToCathode, float plateToCathode,
                                        float screenToCathode) const noexcept {
        const float safeScreen = std::max(0.0f, screenToCathode);
        const float rootS = std::sqrt(kvb1 + safeScreen * safeScreen);
        const float safeRootS = std::max(1.0f, rootS);
        const float inner = kp * (1.0f / std::max(1.0f, mu)
                                + gridToCathode / safeRootS);
        const float limited = std::clamp(inner, -30.0f, 30.0f);
        const float exponential = std::exp(limited);
        const float logTerm = std::log1p(exponential);
        const float clampedLog = std::max(0.0f, logTerm);

        const float kg1Safe = std::max(1.0f, kg1);
        const float kg2Safe = std::max(1.0f, kg2);
        const float spaceCharge = 2.0f * std::pow(clampedLog, exponent) / kg1Safe;
        const float screen = 2.0f * std::pow(clampedLog, exponent) / kg2Safe;

        float dLdVg = 0.0f;
        float dLdVs = 0.0f;
        if (inner > -30.0f && inner < 30.0f && logTerm > 0.0f) {
            const float softplusSlope = exponential / (1.0f + exponential);
            dLdVg = softplusSlope * (kp / safeRootS);
            if (screenToCathode > 0.0f && rootS > 1.0f) {
                dLdVs = softplusSlope
                    * (-kp * gridToCathode * safeScreen / (rootS * rootS * rootS));
            }
        }

        constexpr float twoOverPi = 0.63661977236758134f;
        const float kvbSafe = std::max(1.0e-3f, kvb);
        const float safePlate = std::max(0.0f, plateToCathode);
        const float kneeArg = safePlate / kvbSafe;
        const float knee = std::atan(kneeArg) * twoOverPi;

        float dKneeDVp = 0.0f;
        if (plateToCathode > 0.0f) {
            dKneeDVp = twoOverPi / (kvbSafe * (1.0f + kneeArg * kneeArg));
        }

        float dSpaceDL = 0.0f;
        float dScreenDL = 0.0f;
        if (clampedLog > 0.0f) {
            dSpaceDL = exponent * spaceCharge / clampedLog;
            dScreenDL = exponent * screen / clampedLog;
        }

        CurrentJacobian result;
        result.plateCurrent = spaceCharge * knee;
        result.screenCurrent = screen;
        result.dPlateDVg = dSpaceDL * dLdVg * knee;
        result.dPlateDVp = spaceCharge * dKneeDVp;
        result.dPlateDVs = dSpaceDL * dLdVs * knee;
        result.dScreenDVg = dScreenDL * dLdVg;
        result.dScreenDVs = dScreenDL * dLdVs;
        return result;
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
