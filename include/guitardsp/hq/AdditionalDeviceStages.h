#pragma once

#include "ComponentCatalog.h"
#include "CircuitPrimitives.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

class JFETCommonSourceStage {
public:
    struct Config {
        JFETSpec device = component_presets::j201();
        float supplyVoltage = 9.0f;
        float drainResistanceOhms = 10000.0f;
        float sourceResistanceOhms = 1500.0f;
        float gateBiasVolts = 0.0f;
        float outputScale = 0.35f;
        float sourceMemoryMs = 4.0f;
    };

    void prepare(double sampleRate, const Config& config = {}) noexcept {
        config_ = config;
        source_.setTimeConstant(sampleRate, config_.sourceMemoryMs);
        reset();
        quiescentDrain_ = solveDrainCurrent(config_.gateBiasVolts, 0.0f);
    }

    void reset() noexcept { source_.voltage = 0.0f; }

    float process(float inputVolts) noexcept {
        const float vgs = config_.gateBiasVolts + inputVolts - source_.voltage;
        const float current = drainCurrent(vgs);
        source_.process(current * config_.sourceResistanceOhms);
        const float drain = std::clamp(config_.supplyVoltage - current * config_.drainResistanceOhms,
                                       0.0f, config_.supplyVoltage);
        return (quiescentDrain_ - drain) * config_.outputScale;
    }

private:
    float drainCurrent(float vgs) const noexcept {
        const float vp = std::min(-1.0e-3f, config_.device.pinchOffVoltage);
        if (vgs <= vp) return 0.0f;
        const float ratio = 1.0f - vgs / vp;
        return std::max(0.0f, config_.device.idssAmps * ratio * ratio);
    }

    float solveDrainCurrent(float vgs, float sourceVoltage) const noexcept {
        const float current = drainCurrent(vgs - sourceVoltage);
        return std::clamp(config_.supplyVoltage - current * config_.drainResistanceOhms,
                          0.0f, config_.supplyVoltage);
    }

    Config config_{};
    CathodeBiasState source_{};
    float quiescentDrain_ = 4.5f;
};

class MOSFETCommonSourceStage {
public:
    struct Config {
        MOSFETSpec device = component_presets::bs170();
        float supplyVoltage = 9.0f;
        float drainResistanceOhms = 10000.0f;
        float sourceResistanceOhms = 470.0f;
        float gateBiasVolts = 2.3f;
        float outputScale = 0.18f;
        float sourceMemoryMs = 3.0f;
    };

    void prepare(double sampleRate, const Config& config = {}) noexcept {
        config_ = config;
        source_.setTimeConstant(sampleRate, config_.sourceMemoryMs);
        reset();
        quiescentDrain_ = drainVoltage(config_.gateBiasVolts);
    }

    void reset() noexcept { source_.voltage = 0.0f; }

    float process(float inputVolts) noexcept {
        const float vgs = config_.gateBiasVolts + inputVolts - source_.voltage;
        const float current = drainCurrent(vgs);
        source_.process(current * config_.sourceResistanceOhms);
        const float drain = std::clamp(config_.supplyVoltage - current * config_.drainResistanceOhms,
                                       0.0f, config_.supplyVoltage);
        return (quiescentDrain_ - drain) * config_.outputScale;
    }

private:
    float drainCurrent(float vgs) const noexcept {
        const float overdrive = std::max(0.0f, vgs - config_.device.thresholdVoltage);
        const float ideal = 0.5f * config_.device.transconductance * overdrive * overdrive;
        return std::max(0.0f, ideal * (1.0f + config_.device.lambda * config_.supplyVoltage));
    }

    float drainVoltage(float vgs) const noexcept {
        return std::clamp(config_.supplyVoltage - drainCurrent(vgs) * config_.drainResistanceOhms,
                          0.0f, config_.supplyVoltage);
    }

    Config config_{};
    CathodeBiasState source_{};
    float quiescentDrain_ = 4.5f;
};

class OpAmpStage {
public:
    struct Config {
        OpAmpSpec device = component_presets::jrc4558();
        float positiveRailVolts = 4.5f;
        float negativeRailVolts = -4.5f;
        float closedLoopGain = 1.0f;
    };

    void prepare(double sampleRate, const Config& config = {}) noexcept {
        sampleRate_ = std::max(1.0, sampleRate);
        config_ = config;
        updatePole();
        reset();
    }

    void reset() noexcept {
        state_ = 0.0f;
        previous_ = 0.0f;
    }

    void setClosedLoopGain(float gain) noexcept {
        config_.closedLoopGain = std::clamp(gain, 0.1f, 200.0f);
        updatePole();
    }

    float process(float input) noexcept {
        const float target = input * config_.closedLoopGain + config_.device.inputOffsetVoltage;
        state_ += poleCoefficient_ * (target - state_);

        const float maxStep = config_.device.slewRateVoltsPerSecond / static_cast<float>(sampleRate_);
        const float slewed = previous_ + std::clamp(state_ - previous_, -maxStep, maxStep);

        const float high = config_.positiveRailVolts - config_.device.positiveRailHeadroomVolts;
        const float low = config_.negativeRailVolts + config_.device.negativeRailHeadroomVolts;
        previous_ = std::clamp(slewed, low, high);
        return previous_;
    }

private:
    void updatePole() noexcept {
        const float closedLoopBandwidth = config_.device.gainBandwidthHz /
            std::max(1.0f, std::abs(config_.closedLoopGain));
        poleCoefficient_ = 1.0f - std::exp(-2.0f * 3.14159265359f *
            std::min(closedLoopBandwidth, static_cast<float>(0.45 * sampleRate_)) /
            static_cast<float>(sampleRate_));
    }

    Config config_{};
    double sampleRate_ = 48000.0;
    float poleCoefficient_ = 1.0f;
    float state_ = 0.0f;
    float previous_ = 0.0f;
};

class OptocouplerLDR {
public:
    void prepare(double sampleRate, OptocouplerSpec spec = {}) noexcept {
        sampleRate_ = std::max(1.0, sampleRate);
        spec_ = spec;
        reset();
    }

    void reset() noexcept { illumination_ = 0.0f; }

    float processLedDrive(float normalizedDrive) noexcept {
        const float target = std::clamp(normalizedDrive, 0.0f, 1.0f);
        const float timeMs = target > illumination_ ? spec_.attackMs : spec_.releaseMs;
        const float seconds = std::max(1.0e-5f, timeMs * 0.001f);
        const float coefficient = 1.0f - std::exp(-1.0f / static_cast<float>(sampleRate_ * seconds));
        illumination_ += coefficient * (target - illumination_);
        return resistanceOhms();
    }

    float resistanceOhms() const noexcept {
        const float light = std::pow(std::clamp(illumination_, 0.0f, 1.0f), std::max(0.05f, spec_.gamma));
        const float logDark = std::log(std::max(1.0f, spec_.darkResistanceOhms));
        const float logLight = std::log(std::max(1.0f, spec_.lightResistanceOhms));
        return std::exp(logDark + light * (logLight - logDark));
    }

private:
    double sampleRate_ = 48000.0;
    OptocouplerSpec spec_{};
    float illumination_ = 0.0f;
};

} // namespace guitardsp::hq
