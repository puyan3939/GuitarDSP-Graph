#pragma once

#include "Components.h"
#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

class DiodePairSolver {
public:
    void setPositive(DiodeModel model) noexcept { positive_ = model; }
    void setNegative(DiodeModel model) noexcept { negative_ = model; }
    void setSeriesResistance(float ohms) noexcept { seriesResistance_ = std::max(1.0f, ohms); }
    void reset(float voltage = 0.0f) noexcept { previous_ = voltage; }

    float process(float inputVoltage) noexcept {
        float y = std::clamp(previous_, -8.0f, 8.0f);
        for (int iteration = 0; iteration < 10; ++iteration) {
            const float current = positive_.current(y) - negative_.current(-y);
            const float derivative = 1.0f + seriesResistance_ * (positive_.conductance(y) + negative_.conductance(-y));
            const float residual = y + seriesResistance_ * current - inputVoltage;
            const float step = residual / std::max(1.0e-6f, derivative);
            y -= std::clamp(step, -0.75f, 0.75f);
            if (std::abs(step) < 1.0e-7f) break;
        }
        previous_ = std::clamp(y, -8.0f, 8.0f);
        return previous_;
    }

private:
    DiodeModel positive_ = DiodeModel::forType(DiodeType::silicon);
    DiodeModel negative_ = DiodeModel::forType(DiodeType::silicon);
    float seriesResistance_ = 1000.0f;
    float previous_ = 0.0f;
};

class TriodeCommonCathodeStage {
public:
    struct Config {
        TriodeModel tube = TriodeModel::twelveAX7();
        float supplyVoltage = 300.0f;
        float plateResistance = 100000.0f;
        float cathodeResistance = 1500.0f;
        float gridBias = -1.2f;
        float outputScale = 0.012f;
        float cathodeMemoryMs = 35.0f;
    };

    void prepare(double sampleRate, Config config = {}) noexcept {
        config_ = config;
        cathode_.setTimeConstant(sampleRate, config_.cathodeMemoryMs);
        reset();
        quiescentPlate_ = solvePlate(config_.gridBias, 0.0f, config_.supplyVoltage * 0.6f);
    }

    void reset() noexcept {
        cathode_.voltage = 0.0f;
        previousPlate_ = config_.supplyVoltage * 0.6f;
    }

    float process(float gridSignalVolts) noexcept {
        const float effectiveGrid = config_.gridBias + gridSignalVolts - cathode_.voltage;
        const float plate = solvePlate(effectiveGrid, cathode_.voltage, previousPlate_);
        const float current = config_.tube.plateCurrent(effectiveGrid, plate);
        const float targetCathode = current * config_.cathodeResistance;
        cathode_.process(targetCathode);
        previousPlate_ = plate;
        return (quiescentPlate_ - plate) * config_.outputScale;
    }

    float cathodeVoltage() const noexcept { return cathode_.voltage; }
    float plateVoltage() const noexcept { return previousPlate_; }

private:
    float solvePlate(float grid, float /*cathode*/, float initial) const noexcept {
        float plate = std::clamp(initial, 1.0f, config_.supplyVoltage);
        for (int iteration = 0; iteration < 8; ++iteration) {
            const float current = config_.tube.plateCurrent(grid, plate);
            const float f = plate + config_.plateResistance * current - config_.supplyVoltage;
            const float delta = std::max(0.01f, plate * 0.0005f);
            const float i2 = config_.tube.plateCurrent(grid, std::min(config_.supplyVoltage, plate + delta));
            const float derivative = 1.0f + config_.plateResistance * (i2 - current) / delta;
            const float step = f / std::max(0.05f, derivative);
            plate -= std::clamp(step, -40.0f, 40.0f);
            plate = std::clamp(plate, 1.0f, config_.supplyVoltage);
            if (std::abs(step) < 1.0e-4f) break;
        }
        return plate;
    }

    Config config_{};
    CathodeBiasState cathode_{};
    float previousPlate_ = 180.0f;
    float quiescentPlate_ = 180.0f;
};

class BJTCommonEmitterStage {
public:
    struct Config {
        BJTModel transistor{};
        float supplyVoltage = 9.0f;
        float collectorResistance = 4700.0f;
        float emitterResistance = 680.0f;
        float baseBias = 0.72f;
        float outputScale = 0.35f;
        float emitterMemoryMs = 6.0f;
    };

    void prepare(double sampleRate, Config config = {}) noexcept {
        config_ = config;
        emitter_.setTimeConstant(sampleRate, config_.emitterMemoryMs);
        emitter_.voltage = 0.0f;
        const float iq = config_.transistor.collectorCurrent(config_.baseBias, config_.emitterResistance);
        quiescentCollector_ = std::clamp(config_.supplyVoltage - iq * config_.collectorResistance,
                                         config_.transistor.saturationVoltage, config_.supplyVoltage);
    }

    void reset() noexcept { emitter_.voltage = 0.0f; }

    float process(float inputVolts) noexcept {
        const float vbe = config_.baseBias + inputVolts - emitter_.voltage;
        const float ic = config_.transistor.collectorCurrent(vbe, config_.emitterResistance);
        const float collector = std::clamp(config_.supplyVoltage - ic * config_.collectorResistance,
                                           config_.transistor.saturationVoltage, config_.supplyVoltage);
        emitter_.process(ic * config_.emitterResistance);
        return (quiescentCollector_ - collector) * config_.outputScale;
    }

private:
    Config config_{};
    CathodeBiasState emitter_{};
    float quiescentCollector_ = 4.5f;
};

} // namespace guitardsp::hq
