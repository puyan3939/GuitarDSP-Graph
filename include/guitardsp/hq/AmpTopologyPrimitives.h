#pragma once

#include "ADAA.h"
#include "CircuitPrimitives.h"
#include "DeviceStages.h"
#include "PowerTubeModels.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

class CouplingHighpass {
public:
    void prepare(double sampleRate, float cutoffHz) noexcept {
        filter_.prepare(sampleRate);
        filter_.setLowpass(cutoffHz);
        reset();
    }
    void reset() noexcept { filter_.reset(); }
    float process(float x) noexcept { return filter_.processHighpass(x); }
private:
    OnePole filter_;
};

// Three-band passive-stack abstraction used as a topology primitive. It is not a
// replacement for a named Fender/Marshall tone-stack solve; those can later fit
// component values while keeping the same block contract.
class ThreeBandToneStack {
public:
    void prepare(double sampleRate) noexcept {
        low_.prepare(sampleRate); low_.setLowpass(220.0f);
        highLowpass_.prepare(sampleRate); highLowpass_.setLowpass(2200.0f);
        reset();
    }
    void reset() noexcept { low_.reset(); highLowpass_.reset(); }
    void setControls(float bass, float mid, float treble) noexcept {
        bassGain_ = controlGain(bass);
        midGain_ = controlGain(mid);
        trebleGain_ = controlGain(treble);
    }
    float process(float x) noexcept {
        const float low = low_.processLowpass(x);
        const float belowHigh = highLowpass_.processLowpass(x);
        const float high = x - belowHigh;
        const float mid = belowHigh - low;
        return 0.45f * (bassGain_ * low + midGain_ * mid + trebleGain_ * high);
    }
private:
    static float controlGain(float v) noexcept {
        const float db = -12.0f + 24.0f * std::clamp(v, 0.0f, 1.0f);
        return std::pow(10.0f, db / 20.0f);
    }
    OnePole low_, highLowpass_;
    float bassGain_ = 1.0f, midGain_ = 1.0f, trebleGain_ = 1.0f;
};

// Differential phase-inverter abstraction with independent nonlinear halves and
// a shared tail-memory term. It captures imbalance and dynamic common-mode shift
// without pretending to be a measured long-tail-pair tube fit yet.
class LongTailPairPhaseInverter {
public:
    void prepare(double sampleRate) noexcept {
        tail_.prepare(sampleRate);
        tail_.setLowpass(12.0f);
        reset();
    }
    void reset() noexcept {
        positive_.reset();
        negative_.reset();
        tail_.reset();
    }
    void setDrive(float drive) noexcept { drive_ = std::clamp(drive, 0.2f, 12.0f); }
    void setImbalance(float imbalance) noexcept { imbalance_ = std::clamp(imbalance, -0.25f, 0.25f); }
    float process(float x) noexcept {
        const float common = tail_.processLowpass(x);
        const float a = positive_.process(drive_ * (x - 0.10f * common) * (1.0f + imbalance_));
        const float b = negative_.process(-drive_ * (x + 0.10f * common) * (1.0f - imbalance_));
        return 0.5f * (a - b);
    }
private:
    ADAATanh positive_, negative_;
    OnePole tail_;
    float drive_ = 1.6f;
    float imbalance_ = 0.03f;
};

class NegativeFeedbackLoop {
public:
    void prepare(double sampleRate, float bandwidthHz = 6500.0f) noexcept {
        feedback_.prepare(sampleRate);
        feedback_.setLowpass(bandwidthHz);
        reset();
    }
    void reset() noexcept { feedback_.reset(); }
    void setAmount(float amount) noexcept { amount_ = std::clamp(amount, 0.0f, 0.85f); }
    float drive(float input) noexcept { return input - amount_ * feedbackState_; }
    void observe(float output) noexcept { feedbackState_ = feedback_.processLowpass(output); }
private:
    OnePole feedback_;
    float feedbackState_ = 0.0f;
    float amount_ = 0.25f;
};

// Generic push-pull power-stage primitive with envelope-driven supply sag,
// crossover control, a selectable engineering power-tube family model and a
// transformer block. Device families are starting points for later measured fits.
class PushPullPowerStage {
public:
    void prepare(double sampleRate) noexcept {
        envelope_.prepare(sampleRate); envelope_.setLowpass(14.0f);
        transformer_.prepare(sampleRate);
        reset();
    }
    void reset() noexcept { envelope_.reset(); transformer_.reset(); }
    void setDrive(float drive) noexcept { drive_ = std::clamp(drive, 0.2f, 15.0f); }
    void setSag(float sag) noexcept { sag_ = std::clamp(sag, 0.0f, 0.8f); }
    void setCrossover(float crossover) noexcept { crossover_ = std::clamp(crossover, 0.0f, 0.25f); }
    void setTransformerSaturation(float saturation) noexcept { transformer_.setSaturation(saturation); }
    void setTubeModel(PowerTubeModel model) noexcept { tube_ = model; }
    void setTubeType(PowerTubeType type) noexcept { tube_ = PowerTubeModel::forType(type); }

    float process(float x) noexcept {
        const float env = envelope_.processLowpass(std::abs(x));
        const float supply = std::max(0.28f, 1.0f - sag_ * env);
        const float driven = drive_ * x / supply;
        const float upperInput = std::max(0.0f, driven - crossover_);
        const float lowerInput = std::max(0.0f, -driven - crossover_);
        const float upper = tube_.transfer(upperInput, supply);
        const float lower = tube_.transfer(lowerInput, supply);
        return transformer_.process(supply * (upper - lower) / std::max(0.25f, drive_ * 0.72f));
    }
private:
    PowerTubeModel tube_ = PowerTubeModel::forType(PowerTubeType::el34);
    OnePole envelope_;
    TransformerStage transformer_;
    float drive_ = 2.0f;
    float sag_ = 0.12f;
    float crossover_ = 0.01f;
};

} // namespace guitardsp::hq
