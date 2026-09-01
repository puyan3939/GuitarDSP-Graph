#pragma once

#include "DeviceStages.h"
#include "PolyphaseOversampler.h"
#include "QualityPolicy.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

// Reusable two-transistor feedback fuzz topology.
//
// This is intended as the common foundation for Fuzz Face-derived families and
// later device-specific measured fits. It explicitly models two discrete gain
// stages, one-sample feedback, bias shift and supply-starve-like headroom loss.
class TwoTransistorFuzzNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Two-Transistor Fuzz"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        const auto q = qualityFor(spec.quality, category());
        oversampler_.prepare(spec.channels, spec.maximumBlockSize, q.oversamplingFactor, q.resamplerTaps);
        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));
        q1_.assign(channels, {});
        q2_.assign(channels, {});
        inputHp_.assign(channels, {});
        outputLp_.assign(channels, {});
        feedback_.assign(channels, 0.0f);

        BJTCommonEmitterStage::Config first;
        first.supplyVoltage = 9.0f;
        first.collectorResistance = 33000.0f;
        first.emitterResistance = 470.0f;
        first.baseBias = 0.69f;
        first.outputScale = 0.14f;
        first.emitterMemoryMs = 8.0f;

        BJTCommonEmitterStage::Config second;
        second.supplyVoltage = 9.0f;
        second.collectorResistance = 8200.0f;
        second.emitterResistance = 100.0f;
        second.baseBias = 0.70f;
        second.outputScale = 0.20f;
        second.emitterMemoryMs = 4.0f;

        const double highRate = spec.sampleRate * static_cast<double>(q.oversamplingFactor);
        for (auto& s : q1_) s.prepare(highRate, first);
        for (auto& s : q2_) s.prepare(highRate, second);
        for (auto& f : inputHp_) f.setHighpass(highRate, 14.0f);
        for (auto& f : outputLp_) f.setLowpass(highRate, 12000.0f);
        reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
        for (auto& s : q1_) s.reset();
        for (auto& s : q2_) s.reset();
        for (auto& f : inputHp_) f.reset();
        for (auto& f : outputLp_) f.reset();
        std::fill(feedback_.begin(), feedback_.end(), 0.0f);
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float fuzz = fuzz_.load(std::memory_order_relaxed);
        const float bias = bias_.load(std::memory_order_relaxed);
        const float starve = starve_.load(std::memory_order_relaxed);
        const float level = std::pow(10.0f, levelDb_.load(std::memory_order_relaxed) / 20.0f);

        const float feedbackAmount = 0.08f + 0.48f * fuzz;
        const float q2Drive = 0.9f + 6.5f * fuzz * fuzz;
        const float biasOffset = (bias - 0.5f) * 0.34f;
        const float headroom = 1.0f - 0.72f * starve;
        const float starvationBias = starve * (0.18f - 0.36f * bias);

        oversampler_.process(input, output, numSamples, [&](int ch, float x) noexcept {
            const auto i = static_cast<std::size_t>(ch);
            const float dry = inputHp_[i].process(x);

            // Feedback from Q2 into Q1 is intentionally delayed by one oversampled
            // sample. This keeps the realtime solve bounded while preserving the
            // strongly interactive feedback/bias behavior of the topology family.
            const float q1Input = dry - feedbackAmount * feedback_[i];
            const float first = q1_[i].process(q1Input * (0.55f + 0.95f * fuzz));
            const float secondInput = first * q2Drive + biasOffset + starvationBias;
            const float second = q2_[i].process(secondInput);

            feedback_[i] = std::clamp(second, -2.0f, 2.0f);
            const float starved = headroom * second + (1.0f - headroom) * std::tanh(3.0f * second);
            return level * outputLp_[i].process(starved);
        });
    }

    int latencySamples() const noexcept override { return oversampler_.latencySamples(); }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) {
            case 0: return fuzz_.load(std::memory_order_relaxed);
            case 1: return bias_.load(std::memory_order_relaxed);
            case 2: return starve_.load(std::memory_order_relaxed);
            case 3: return levelDb_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float v) noexcept override {
        if (i >= descriptors_.size()) return false;
        v = graph::clampParameter(descriptors_[i], v);
        switch (i) {
            case 0: fuzz_.store(v, std::memory_order_relaxed); break;
            case 1: bias_.store(v, std::memory_order_relaxed); break;
            case 2: starve_.store(v, std::memory_order_relaxed); break;
            case 3: levelDb_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    class OnePole {
    public:
        void setLowpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::lowpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz
                                     / static_cast<float>(std::max(1.0, sampleRate)));
            a_ = x; b_ = 1.0f - x;
        }
        void setHighpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::highpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz
                                     / static_cast<float>(std::max(1.0, sampleRate)));
            a_ = x; b_ = 1.0f - x;
        }
        void reset() noexcept { state_ = 0.0f; }
        float process(float x) noexcept {
            state_ = b_ * x + a_ * state_;
            return mode_ == Mode::lowpass ? state_ : x - state_;
        }
    private:
        enum class Mode { lowpass, highpass };
        Mode mode_ = Mode::lowpass;
        float a_ = 0.0f, b_ = 1.0f, state_ = 0.0f;
    };

    PolyphaseOversampler oversampler_;
    std::vector<BJTCommonEmitterStage> q1_, q2_;
    std::vector<OnePole> inputHp_, outputLp_;
    std::vector<float> feedback_;

    std::atomic<float> fuzz_{0.65f};
    std::atomic<float> bias_{0.50f};
    std::atomic<float> starve_{0.0f};
    std::atomic<float> levelDb_{-8.0f};

    static constexpr std::array<graph::ParameterDescriptor, 4> descriptors_{{
        {"fuzz", "Fuzz", 0.0f, 1.0f, 0.65f, graph::ParameterUnit::percent, 1.0f},
        {"bias", "Bias", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"starve", "Starve", 0.0f, 1.0f, 0.0f, graph::ParameterUnit::percent, 1.0f},
        {"level", "Level", -30.0f, 12.0f, -8.0f, graph::ParameterUnit::decibels, 1.0f}
    }};
};

} // namespace guitardsp::hq
