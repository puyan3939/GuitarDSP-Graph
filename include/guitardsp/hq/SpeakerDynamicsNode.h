#pragma once

#include "CircuitPrimitives.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace guitardsp::hq {

// Nonlinear speaker-dynamics layer intended to sit before/after a linear cabinet
// IR depending on the chosen model architecture. It models level-dependent voice
// coil compression, excursion softening and low-frequency resonance memory.
// It is an engineering primitive, not a named speaker fit.
class SpeakerDynamicsNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Speaker Dynamics"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::cab; }

    void prepare(const graph::PrepareSpec& spec) override {
        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));
        envelope_.assign(channels, {});
        low_.assign(channels, {});
        softener_.assign(channels, {});
        for (std::size_t i = 0; i < channels; ++i) {
            envelope_[i].prepare(spec.sampleRate);
            envelope_[i].setLowpass(10.0f);
            low_[i].prepare(spec.sampleRate);
            low_[i].setLowpass(120.0f);
        }
        reset();
    }

    void reset() noexcept override {
        for (auto& x : envelope_) x.reset();
        for (auto& x : low_) x.reset();
        for (auto& x : softener_) x.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float compression = compression_.load(std::memory_order_relaxed);
        const float excursion = excursion_.load(std::memory_order_relaxed);
        const float resonance = resonance_.load(std::memory_order_relaxed);
        const float level = std::pow(10.0f, outputDb_.load(std::memory_order_relaxed) / 20.0f);
        const int channels = std::min({input.channels(), output.channels(), static_cast<int>(envelope_.size())});

        for (int ch = 0; ch < channels; ++ch) {
            const auto c = static_cast<std::size_t>(ch);
            const float* in = input.channel(ch);
            float* out = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) {
                const float x = in[i];
                const float env = envelope_[c].processLowpass(std::abs(x));
                const float thermalGain = 1.0f / (1.0f + 2.8f * compression * env);
                const float lf = low_[c].processLowpass(x);
                const float excursionDrive = x + resonance * 0.20f * lf;
                const float softened = softener_[c].process((1.0f + 3.5f * excursion) * excursionDrive)
                                     / (1.0f + 3.5f * excursion);
                out[i] = level * thermalGain * ((1.0f - excursion) * x + excursion * softened);
            }
        }
        for (int ch = channels; ch < output.channels(); ++ch)
            std::fill(output.channel(ch), output.channel(ch) + numSamples, 0.0f);
    }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) {
            case 0: return compression_.load(std::memory_order_relaxed);
            case 1: return excursion_.load(std::memory_order_relaxed);
            case 2: return resonance_.load(std::memory_order_relaxed);
            case 3: return outputDb_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float v) noexcept override {
        if (i >= descriptors_.size()) return false;
        v = graph::clampParameter(descriptors_[i], v);
        switch (i) {
            case 0: compression_.store(v, std::memory_order_relaxed); break;
            case 1: excursion_.store(v, std::memory_order_relaxed); break;
            case 2: resonance_.store(v, std::memory_order_relaxed); break;
            case 3: outputDb_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    std::vector<OnePole> envelope_, low_;
    std::vector<ADAATanh> softener_;
    std::atomic<float> compression_{0.20f};
    std::atomic<float> excursion_{0.18f};
    std::atomic<float> resonance_{0.35f};
    std::atomic<float> outputDb_{0.0f};

    static constexpr std::array<graph::ParameterDescriptor, 4> descriptors_{{
        {"compression", "Voice Coil Compression", 0.0f, 1.0f, 0.20f, graph::ParameterUnit::percent, 1.0f},
        {"excursion", "Excursion", 0.0f, 1.0f, 0.18f, graph::ParameterUnit::percent, 1.0f},
        {"resonance", "LF Resonance", 0.0f, 1.0f, 0.35f, graph::ParameterUnit::percent, 1.0f},
        {"output", "Output", -18.0f, 12.0f, 0.0f, graph::ParameterUnit::decibels, 1.0f}
    }};
};

} // namespace guitardsp::hq
