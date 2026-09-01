#pragma once

#include "ADAA.h"
#include "CabinetChainNode.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

// Dedicated lightweight bass reference voicing. This is a transparent
// synthesized reference circuit, not a claim to model a measured commercial amp.
class BassAmpNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Bass Amp Reference"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::amp; }

    void prepare(const graph::PrepareSpec& spec) override {
        sampleRate_ = std::max(8000.0, spec.sampleRate);
        couplingCoefficient_ = coefficientFor(32.0);
        bassCoefficient_ = coefficientFor(170.0);
        states_.assign(static_cast<std::size_t>(std::max(1, spec.channels)), {});
        reset();
    }

    void reset() noexcept override {
        for (auto& state : states_) state = {};
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        const float gain = gain_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = level_.load(std::memory_order_relaxed);
        const float trebleCoefficient = coefficientFor(850.0 + 3300.0
                                                       * static_cast<double>(tone));
        const float drive = 1.0f + 5.0f * gain;
        const float compensate = 1.0f / (1.0f + 2.2f * gain);
        const int channels = std::min({input.channels(), output.channels(),
                                       static_cast<int>(states_.size())});
        const int count = std::min({numSamples, input.samples(), output.samples()});

        for (int channel = 0; channel < channels; ++channel) {
            auto& state = states_[static_cast<std::size_t>(channel)];
            const float* source = input.channel(channel);
            float* destination = output.channel(channel);
            for (int index = 0; index < count; ++index) {
                const float sample = std::isfinite(source[index]) ? source[index] : 0.0f;
                state.coupling += couplingCoefficient_ * (sample - state.coupling);
                const float coupled = sample - state.coupling;
                state.low += bassCoefficient_ * (coupled - state.low);
                const float voiced = coupled + state.low * (0.65f + 0.55f * (1.0f - tone));
                const float saturated = state.saturator.process(voiced * drive) * compensate;
                state.treble += trebleCoefficient * (saturated - state.treble);
                destination[index] = state.treble * level * 1.6f;
            }
        }
        for (int channel = channels; channel < output.channels(); ++channel)
            std::fill_n(output.channel(channel), count, 0.0f);
    }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t index) const noexcept override {
        return index < descriptors_.size() ? descriptors_[index] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t index) const noexcept override {
        switch (index) {
            case 0: return gain_.load(std::memory_order_relaxed);
            case 1: return tone_.load(std::memory_order_relaxed);
            case 2: return level_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t index, float value) noexcept override {
        if (index >= descriptors_.size()) return false;
        value = graph::clampParameter(descriptors_[index], value);
        switch (index) {
            case 0: gain_.store(value, std::memory_order_relaxed); break;
            case 1: tone_.store(value, std::memory_order_relaxed); break;
            case 2: level_.store(value, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    struct State {
        float coupling = 0.0f;
        float low = 0.0f;
        float treble = 0.0f;
        ADAATanh saturator;
    };

    float coefficientFor(double frequency) const noexcept {
        return static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
                                                   * frequency / sampleRate_));
    }

    double sampleRate_ = 48000.0;
    float couplingCoefficient_ = 0.0f;
    float bassCoefficient_ = 0.0f;
    std::vector<State> states_;
    std::atomic<float> gain_{0.45f};
    std::atomic<float> tone_{0.50f};
    std::atomic<float> level_{0.65f};

    static constexpr std::array<graph::ParameterDescriptor, 3> descriptors_{{
        {"gain", "Bass Drive", 0.0f, 1.0f, 0.45f, graph::ParameterUnit::percent, 1.0f},
        {"tone", "Bass Tone", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"level", "Bass Output", 0.0f, 1.0f, 0.65f, graph::ParameterUnit::percent, 1.0f}
    }};
};

// A separately addressable full speaker-dynamics/convolution chain for the
// parallel bass branch. Keeping a distinct type prevents guitar-cab controls
// from silently changing both cabinet paths.
class BassCabinetNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Bass Cabinet Reference"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::cab; }

    void setImpulseResponse(std::vector<float> impulse) {
        chain_.setImpulseResponse(std::move(impulse));
    }
    void setPartitionSize(int samples) noexcept { chain_.setPartitionSize(samples); }
    void prepare(const graph::PrepareSpec& spec) override { chain_.prepare(spec); }
    void reset() noexcept override { chain_.reset(); }
    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        chain_.process(input, output, numSamples);
    }
    int latencySamples() const noexcept override { return chain_.latencySamples(); }
    std::size_t parameterCount() const noexcept override { return chain_.parameterCount(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t index) const noexcept override {
        return chain_.parameterDescriptor(index);
    }
    float parameterValue(std::size_t index) const noexcept override {
        return chain_.parameterValue(index);
    }
    bool setParameterValue(std::size_t index, float value) noexcept override {
        return chain_.setParameterValue(index, value);
    }

private:
    CabinetChainNode chain_;
};

} // namespace guitardsp::hq
