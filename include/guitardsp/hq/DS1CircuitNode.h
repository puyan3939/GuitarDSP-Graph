#pragma once

#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

namespace guitardsp::hq {

// Graph node backed by the component-level modern DS-1 MNA circuit. Each channel
// owns a completely independent circuit so semiconductor and capacitor history are
// never shared between channels.
class DS1CircuitNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "DS-1 Circuit"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        sampleRate_ = std::max(1.0, spec.sampleRate);
        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));
        circuits_.clear();
        circuits_.resize(channels);
        prepared_ = true;
        for (auto& circuit : circuits_) {
            if (!circuit.prepare(sampleRate_)) {
                prepared_ = false;
                break;
            }
        }
    }

    void reset() noexcept override {
        for (auto& circuit : circuits_) circuit.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        if (!prepared_) {
            output.copyFrom(input, numSamples);
            return;
        }

        const float distortion = distortion_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = level_.load(std::memory_order_relaxed);
        const int channels = std::min(input.channels(), output.channels());
        const int samples = std::min({numSamples, input.samples(), output.samples()});

        for (int ch = 0; ch < channels; ++ch) {
            auto& circuit = circuits_[static_cast<std::size_t>(ch)];
            circuit.setControls(distortion, tone, level);
            const float* src = input.channel(ch);
            float* dst = output.channel(ch);
            for (int i = 0; i < samples; ++i)
                dst[i] = circuit.processSample(src[i]);
        }

        for (int ch = channels; ch < output.channels(); ++ch) {
            float* dst = output.channel(ch);
            for (int i = 0; i < samples; ++i) dst[i] = 0.0f;
        }
    }

    int latencySamples() const noexcept override { return 0; }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) {
            case 0: return distortion_.load(std::memory_order_relaxed);
            case 1: return tone_.load(std::memory_order_relaxed);
            case 2: return level_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float value) noexcept override {
        if (i >= descriptors_.size()) return false;
        value = graph::clampParameter(descriptors_[i], value);
        switch (i) {
            case 0: distortion_.store(value, std::memory_order_relaxed); return true;
            case 1: tone_.store(value, std::memory_order_relaxed); return true;
            case 2: level_.store(value, std::memory_order_relaxed); return true;
            default: return false;
        }
    }

    bool prepared() const noexcept { return prepared_; }
    const circuit::DS1Circuit* circuitForChannel(std::size_t channel) const noexcept {
        return channel < circuits_.size() ? &circuits_[channel] : nullptr;
    }

private:
    std::vector<circuit::DS1Circuit> circuits_;
    double sampleRate_ = 48000.0;
    bool prepared_ = false;
    std::atomic<float> distortion_{circuit::DS1Circuit::defaultDistortion};
    std::atomic<float> tone_{circuit::DS1Circuit::defaultTone};
    std::atomic<float> level_{circuit::DS1Circuit::defaultLevel};

    static constexpr std::array<graph::ParameterDescriptor, 3> descriptors_{{
        {"distortion", "Distortion", 0.0f, 1.0f, circuit::DS1Circuit::defaultDistortion,
         graph::ParameterUnit::percent, 1.0f},
        {"tone", "Tone", 0.0f, 1.0f, circuit::DS1Circuit::defaultTone,
         graph::ParameterUnit::percent, 1.0f},
        {"level", "Level", 0.0f, 1.0f, circuit::DS1Circuit::defaultLevel,
         graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
