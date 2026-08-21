#pragma once

#include "guitardsp/circuit/TS808Circuit.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

namespace guitardsp::hq {

// Graph node backed by the actual component-level TS808 MNA circuit rather than
// the earlier circuit-inspired transfer approximation. One independent circuit
// instance is kept per channel so nonlinear device and capacitor history never
// leaks between channels.
class TS808CircuitNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "TS808 Circuit"; }
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

        const float drive = drive_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = level_.load(std::memory_order_relaxed);
        const int channels = std::min(input.channels(), output.channels());
        const int samples = std::min({numSamples, input.samples(), output.samples()});

        for (int ch = 0; ch < channels; ++ch) {
            auto& circuit = circuits_[static_cast<std::size_t>(ch)];
            circuit.setControls(drive, tone, level);
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
            case 0: return drive_.load(std::memory_order_relaxed);
            case 1: return tone_.load(std::memory_order_relaxed);
            case 2: return level_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float value) noexcept override {
        if (i >= descriptors_.size()) return false;
        value = graph::clampParameter(descriptors_[i], value);
        switch (i) {
            case 0: drive_.store(value, std::memory_order_relaxed); return true;
            case 1: tone_.store(value, std::memory_order_relaxed); return true;
            case 2: level_.store(value, std::memory_order_relaxed); return true;
            default: return false;
        }
    }

    bool prepared() const noexcept { return prepared_; }
    const circuit::TS808Circuit* circuitForChannel(std::size_t channel) const noexcept {
        return channel < circuits_.size() ? &circuits_[channel] : nullptr;
    }

private:
    std::vector<circuit::TS808Circuit> circuits_;
    double sampleRate_ = 48000.0;
    bool prepared_ = false;
    std::atomic<float> drive_{circuit::TS808Circuit::defaultDrive};
    std::atomic<float> tone_{circuit::TS808Circuit::defaultTone};
    std::atomic<float> level_{circuit::TS808Circuit::defaultLevel};

    static constexpr std::array<graph::ParameterDescriptor, 3> descriptors_{{
        {"drive", "Drive", 0.0f, 1.0f, circuit::TS808Circuit::defaultDrive,
         graph::ParameterUnit::percent, 1.0f},
        {"tone", "Tone", 0.0f, 1.0f, circuit::TS808Circuit::defaultTone,
         graph::ParameterUnit::percent, 1.0f},
        {"level", "Level", 0.0f, 1.0f, circuit::TS808Circuit::defaultLevel,
         graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
