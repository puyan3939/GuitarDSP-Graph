#pragma once

#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/graph/AudioNode.h"
#include "guitardsp/hq/PolyphaseOversampler.h"
#include "guitardsp/hq/QualityPolicy.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

namespace guitardsp::hq {

// Graph node backed by the component-level modern DS-1 MNA circuit. Each channel
// owns a completely independent circuit so semiconductor and capacitor history are
// never shared between channels. The whole nonlinear circuit runs at the selected
// quality-policy oversampled rate and reports the resampler delay to graph PDC.
class DS1CircuitNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "DS-1 Circuit"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        sampleRate_ = std::max(1.0, spec.sampleRate);
        const int channelCount = std::max(1, spec.channels);
        const auto channels = static_cast<std::size_t>(channelCount);
        const auto quality = qualityFor(spec.quality, category());

        oversampler_.prepare(channelCount, std::max(1, spec.maximumBlockSize),
                             quality.oversamplingFactor, quality.resamplerTaps);
        internalSampleRate_ = sampleRate_ * static_cast<double>(oversampler_.factor());

        circuits_.clear();
        circuits_.resize(channels);
        prepared_ = true;
        for (auto& circuit : circuits_) {
            if (!circuit.prepare(internalSampleRate_)) {
                prepared_ = false;
                break;
            }
        }
        if (prepared_) oversampler_.reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
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
        for (auto& circuit : circuits_) circuit.setControls(distortion, tone, level);

        const int samples = std::min({numSamples, input.samples(), output.samples()});
        oversampler_.process(input, output, samples, [&](int ch, float x) noexcept {
            const auto index = static_cast<std::size_t>(ch);
            return index < circuits_.size() ? circuits_[index].processSample(x) : 0.0f;
        });
    }

    int latencySamples() const noexcept override { return oversampler_.latencySamples(); }

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
    int oversamplingFactor() const noexcept { return oversampler_.factor(); }
    double internalSampleRate() const noexcept { return internalSampleRate_; }
    const circuit::DS1Circuit* circuitForChannel(std::size_t channel) const noexcept {
        return channel < circuits_.size() ? &circuits_[channel] : nullptr;
    }

private:
    std::vector<circuit::DS1Circuit> circuits_;
    PolyphaseOversampler oversampler_;
    double sampleRate_ = 48000.0;
    double internalSampleRate_ = 48000.0;
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
