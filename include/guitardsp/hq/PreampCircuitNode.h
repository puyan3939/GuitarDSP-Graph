#pragma once

#include "guitardsp/circuit/PreampCircuit.h"
#include "guitardsp/graph/AudioNode.h"
#include "guitardsp/hq/PolyphaseOversampler.h"
#include "guitardsp/hq/QualityPolicy.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

namespace guitardsp::hq {

// Graph node backed by the component-level 12AX7 common-cathode preamp MNA
// circuit (guitardsp::circuit::PreampCircuit), following the same pattern as
// TS808CircuitNode/DS1CircuitNode: one independent circuit instance per
// channel, run at the quality-policy oversampled rate, with resampler
// latency reported to graph PDC.
//
// PreampCircuit itself only exposes Bass/Treble (its tone stack); it has no
// notion of a "Drive" control because a single self-biased common-cathode
// stage has a fixed operating point. The Drive parameter here is a pre-gain
// applied to the input before it reaches the tube stage -- exactly what a
// real preamp's input/gain trim does -- so driving it harder pushes the
// 12AX7 stage further into grid conduction and plate-swing clipping.
class PreampCircuitNode final : public graph::AudioNode {
public:
    static constexpr float defaultDrive = 0.5f;
    static constexpr float minDriveGain = 0.25f;
    static constexpr float maxDriveGain = 4.0f;

    std::string_view typeName() const noexcept override { return "Preamp Circuit"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::amp; }

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

        const float drive = drive_.load(std::memory_order_relaxed);
        const float bass = bass_.load(std::memory_order_relaxed);
        const float treble = treble_.load(std::memory_order_relaxed);
        const float driveGain = minDriveGain + drive * (maxDriveGain - minDriveGain);
        for (auto& circuit : circuits_) circuit.setControls(bass, treble);

        const int samples = std::min({numSamples, input.samples(), output.samples()});
        oversampler_.process(input, output, samples, [&](int ch, float x) noexcept {
            const auto index = static_cast<std::size_t>(ch);
            return index < circuits_.size() ? circuits_[index].processSample(x * driveGain) : 0.0f;
        });
    }

    int latencySamples() const noexcept override { return oversampler_.latencySamples(); }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) {
            case 0: return drive_.load(std::memory_order_relaxed);
            case 1: return bass_.load(std::memory_order_relaxed);
            case 2: return treble_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float value) noexcept override {
        if (i >= descriptors_.size()) return false;
        value = graph::clampParameter(descriptors_[i], value);
        switch (i) {
            case 0: drive_.store(value, std::memory_order_relaxed); return true;
            case 1: bass_.store(value, std::memory_order_relaxed); return true;
            case 2: treble_.store(value, std::memory_order_relaxed); return true;
            default: return false;
        }
    }

    bool prepared() const noexcept { return prepared_; }
    int oversamplingFactor() const noexcept { return oversampler_.factor(); }
    double internalSampleRate() const noexcept { return internalSampleRate_; }
    const circuit::PreampCircuit* circuitForChannel(std::size_t channel) const noexcept {
        return channel < circuits_.size() ? &circuits_[channel] : nullptr;
    }

private:
    std::vector<circuit::PreampCircuit> circuits_;
    PolyphaseOversampler oversampler_;
    double sampleRate_ = 48000.0;
    double internalSampleRate_ = 48000.0;
    bool prepared_ = false;
    std::atomic<float> drive_{defaultDrive};
    std::atomic<float> bass_{circuit::PreampCircuit::defaultBass};
    std::atomic<float> treble_{circuit::PreampCircuit::defaultTreble};

    static constexpr std::array<graph::ParameterDescriptor, 3> descriptors_{{
        {"drive", "Drive", 0.0f, 1.0f, defaultDrive,
         graph::ParameterUnit::percent, 1.0f},
        {"bass", "Bass", 0.0f, 1.0f, circuit::PreampCircuit::defaultBass,
         graph::ParameterUnit::percent, 1.0f},
        {"treble", "Treble", 0.0f, 1.0f, circuit::PreampCircuit::defaultTreble,
         graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
