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

// Graph node backed by the component-level 12AX7 common-cathode preamp stage
// (guitardsp::circuit::PreampCircuit), following the same pattern as
// TS808CircuitNode/DS1CircuitNode: one independent circuit instance per
// channel so nonlinear device and capacitor history never leaks between
// channels, running at the quality-policy oversampled rate with resampler
// latency reported to graph PDC.
//
// PreampCircuit itself only models Bass/Treble (its passive tone stack) --
// a single self-biased common-cathode stage has one fixed operating point,
// so there is no hardware gain pot in the circuit itself. Drive is exposed
// here as an input pre-gain applied before the signal reaches the tube
// stage, the same role a real preamp's input/gain trim plays: turning it up
// drives the 12AX7 further into grid conduction and plate-swing clipping.
class PreampCircuitNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Preamp Circuit"; }
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

        const float drive = drive_.load(std::memory_order_relaxed);
        const float bass = bass_.load(std::memory_order_relaxed);
        const float treble = treble_.load(std::memory_order_relaxed);
        const float preGain = driveToPreGain(drive);
        for (auto& circuit : circuits_) circuit.setControls(bass, treble);

        const int samples = std::min({numSamples, input.samples(), output.samples()});
        oversampler_.process(input, output, samples, [&](int ch, float x) noexcept {
            const auto index = static_cast<std::size_t>(ch);
            return index < circuits_.size() ? circuits_[index].processSample(x * preGain) : 0.0f;
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

    static constexpr float defaultDrive = 0.30f;

private:
    // 0..1 normalized drive maps to a 0.25x-4x input pre-gain -- wide enough
    // to move the fixed-bias 12AX7 stage from near-clean to clearly driven,
    // matching the range ReferenceAmpTopologyNode uses for its own pre-gain.
    static float driveToPreGain(float drive) noexcept {
        return 0.25f + 3.75f * std::clamp(drive, 0.0f, 1.0f);
    }

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
