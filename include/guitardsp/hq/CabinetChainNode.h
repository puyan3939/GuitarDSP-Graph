#pragma once

#include "PartitionedCabNode.h"
#include "SpeakerDynamicsNode.h"
#include "guitardsp/graph/AudioNode.h"
#include "guitardsp/graph/DelayCompensator.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace guitardsp::hq {

// Production-oriented cabinet chain: nonlinear speaker dynamics followed by a
// long linear measured IR. IR replacement remains a control-thread operation.
class CabinetChainNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Speaker Dynamics + Partitioned Cab"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::cab; }

    void setImpulseResponse(std::vector<float> impulse) { cab_.setImpulseResponse(std::move(impulse)); }
    void setPartitionSize(int samples) noexcept { cab_.setPartitionSize(samples); }

    void prepare(const graph::PrepareSpec& spec) override {
        spec_ = spec;
        scratch_.resize(std::max(1, spec.channels), std::max(1, spec.maximumBlockSize));
        speaker_.prepare(spec);
        cab_.prepare(spec);
        dryDelay_.prepare(std::max(1, spec.channels), cab_.latencySamples(),
                          std::max(1, spec.maximumBlockSize));
        dryDelay_.setDelaySamples(cab_.latencySamples());
        applyParameters();
    }

    void reset() noexcept override {
        speaker_.reset();
        cab_.reset();
        dryDelay_.reset();
        scratch_.clear();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const int safeSamples = std::min(numSamples, spec_.maximumBlockSize);
        applyParameters();
        speaker_.process(input, scratch_, safeSamples);
        cab_.process(scratch_, output, safeSamples);
        const float mix = mix_.load(std::memory_order_relaxed);
        const int channels = std::min(scratch_.channels(), output.channels());
        for (int ch = 0; ch < channels; ++ch) {
            float* destination = output.channel(ch);
            for (int i = 0; i < safeSamples; ++i) destination[i] *= mix;
        }
        // Keep the pre-IR path aligned with the partitioned convolution even
        // when fully wet, so live mix edits never expose stale delay history.
        dryDelay_.processAdd(scratch_, output, safeSamples, 1.0f - mix);
        if (safeSamples < numSamples) {
            for (int ch = 0; ch < output.channels(); ++ch)
                std::fill(output.channel(ch) + safeSamples, output.channel(ch) + numSamples, 0.0f);
        }
    }

    int latencySamples() const noexcept override { return cab_.latencySamples(); }

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
            case 4: return mix_.load(std::memory_order_relaxed);
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
            case 4: mix_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    void applyParameters() noexcept {
        speaker_.setParameterValue(0, compression_.load(std::memory_order_relaxed));
        speaker_.setParameterValue(1, excursion_.load(std::memory_order_relaxed));
        speaker_.setParameterValue(2, resonance_.load(std::memory_order_relaxed));
        speaker_.setParameterValue(3, outputDb_.load(std::memory_order_relaxed));
    }

    graph::PrepareSpec spec_{};
    graph::AudioBuffer scratch_;
    SpeakerDynamicsNode speaker_;
    PartitionedCabNode cab_;
    graph::DelayCompensator dryDelay_;
    std::atomic<float> compression_{0.20f};
    std::atomic<float> excursion_{0.18f};
    std::atomic<float> resonance_{0.35f};
    std::atomic<float> outputDb_{0.0f};
    std::atomic<float> mix_{1.0f};

    static constexpr std::array<graph::ParameterDescriptor, 5> descriptors_{{
        {"compression", "Voice Coil Compression", 0.0f, 1.0f, 0.20f, graph::ParameterUnit::percent, 1.0f},
        {"excursion", "Excursion", 0.0f, 1.0f, 0.18f, graph::ParameterUnit::percent, 1.0f},
        {"resonance", "LF Resonance", 0.0f, 1.0f, 0.35f, graph::ParameterUnit::percent, 1.0f},
        {"output", "Output", -18.0f, 12.0f, 0.0f, graph::ParameterUnit::decibels, 1.0f},
        {"mix", "Cabinet IR Mix", 0.0f, 1.0f, 1.0f, graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
