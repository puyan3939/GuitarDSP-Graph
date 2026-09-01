#pragma once

#include "PartitionedCabNode.h"
#include "SpeakerDynamicsNode.h"
#include "guitardsp/graph/AudioNode.h"
#include "guitardsp/graph/DelayCompensator.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <numbers>
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
        toneStates_.assign(static_cast<std::size_t>(std::max(1, spec.channels)), {});
        appliedLowCutHz_ = -1.0f;
        appliedHighCutHz_ = -1.0f;
        applyParameters();
    }

    void reset() noexcept override {
        speaker_.reset();
        cab_.reset();
        dryDelay_.reset();
        scratch_.clear();
        for (auto& state : toneStates_) state = {};
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
            auto& state = toneStates_[static_cast<std::size_t>(ch)];
            for (int i = 0; i < safeSamples; ++i) {
                const float highpassed = highpass_.process(
                    destination[i], state.highpass1, state.highpass2);
                destination[i] = mix * lowpass_.process(
                    highpassed, state.lowpass1, state.lowpass2);
            }
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
            case 5: return lowCutHz_.load(std::memory_order_relaxed);
            case 6: return highCutHz_.load(std::memory_order_relaxed);
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
            case 5: lowCutHz_.store(v, std::memory_order_relaxed); break;
            case 6: highCutHz_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    struct FilterCoefficients {
        float b0 = 1.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;

        float process(float input, float& state1, float& state2) const noexcept {
            const float output = b0 * input + state1;
            state1 = b1 * input - a1 * output + state2;
            state2 = b2 * input - a2 * output;
            return output;
        }
    };

    struct ToneFilterState {
        float highpass1 = 0.0f;
        float highpass2 = 0.0f;
        float lowpass1 = 0.0f;
        float lowpass2 = 0.0f;
    };

    static FilterCoefficients makeButterworth(double sampleRate,
                                              float cutoffHz,
                                              bool highpass) noexcept {
        const double frequency = std::clamp(static_cast<double>(cutoffHz),
                                             5.0, sampleRate * 0.45);
        const double omega = 2.0 * std::numbers::pi * frequency / sampleRate;
        const double cosine = std::cos(omega);
        const double alpha = std::sin(omega) / std::sqrt(2.0);
        const double normalization = 1.0 / (1.0 + alpha);
        const double endpoint = highpass ? 1.0 + cosine : 1.0 - cosine;
        return FilterCoefficients{
            static_cast<float>(0.5 * endpoint * normalization),
            static_cast<float>((highpass ? -endpoint : endpoint) * normalization),
            static_cast<float>(0.5 * endpoint * normalization),
            static_cast<float>(-2.0 * cosine * normalization),
            static_cast<float>((1.0 - alpha) * normalization)
        };
    }

    void applyParameters() noexcept {
        speaker_.setParameterValue(0, compression_.load(std::memory_order_relaxed));
        speaker_.setParameterValue(1, excursion_.load(std::memory_order_relaxed));
        speaker_.setParameterValue(2, resonance_.load(std::memory_order_relaxed));
        speaker_.setParameterValue(3, outputDb_.load(std::memory_order_relaxed));
        const float lowCut = lowCutHz_.load(std::memory_order_relaxed);
        const float highCut = highCutHz_.load(std::memory_order_relaxed);
        const double sampleRate = std::max(8000.0, spec_.sampleRate);
        if (lowCut != appliedLowCutHz_) {
            highpass_ = makeButterworth(sampleRate, lowCut, true);
            appliedLowCutHz_ = lowCut;
        }
        if (highCut != appliedHighCutHz_) {
            lowpass_ = makeButterworth(sampleRate, highCut, false);
            appliedHighCutHz_ = highCut;
        }
    }

    graph::PrepareSpec spec_{};
    graph::AudioBuffer scratch_;
    SpeakerDynamicsNode speaker_;
    PartitionedCabNode cab_;
    graph::DelayCompensator dryDelay_;
    std::vector<ToneFilterState> toneStates_;
    FilterCoefficients highpass_;
    FilterCoefficients lowpass_;
    float appliedLowCutHz_ = -1.0f;
    float appliedHighCutHz_ = -1.0f;
    std::atomic<float> compression_{0.20f};
    std::atomic<float> excursion_{0.18f};
    std::atomic<float> resonance_{0.35f};
    std::atomic<float> outputDb_{0.0f};
    std::atomic<float> mix_{1.0f};
    std::atomic<float> lowCutHz_{72.0f};
    std::atomic<float> highCutHz_{7200.0f};

    static constexpr std::array<graph::ParameterDescriptor, 7> descriptors_{{
        {"compression", "Voice Coil Compression", 0.0f, 1.0f, 0.20f, graph::ParameterUnit::percent, 1.0f},
        {"excursion", "Excursion", 0.0f, 1.0f, 0.18f, graph::ParameterUnit::percent, 1.0f},
        {"resonance", "LF Resonance", 0.0f, 1.0f, 0.35f, graph::ParameterUnit::percent, 1.0f},
        {"output", "Output", -18.0f, 12.0f, 0.0f, graph::ParameterUnit::decibels, 1.0f},
        {"mix", "Cabinet IR Mix", 0.0f, 1.0f, 1.0f, graph::ParameterUnit::percent, 1.0f},
        {"low_cut", "Cabinet Low Cut", 35.0f, 240.0f, 72.0f, graph::ParameterUnit::hertz, 0.45f},
        {"high_cut", "Cabinet High Cut", 1800.0f, 16000.0f, 7200.0f, graph::ParameterUnit::hertz, 0.40f}
    }};
};

} // namespace guitardsp::hq
