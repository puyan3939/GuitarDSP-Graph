#pragma once

#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

// Monophonic Schmitt/PLL octave divider. The oscillator follows measured input
// periods instead of resampling blocks, so there is no allocation, FFT, or
// extra buffering on the realtime thread. Chords are deliberately not claimed
// to track polyphonically.
class OctaveDownNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Monophonic Octave Down"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::pitch; }

    void prepare(const graph::PrepareSpec& spec) override {
        sampleRate_ = std::max(8000.0, spec.sampleRate);
        detectorCoefficient_ = coefficientFor(900.0);
        envelopeAttack_ = coefficientFor(70.0);
        envelopeRelease_ = coefficientFor(8.0);
        channels_.assign(static_cast<std::size_t>(std::max(1, spec.channels)), {});
        reset();
    }

    void reset() noexcept override {
        for (auto& channel : channels_) channel = {};
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        const float mix = mix_.load(std::memory_order_relaxed);
        const float level = level_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float tracking = tracking_.load(std::memory_order_relaxed);
        const float outputCoefficient = coefficientFor(280.0 + 2400.0 * static_cast<double>(tone));
        const int channels = std::min({input.channels(), output.channels(),
                                       static_cast<int>(channels_.size())});
        const int count = std::min({numSamples, input.samples(), output.samples()});

        for (int channel = 0; channel < channels; ++channel) {
            auto& state = channels_[static_cast<std::size_t>(channel)];
            const float* source = input.channel(channel);
            float* destination = output.channel(channel);

            for (int index = 0; index < count; ++index) {
                const float sample = std::isfinite(source[index]) ? source[index] : 0.0f;
                state.detector += detectorCoefficient_ * (sample - state.detector);
                const float absolute = std::abs(state.detector);
                const float envelopeCoefficient = absolute > state.envelope
                    ? envelopeAttack_ : envelopeRelease_;
                state.envelope += envelopeCoefficient * (absolute - state.envelope);
                ++state.samplesSinceCrossing;

                const float hysteresis = std::max(1.0e-5f,
                    state.envelope * (0.04f + 0.12f * (1.0f - tracking)));
                if (!state.high && state.detector > hysteresis) {
                    state.high = true;
                    const int period = state.samplesSinceCrossing;
                    const int shortest = std::max(8, static_cast<int>(sampleRate_ / 1200.0));
                    const int longest = static_cast<int>(sampleRate_ / 35.0);
                    if (period >= shortest && period <= longest) {
                        const float measured = std::numbers::pi_v<float>
                            / static_cast<float>(period);
                        state.increment = state.increment == 0.0f
                            ? measured : 0.82f * state.increment + 0.18f * measured;
                        ++state.crossingCount;
                        if ((state.crossingCount & 1) == 0) {
                            // Gentle phase correction keeps the divider locked
                            // without generating hard reset clicks.
                            const float wrapped = state.phase > std::numbers::pi_v<float>
                                ? state.phase - 2.0f * std::numbers::pi_v<float>
                                : state.phase;
                            state.phase -= 0.14f * wrapped;
                        }
                    }
                    state.samplesSinceCrossing = 0;
                } else if (state.high && state.detector < -hysteresis) {
                    state.high = false;
                }

                state.phase += state.increment;
                if (state.phase >= 2.0f * std::numbers::pi_v<float>)
                    state.phase -= 2.0f * std::numbers::pi_v<float>;

                const float fundamental = std::sin(state.phase);
                const float shaped = fundamental + 0.10f * std::sin(3.0f * state.phase);
                const float octave = shaped * state.envelope * 1.55f * level;
                state.outputLowpass += outputCoefficient * (octave - state.outputLowpass);
                destination[index] = (1.0f - mix) * sample + mix * state.outputLowpass;
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
            case 0: return mix_.load(std::memory_order_relaxed);
            case 1: return level_.load(std::memory_order_relaxed);
            case 2: return tone_.load(std::memory_order_relaxed);
            case 3: return tracking_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t index, float value) noexcept override {
        if (index >= descriptors_.size()) return false;
        value = graph::clampParameter(descriptors_[index], value);
        switch (index) {
            case 0: mix_.store(value, std::memory_order_relaxed); break;
            case 1: level_.store(value, std::memory_order_relaxed); break;
            case 2: tone_.store(value, std::memory_order_relaxed); break;
            case 3: tracking_.store(value, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    struct State {
        float detector = 0.0f;
        float envelope = 0.0f;
        float outputLowpass = 0.0f;
        float phase = 0.0f;
        float increment = 0.0f;
        int samplesSinceCrossing = 0;
        int crossingCount = 0;
        bool high = false;
    };

    float coefficientFor(double frequency) const noexcept {
        return static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
                                                   * frequency / sampleRate_));
    }

    double sampleRate_ = 48000.0;
    float detectorCoefficient_ = 0.0f;
    float envelopeAttack_ = 0.0f;
    float envelopeRelease_ = 0.0f;
    std::vector<State> channels_;
    std::atomic<float> mix_{1.0f};
    std::atomic<float> level_{0.85f};
    std::atomic<float> tone_{0.50f};
    std::atomic<float> tracking_{0.50f};

    static constexpr std::array<graph::ParameterDescriptor, 4> descriptors_{{
        {"mix", "Octave Mix", 0.0f, 1.0f, 1.0f, graph::ParameterUnit::percent, 1.0f},
        {"level", "Octave Level", 0.0f, 1.5f, 0.85f, graph::ParameterUnit::generic, 1.0f},
        {"tone", "Octave Tone", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"tracking", "Tracking Sensitivity", 0.0f, 1.0f, 0.50f,
         graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
