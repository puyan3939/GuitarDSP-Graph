#pragma once

#include "guitardsp/graph/AudioNode.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace guitardsp::dsp {

// Self-contained per-channel feedback delay line: 4-point Catmull-Rom
// fractional-sample read for click-free time changes, plus a low-pass/
// DC-blocking-high-pass pair and a soft saturator inside the feedback path
// that approximate a BBD bucket-brigade delay's band-limiting and
// progressively darkening/compressing repeats, without modelling the analog
// device itself.
//
// This class only knows about samples/seconds, not AudioNode/graph types,
// so a future component-level BBD circuit engine (MnaCircuitEngine-based,
// following the PreampCircuit/PowerAmpCircuit pattern) can be dropped in
// behind the same prepare/reset/processSample/set* surface without touching
// DelayEffectNode or its graph-facing Time/Feedback/Tone/Mix parameter
// contract.
class DigitalDelayEngine {
public:
    // Kept below 1.0 so the loop gain can never reach unity: this is the
    // sole structural guarantee against runaway/self-sustaining feedback,
    // independent of the tanh soft-saturator below (which shapes the decay
    // character but is not relied on for stability).
    static constexpr float kMaxFeedback = 0.97f;

    void prepare(double sampleRate, float maxDelaySeconds) noexcept {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : 48000.0;
        const int size = std::max(8, static_cast<int>(std::ceil(maxDelaySeconds * static_cast<float>(sampleRate_))) + 4);
        buffer_.assign(static_cast<std::size_t>(size), 0.0f);
        maxDelaySamples_ = static_cast<float>(size - 4);
        targetDelaySamples_ = std::clamp(targetDelaySamples_, 1.0, static_cast<double>(maxDelaySamples_));
        // ~15 ms one-pole glide: fast enough that tap-tempo time changes feel
        // immediate, slow enough that the read pointer never jumps abruptly
        // (which is what a fractional-delay chorus/vibrato voice needs too).
        smoothingCoeff_ = 1.0 - std::exp(-1.0 / (0.015 * sampleRate_));
        reset();
    }

    void reset() noexcept {
        std::fill(buffer_.begin(), buffer_.end(), 0.0f);
        writeIndex_ = 0;
        lowpassState_ = 0.0f;
        dcState_ = 0.0f;
        smoothedDelaySamples_ = targetDelaySamples_;
    }

    void setDelaySeconds(float seconds) noexcept {
        const double samples = std::max(0.0, static_cast<double>(seconds)) * sampleRate_;
        targetDelaySamples_ = std::clamp(samples, 1.0, static_cast<double>(maxDelaySamples_));
    }
    void setFeedback(float feedback) noexcept { feedback_ = std::clamp(feedback, 0.0f, kMaxFeedback); }
    void setTone(float tone) noexcept { tone_ = std::clamp(tone, 0.0f, 1.0f); }

    float processSample(float x) noexcept {
        // Smoothing runs in double: the tracked value sits near the delay
        // length in samples (up to ~1e5) while the per-sample step shrinks
        // geometrically as it converges, and single precision's ~7 decimal
        // digits can't represent a step much smaller than roughly 1e-4 of
        // that magnitude -- the update silently rounds to zero and the glide
        // stalls a fraction of a sample short of the target instead of
        // reaching it, which then blurs the fractional-read tap. Double's
        // ~15 digits keeps the step representable for any delay time this
        // engine supports.
        smoothedDelaySamples_ += (targetDelaySamples_ - smoothedDelaySamples_) * smoothingCoeff_;

        const int size = static_cast<int>(buffer_.size());
        double pos = static_cast<double>(writeIndex_) - smoothedDelaySamples_;
        while (pos < 0.0) pos += static_cast<double>(size);
        const float wet = interpolate(pos, size);

        // Tone sweeps the feedback-path low-pass; the fixed DC-blocking
        // high-pass keeps repeats from accumulating rumble/offset the way a
        // real BBD's AC-coupled stages would.
        const float lpHz = 800.0f + tone_ * 7200.0f;
        const float lpA = std::exp(-2.0f * pi_ * lpHz / static_cast<float>(sampleRate_));
        lowpassState_ = (1.0f - lpA) * wet + lpA * lowpassState_;
        constexpr float dcBlockHz = 60.0f;
        const float dcA = std::exp(-2.0f * pi_ * dcBlockHz / static_cast<float>(sampleRate_));
        dcState_ = (1.0f - dcA) * lowpassState_ + dcA * dcState_;
        const float shaped = std::tanh(lowpassState_ - dcState_);

        buffer_[static_cast<std::size_t>(writeIndex_)] = x + shaped * feedback_;
        if (++writeIndex_ >= size) writeIndex_ = 0;
        return wet;
    }

private:
    static float catmullRom(float y0, float y1, float y2, float y3, float t) noexcept {
        const float a0 = -0.5f * y0 + 1.5f * y1 - 1.5f * y2 + 0.5f * y3;
        const float a1 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
        const float a2 = -0.5f * y0 + 0.5f * y2;
        return ((a0 * t + a1) * t + a2) * t + y1;
    }

    float interpolate(double pos, int size) const noexcept {
        const int i1 = static_cast<int>(pos);
        const float frac = static_cast<float>(pos - static_cast<double>(i1));
        const int i0 = (i1 - 1 + size) % size;
        const int i2 = (i1 + 1) % size;
        const int i3 = (i1 + 2) % size;
        return catmullRom(buffer_[static_cast<std::size_t>(i0)], buffer_[static_cast<std::size_t>(i1 % size)],
                           buffer_[static_cast<std::size_t>(i2)], buffer_[static_cast<std::size_t>(i3)], frac);
    }

    static constexpr float pi_ = 3.14159265358979323846f;
    double sampleRate_ = 48000.0;
    std::vector<float> buffer_;
    int writeIndex_ = 0;
    float maxDelaySamples_ = 1.0f;
    double targetDelaySamples_ = 1.0;
    double smoothedDelaySamples_ = 1.0;
    double smoothingCoeff_ = 1.0;
    float feedback_ = 0.0f;
    float tone_ = 0.5f;
    float lowpassState_ = 0.0f;
    float dcState_ = 0.0f;
};

// Graph node exposing Time/Feedback/Tone/Mix over one DigitalDelayEngine per
// channel. See DigitalDelayEngine's header comment for the intended future
// BBD-circuit swap.
class DelayEffectNode final : public guitardsp::graph::AudioNode {
public:
    static constexpr float kMaxDelaySeconds = 2.0f;

    std::string_view typeName() const noexcept override { return "Digital Delay"; }
    guitardsp::graph::NodeCategory category() const noexcept override { return guitardsp::graph::NodeCategory::time; }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    guitardsp::graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : guitardsp::graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) {
            case 0: return timeMs_.load(std::memory_order_relaxed);
            case 1: return feedback_.load(std::memory_order_relaxed);
            case 2: return tone_.load(std::memory_order_relaxed);
            case 3: return mix_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float value) noexcept override {
        if (i >= descriptors_.size()) return false;
        value = guitardsp::graph::clampParameter(descriptors_[i], value);
        switch (i) {
            case 0: timeMs_.store(value, std::memory_order_relaxed); return true;
            case 1: feedback_.store(value, std::memory_order_relaxed); return true;
            case 2: tone_.store(value, std::memory_order_relaxed); return true;
            case 3: mix_.store(value, std::memory_order_relaxed); return true;
            default: return false;
        }
    }

    void prepare(const guitardsp::graph::PrepareSpec& spec) override {
        channels_ = std::max(1, spec.channels);
        engines_.assign(static_cast<std::size_t>(channels_), DigitalDelayEngine{});
        for (auto& engine : engines_) engine.prepare(spec.sampleRate, kMaxDelaySeconds);
        applyParameters();
    }
    void reset() noexcept override {
        for (auto& engine : engines_) engine.reset();
    }

    void process(const guitardsp::graph::AudioBuffer& input, guitardsp::graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        applyParameters();
        const float mix = mix_.load(std::memory_order_relaxed);
        const int chs = std::min({channels_, input.channels(), output.channels(), static_cast<int>(engines_.size())});
        for (int ch = 0; ch < chs; ++ch) {
            const float* src = input.channel(ch);
            float* dst = output.channel(ch);
            auto& engine = engines_[static_cast<std::size_t>(ch)];
            for (int i = 0; i < numSamples; ++i) {
                const float dry = src[i];
                const float wet = engine.processSample(dry);
                dst[i] = dry + mix * (wet - dry);
            }
        }
        for (int ch = chs; ch < output.channels(); ++ch) {
            float* dst = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) dst[i] = 0.0f;
        }
    }

private:
    void applyParameters() noexcept {
        const float seconds = timeMs_.load(std::memory_order_relaxed) * 0.001f;
        const float feedback = feedback_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        for (auto& engine : engines_) {
            engine.setDelaySeconds(seconds);
            engine.setFeedback(feedback);
            engine.setTone(tone);
        }
    }

    int channels_ = 0;
    std::vector<DigitalDelayEngine> engines_;
    std::atomic<float> timeMs_{350.0f};
    std::atomic<float> feedback_{0.35f};
    std::atomic<float> tone_{0.5f};
    std::atomic<float> mix_{0.35f};

    static constexpr std::array<guitardsp::graph::ParameterDescriptor, 4> descriptors_{{
        {"time", "Time", 1.0f, 2000.0f, 350.0f, guitardsp::graph::ParameterUnit::milliseconds, 0.4f},
        {"feedback", "Feedback", 0.0f, DigitalDelayEngine::kMaxFeedback, 0.35f, guitardsp::graph::ParameterUnit::percent, 1.0f},
        {"tone", "Tone", 0.0f, 1.0f, 0.5f, guitardsp::graph::ParameterUnit::percent, 1.0f},
        {"mix", "Mix", 0.0f, 1.0f, 0.35f, guitardsp::graph::ParameterUnit::percent, 1.0f},
    }};
};

} // namespace guitardsp::dsp
