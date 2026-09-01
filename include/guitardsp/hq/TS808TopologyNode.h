#pragma once

#include "DiodeClipper.h"
#include "PolyphaseOversampler.h"
#include "QualityPolicy.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

// Circuit-inspired Tube Screamer / TS808 topology reference.
// The implementation keeps the characteristic signal roles explicit:
// input HP -> mid-focused op-amp gain -> feedback-style soft clipping
// -> post-clipping low-pass/tone shaping -> output level.
// This is not yet a measured hardware-equivalent fit.
class TS808TopologyNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "TS808 Topology"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        const auto q = qualityFor(spec.quality, category());
        oversampler_.prepare(spec.channels, spec.maximumBlockSize, q.oversamplingFactor, q.resamplerTaps);
        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));
        inputHp_.assign(channels, {});
        focusHp_.assign(channels, {});
        focusLp_.assign(channels, {});
        postLp_.assign(channels, {});
        brightHp_.assign(channels, {});
        clippers_.assign(channels, {});

        const double highRate = spec.sampleRate * static_cast<double>(q.oversamplingFactor);
        for (auto& f : inputHp_) f.setHighpass(highRate, 20.0f);
        for (auto& f : focusHp_) f.setHighpass(highRate, 720.0f);
        for (auto& f : focusLp_) f.setLowpass(highRate, 3400.0f);
        for (auto& f : postLp_) f.setLowpass(highRate, 4600.0f);
        for (auto& f : brightHp_) f.setHighpass(highRate, 1800.0f);
        for (auto& c : clippers_) {
            DiodePairModel m;
            m.saturationCurrent = 2.0e-9f;
            m.thermalVoltage = 0.026f;
            m.seriesResistance = 4700.0f;
            c.setModel(m);
        }
        reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
        for (auto& f : inputHp_) f.reset();
        for (auto& f : focusHp_) f.reset();
        for (auto& f : focusLp_) f.reset();
        for (auto& f : postLp_) f.reset();
        for (auto& f : brightHp_) f.reset();
        for (auto& c : clippers_) c.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float drive = drive_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = std::pow(10.0f, levelDb_.load(std::memory_order_relaxed) / 20.0f);
        const float cleanBlend = cleanBlend_.load(std::memory_order_relaxed);
        const float gain = 1.0f + 120.0f * drive * drive;

        oversampler_.process(input, output, numSamples, [&](int ch, float x) noexcept {
            const auto i = static_cast<std::size_t>(ch);
            const float dry = inputHp_[i].process(x);

            // The Screamer character comes from emphasizing the guitar mid band
            // before the feedback-style diode compression, rather than clipping a
            // full-band signal uniformly.
            float focused = focusHp_[i].process(dry);
            focused = focusLp_[i].process(focused);
            float driven = dry + gain * focused;

            // Feedback-diode behavior is softer than the DS-1's shunt hard clip.
            // Blend some unclipped op-amp path around the solved diode transfer to
            // retain touch dynamics and approximate the feedback-loop character.
            const float clipped = clippers_[i].process(driven);
            float y = 0.18f * driven + 0.82f * clipped;

            const float warm = postLp_[i].process(y);
            const float bright = brightHp_[i].process(y);
            y = warm + tone * 0.55f * bright;

            y = (1.0f - cleanBlend) * y + cleanBlend * dry;
            return level * y;
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
            case 1: return tone_.load(std::memory_order_relaxed);
            case 2: return levelDb_.load(std::memory_order_relaxed);
            case 3: return cleanBlend_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float v) noexcept override {
        if (i >= descriptors_.size()) return false;
        v = graph::clampParameter(descriptors_[i], v);
        switch (i) {
            case 0: drive_.store(v, std::memory_order_relaxed); break;
            case 1: tone_.store(v, std::memory_order_relaxed); break;
            case 2: levelDb_.store(v, std::memory_order_relaxed); break;
            case 3: cleanBlend_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    class OnePole {
    public:
        void setLowpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::lowpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz / static_cast<float>(std::max(1.0, sampleRate)));
            a_ = x; b_ = 1.0f - x;
        }
        void setHighpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::highpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz / static_cast<float>(std::max(1.0, sampleRate)));
            a_ = x; b_ = 1.0f - x;
        }
        void reset() noexcept { state_ = 0.0f; }
        float process(float x) noexcept {
            state_ = b_ * x + a_ * state_;
            return mode_ == Mode::lowpass ? state_ : x - state_;
        }
    private:
        enum class Mode { lowpass, highpass };
        Mode mode_ = Mode::lowpass;
        float a_ = 0.0f, b_ = 1.0f, state_ = 0.0f;
    };

    PolyphaseOversampler oversampler_;
    std::vector<OnePole> inputHp_, focusHp_, focusLp_, postLp_, brightHp_;
    std::vector<ImplicitDiodeClipper> clippers_;
    std::atomic<float> drive_{0.45f};
    std::atomic<float> tone_{0.5f};
    std::atomic<float> levelDb_{-3.0f};
    std::atomic<float> cleanBlend_{0.08f};

    static constexpr std::array<graph::ParameterDescriptor, 4> descriptors_{{
        {"drive", "Drive", 0.0f, 1.0f, 0.45f, graph::ParameterUnit::percent, 1.0f},
        {"tone", "Tone", 0.0f, 1.0f, 0.5f, graph::ParameterUnit::percent, 1.0f},
        {"level", "Level", -24.0f, 12.0f, -3.0f, graph::ParameterUnit::decibels, 1.0f},
        {"clean_blend", "Clean Blend", 0.0f, 0.35f, 0.08f, graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
