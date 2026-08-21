#pragma once

#include "DeviceStages.h"
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

// Circuit-inspired DS-1 topology reference node.
//
// This intentionally models the important signal roles instead of collapsing the
// pedal to a single waveshaper:
//   input coupling/buffer -> transistor booster -> variable op-amp gain/bandlimit
//   -> shunt hard-clipping diode pair -> passive low/high tone blend -> level.
//
// Component values and corner frequencies follow the modern DS-1 topology at a
// functional level. This is not yet a measured hardware-equivalent fit.
class DS1TopologyNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "DS-1 Topology"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        spec_ = spec;
        const auto q = qualityFor(spec.quality, category());
        oversampler_.prepare(spec.channels, spec.maximumBlockSize,
                             q.oversamplingFactor, q.resamplerTaps);

        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));
        booster_.assign(channels, {});
        clippers_.assign(channels, {});
        preHp_.assign(channels, {});
        clipHp_.assign(channels, {});
        clipLp_.assign(channels, {});
        toneLow_.assign(channels, {});
        toneHighHp_.assign(channels, {});
        dcBlock_.assign(channels, {});

        BJTCommonEmitterStage::Config boosterConfig;
        boosterConfig.supplyVoltage = 9.0f;
        boosterConfig.collectorResistance = 100000.0f;
        boosterConfig.emitterResistance = 22.0f;
        boosterConfig.baseBias = 0.72f;
        boosterConfig.outputScale = 0.020f;
        boosterConfig.emitterMemoryMs = 1.5f;
        for (auto& b : booster_) b.prepare(spec.sampleRate * static_cast<double>(q.oversamplingFactor), boosterConfig);

        for (auto& c : clippers_) {
            c.setPositive(DiodeModel::forType(DiodeType::silicon));
            c.setNegative(DiodeModel::forType(DiodeType::silicon));
            c.setSeriesResistance(2200.0f);
        }

        const double highRate = spec.sampleRate * static_cast<double>(q.oversamplingFactor);
        for (auto& f : preHp_) f.setHighpass(highRate, 23.0f);
        for (auto& f : clipHp_) f.setHighpass(highRate, 72.0f);
        for (auto& f : clipLp_) f.setLowpass(highRate, 7200.0f);
        for (auto& f : toneLow_) f.setLowpass(highRate, 234.0f);
        for (auto& f : toneHighHp_) f.setHighpass(highRate, 1063.0f);
        for (auto& f : dcBlock_) f.setHighpass(highRate, 18.0f);
        reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
        for (auto& b : booster_) b.reset();
        for (auto& c : clippers_) c.reset();
        for (auto& f : preHp_) f.reset();
        for (auto& f : clipHp_) f.reset();
        for (auto& f : clipLp_) f.reset();
        for (auto& f : toneLow_) f.reset();
        for (auto& f : toneHighHp_) f.reset();
        for (auto& f : dcBlock_) f.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float distortion = distortion_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = std::pow(10.0f, levelDb_.load(std::memory_order_relaxed) / 20.0f);
        const float boosterTrim = boosterTrim_.load(std::memory_order_relaxed);
        const float opAmpGain = 1.0f + 21.3f * distortion;

        oversampler_.process(input, output, numSamples, [&](int ch, float x) noexcept {
            const auto i = static_cast<std::size_t>(ch);

            // Input coupling and the strongly driven transistor booster are important
            // to the DS-1's even-harmonic/asymmetric character before diode clipping.
            float y = preHp_[i].process(x);
            y = booster_[i].process(y * boosterTrim);

            // Modern DS-1 variable non-inverting gain is roughly 1..22.3x, with a
            // ~72 Hz lower corner and ~7.2 kHz clipping-stage upper shaping.
            y = clipHp_[i].process(y);
            y = clipLp_[i].process(y * opAmpGain);

            // Antiparallel silicon diodes shunt the amplified signal to AC ground.
            y = clippers_[i].process(y);

            // Passive Big-Muff-like tone network: low-pass and high-pass branches
            // crossfade, producing the characteristic mid scoop near the center.
            const float low = toneLow_[i].process(y);
            const float high = toneHighHp_[i].process(y);
            const float shaped = (1.0f - tone) * low + tone * high;
            return level * dcBlock_[i].process(shaped);
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
            case 2: return levelDb_.load(std::memory_order_relaxed);
            case 3: return boosterTrim_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float v) noexcept override {
        if (i >= descriptors_.size()) return false;
        v = graph::clampParameter(descriptors_[i], v);
        switch (i) {
            case 0: distortion_.store(v, std::memory_order_relaxed); break;
            case 1: tone_.store(v, std::memory_order_relaxed); break;
            case 2: levelDb_.store(v, std::memory_order_relaxed); break;
            case 3: boosterTrim_.store(v, std::memory_order_relaxed); break;
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
            a_ = x;
            b_ = 1.0f - x;
        }
        void setHighpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::highpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz / static_cast<float>(std::max(1.0, sampleRate)));
            a_ = x;
            b_ = 1.0f - x;
        }
        void reset() noexcept { state_ = 0.0f; }
        float process(float x) noexcept {
            state_ = b_ * x + a_ * state_;
            return mode_ == Mode::lowpass ? state_ : x - state_;
        }
    private:
        enum class Mode { lowpass, highpass };
        Mode mode_ = Mode::lowpass;
        float a_ = 0.0f;
        float b_ = 1.0f;
        float state_ = 0.0f;
    };

    graph::PrepareSpec spec_{};
    PolyphaseOversampler oversampler_;
    std::vector<BJTCommonEmitterStage> booster_;
    std::vector<DiodePairSolver> clippers_;
    std::vector<OnePole> preHp_, clipHp_, clipLp_, toneLow_, toneHighHp_, dcBlock_;

    std::atomic<float> distortion_{0.55f};
    std::atomic<float> tone_{0.55f};
    std::atomic<float> levelDb_{-6.0f};
    std::atomic<float> boosterTrim_{0.65f};

    static constexpr std::array<graph::ParameterDescriptor, 4> descriptors_{{
        {"distortion", "Distortion", 0.0f, 1.0f, 0.55f, graph::ParameterUnit::percent, 1.0f},
        {"tone", "Tone", 0.0f, 1.0f, 0.55f, graph::ParameterUnit::percent, 1.0f},
        {"level", "Level", -24.0f, 12.0f, -6.0f, graph::ParameterUnit::decibels, 1.0f},
        {"booster_trim", "Booster Trim", 0.1f, 1.0f, 0.65f, graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
