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

// Circuit-inspired Blues Driver topology reference.
//
// The intent is to preserve the major signal roles of a discrete, dynamic drive:
// input coupling -> first transistor gain stage -> frequency-shaped second gain stage
// -> asymmetric limiting/recovery -> broad tone network -> level.
//
// This is deliberately not presented as a measured hardware-equivalent BD-2 fit yet.
class BD2TopologyNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "BD-2 Topology"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        const auto q = qualityFor(spec.quality, category());
        oversampler_.prepare(spec.channels, spec.maximumBlockSize, q.oversamplingFactor, q.resamplerTaps);
        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));

        stage1_.assign(channels, {});
        stage2_.assign(channels, {});
        limiter_.assign(channels, {});
        inputHp_.assign(channels, {});
        bodyLp_.assign(channels, {});
        interHp_.assign(channels, {});
        interLp_.assign(channels, {});
        toneLow_.assign(channels, {});
        toneHigh_.assign(channels, {});
        outputLp_.assign(channels, {});
        dcBlock_.assign(channels, {});

        const double highRate = spec.sampleRate * static_cast<double>(q.oversamplingFactor);

        BJTCommonEmitterStage::Config first;
        first.supplyVoltage = 9.0f;
        first.collectorResistance = 12000.0f;
        first.emitterResistance = 820.0f;
        first.baseBias = 0.71f;
        first.outputScale = 0.22f;
        first.emitterMemoryMs = 4.5f;

        BJTCommonEmitterStage::Config second;
        second.supplyVoltage = 9.0f;
        second.collectorResistance = 6800.0f;
        second.emitterResistance = 390.0f;
        second.baseBias = 0.70f;
        second.outputScale = 0.18f;
        second.emitterMemoryMs = 2.2f;

        for (auto& s : stage1_) s.prepare(highRate, first);
        for (auto& s : stage2_) s.prepare(highRate, second);
        for (auto& clip : limiter_) {
            clip.setPositive(DiodeModel::forType(DiodeType::silicon));
            clip.setNegative(DiodeModel::forType(DiodeType::led));
            clip.setSeriesResistance(3300.0f);
        }

        for (auto& f : inputHp_) f.setHighpass(highRate, 18.0f);
        for (auto& f : bodyLp_) f.setLowpass(highRate, 420.0f);
        for (auto& f : interHp_) f.setHighpass(highRate, 115.0f);
        for (auto& f : interLp_) f.setLowpass(highRate, 6900.0f);
        for (auto& f : toneLow_) f.setLowpass(highRate, 1250.0f);
        for (auto& f : toneHigh_) f.setHighpass(highRate, 1450.0f);
        for (auto& f : outputLp_) f.setLowpass(highRate, 10500.0f);
        for (auto& f : dcBlock_) f.setHighpass(highRate, 14.0f);
        reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
        for (auto& s : stage1_) s.reset();
        for (auto& s : stage2_) s.reset();
        for (auto& c : limiter_) c.reset();
        for (auto& f : inputHp_) f.reset();
        for (auto& f : bodyLp_) f.reset();
        for (auto& f : interHp_) f.reset();
        for (auto& f : interLp_) f.reset();
        for (auto& f : toneLow_) f.reset();
        for (auto& f : toneHigh_) f.reset();
        for (auto& f : outputLp_) f.reset();
        for (auto& f : dcBlock_) f.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float drive = drive_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = std::pow(10.0f, levelDb_.load(std::memory_order_relaxed) / 20.0f);
        const float dynamics = dynamics_.load(std::memory_order_relaxed);

        const float firstDrive = 0.35f + 1.65f * drive;
        const float secondDrive = 0.55f + 3.8f * drive * drive;
        const float cleanMemory = 0.04f + 0.16f * dynamics;

        oversampler_.process(input, output, numSamples, [&](int ch, float x) noexcept {
            const auto i = static_cast<std::size_t>(ch);
            const float dry = inputHp_[i].process(x);
            const float body = bodyLp_[i].process(dry);

            // Keep some low-frequency body at the first discrete stage while the
            // drive control progressively increases the transistor excursion.
            float y = stage1_[i].process((dry + 0.22f * body) * firstDrive);
            y = interHp_[i].process(y);
            y = interLp_[i].process(y);

            // A second dynamic gain stage provides touch-dependent compression before
            // the asymmetric limiter. Retaining part of its unclipped signal keeps
            // the low-gain response from collapsing into a generic diode clipper.
            const float second = stage2_[i].process(y * secondDrive);
            const float limited = limiter_[i].process(second);
            y = (0.34f + cleanMemory) * second + (0.66f - cleanMemory) * limited;

            const float low = toneLow_[i].process(y);
            const float high = toneHigh_[i].process(y);
            const float shaped = (1.0f - tone) * (0.82f * low + 0.18f * y)
                               + tone * (0.48f * y + 0.52f * high);
            return level * dcBlock_[i].process(outputLp_[i].process(shaped));
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
            case 3: return dynamics_.load(std::memory_order_relaxed);
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
            case 3: dynamics_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    class OnePole {
    public:
        void setLowpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::lowpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz
                                     / static_cast<float>(std::max(1.0, sampleRate)));
            a_ = x; b_ = 1.0f - x;
        }
        void setHighpass(double sampleRate, float hz) noexcept {
            mode_ = Mode::highpass;
            const float x = std::exp(-2.0f * std::numbers::pi_v<float> * hz
                                     / static_cast<float>(std::max(1.0, sampleRate)));
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
    std::vector<BJTCommonEmitterStage> stage1_, stage2_;
    std::vector<DiodePairSolver> limiter_;
    std::vector<OnePole> inputHp_, bodyLp_, interHp_, interLp_, toneLow_, toneHigh_, outputLp_, dcBlock_;

    std::atomic<float> drive_{0.40f};
    std::atomic<float> tone_{0.50f};
    std::atomic<float> levelDb_{-5.0f};
    std::atomic<float> dynamics_{0.55f};

    static constexpr std::array<graph::ParameterDescriptor, 4> descriptors_{{
        {"drive", "Drive", 0.0f, 1.0f, 0.40f, graph::ParameterUnit::percent, 1.0f},
        {"tone", "Tone", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"level", "Level", -24.0f, 12.0f, -5.0f, graph::ParameterUnit::decibels, 1.0f},
        {"dynamics", "Dynamics", 0.0f, 1.0f, 0.55f, graph::ParameterUnit::percent, 1.0f}
    }};
};

} // namespace guitardsp::hq
