#pragma once

#include "AmpTopologyPrimitives.h"
#include "DeviceStages.h"
#include "PolyphaseOversampler.h"
#include "QualityPolicy.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace guitardsp::hq {

// Generic high-quality amplifier topology used to validate the reusable amp blocks.
// It is intentionally unnamed: measured Marshall/Fender/Ampeg models will supply
// device values and topology-specific networks on top of the same contracts.
class ReferenceAmpTopologyNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Reference Amp Topology"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::amp; }

    void prepare(const graph::PrepareSpec& spec) override {
        const auto q = qualityFor(spec.quality, category());
        oversampler_.prepare(spec.channels, spec.maximumBlockSize, q.oversamplingFactor, q.resamplerTaps);
        const auto channels = static_cast<std::size_t>(std::max(1, spec.channels));
        inputCoupling_.assign(channels, {});
        interCoupling_.assign(channels, {});
        v1_.assign(channels, {});
        v2_.assign(channels, {});
        tone_.assign(channels, {});
        phaseInverter_.assign(channels, {});
        feedback_.assign(channels, {});
        power_.assign(channels, {});

        const double highRate = spec.sampleRate * static_cast<double>(q.oversamplingFactor);
        TriodeCommonCathodeStage::Config first;
        first.tube = TriodeModel::twelveAX7();
        first.supplyVoltage = 300.0f;
        first.plateResistance = 100000.0f;
        first.cathodeResistance = 1500.0f;
        first.gridBias = -1.25f;
        first.outputScale = 0.0090f;
        first.cathodeMemoryMs = 32.0f;

        TriodeCommonCathodeStage::Config second = first;
        second.supplyVoltage = 285.0f;
        second.cathodeResistance = 1800.0f;
        second.gridBias = -1.35f;
        second.outputScale = 0.0105f;
        second.cathodeMemoryMs = 24.0f;

        for (std::size_t i = 0; i < channels; ++i) {
            inputCoupling_[i].prepare(highRate, 28.0f);
            interCoupling_[i].prepare(highRate, 55.0f);
            v1_[i].prepare(highRate, first);
            v2_[i].prepare(highRate, second);
            tone_[i].prepare(highRate);
            phaseInverter_[i].prepare(highRate);
            feedback_[i].prepare(highRate, 7200.0f);
            power_[i].prepare(highRate);
        }
        reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
        for (auto& x : inputCoupling_) x.reset();
        for (auto& x : interCoupling_) x.reset();
        for (auto& x : v1_) x.reset();
        for (auto& x : v2_) x.reset();
        for (auto& x : tone_) x.reset();
        for (auto& x : phaseInverter_) x.reset();
        for (auto& x : feedback_) x.reset();
        for (auto& x : power_) x.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float gain = gain_.load(std::memory_order_relaxed);
        const float bass = bass_.load(std::memory_order_relaxed);
        const float mid = mid_.load(std::memory_order_relaxed);
        const float treble = treble_.load(std::memory_order_relaxed);
        const float master = master_.load(std::memory_order_relaxed);
        const float presence = presence_.load(std::memory_order_relaxed);
        const float outputGain = std::pow(10.0f, outputDb_.load(std::memory_order_relaxed) / 20.0f);

        for (auto& t : tone_) t.setControls(bass, mid, treble);
        for (auto& p : phaseInverter_) {
            p.setDrive(1.1f + 2.8f * master);
            p.setImbalance(0.02f + 0.05f * gain);
        }
        for (auto& f : feedback_) f.setAmount(0.48f - 0.30f * presence);
        for (auto& p : power_) {
            p.setDrive(1.0f + 5.0f * master);
            p.setSag(0.08f + 0.34f * master);
            p.setCrossover(0.005f + 0.025f * (1.0f - master));
            p.setTransformerSaturation(0.12f + 0.42f * master);
        }

        const float preGain = 0.35f + 2.4f * gain;
        const float secondGain = 0.55f + 2.8f * gain * gain;

        oversampler_.process(input, output, numSamples, [&](int ch, float x) noexcept {
            const auto i = static_cast<std::size_t>(ch);
            float y = inputCoupling_[i].process(x);
            y = v1_[i].process(y * preGain);
            y = interCoupling_[i].process(y);
            y = v2_[i].process(y * secondGain);
            y = tone_[i].process(y);
            y = phaseInverter_[i].process(y * (0.30f + 1.70f * master));
            y = feedback_[i].drive(y);
            const float power = power_[i].process(y);
            feedback_[i].observe(power);
            return outputGain * power;
        });
    }

    int latencySamples() const noexcept override { return oversampler_.latencySamples(); }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) {
            case 0: return gain_.load(std::memory_order_relaxed);
            case 1: return bass_.load(std::memory_order_relaxed);
            case 2: return mid_.load(std::memory_order_relaxed);
            case 3: return treble_.load(std::memory_order_relaxed);
            case 4: return master_.load(std::memory_order_relaxed);
            case 5: return presence_.load(std::memory_order_relaxed);
            case 6: return outputDb_.load(std::memory_order_relaxed);
            default: return 0.0f;
        }
    }
    bool setParameterValue(std::size_t i, float v) noexcept override {
        if (i >= descriptors_.size()) return false;
        v = graph::clampParameter(descriptors_[i], v);
        switch (i) {
            case 0: gain_.store(v, std::memory_order_relaxed); break;
            case 1: bass_.store(v, std::memory_order_relaxed); break;
            case 2: mid_.store(v, std::memory_order_relaxed); break;
            case 3: treble_.store(v, std::memory_order_relaxed); break;
            case 4: master_.store(v, std::memory_order_relaxed); break;
            case 5: presence_.store(v, std::memory_order_relaxed); break;
            case 6: outputDb_.store(v, std::memory_order_relaxed); break;
            default: return false;
        }
        return true;
    }

private:
    PolyphaseOversampler oversampler_;
    std::vector<CouplingHighpass> inputCoupling_, interCoupling_;
    std::vector<TriodeCommonCathodeStage> v1_, v2_;
    std::vector<ThreeBandToneStack> tone_;
    std::vector<LongTailPairPhaseInverter> phaseInverter_;
    std::vector<NegativeFeedbackLoop> feedback_;
    std::vector<PushPullPowerStage> power_;

    std::atomic<float> gain_{0.35f};
    std::atomic<float> bass_{0.50f};
    std::atomic<float> mid_{0.50f};
    std::atomic<float> treble_{0.50f};
    std::atomic<float> master_{0.45f};
    std::atomic<float> presence_{0.50f};
    std::atomic<float> outputDb_{-10.0f};

    static constexpr std::array<graph::ParameterDescriptor, 7> descriptors_{{
        {"gain", "Gain", 0.0f, 1.0f, 0.35f, graph::ParameterUnit::percent, 1.0f},
        {"bass", "Bass", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"mid", "Mid", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"treble", "Treble", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"master", "Master", 0.0f, 1.0f, 0.45f, graph::ParameterUnit::percent, 1.0f},
        {"presence", "Presence", 0.0f, 1.0f, 0.50f, graph::ParameterUnit::percent, 1.0f},
        {"output", "Output", -30.0f, 6.0f, -10.0f, graph::ParameterUnit::decibels, 1.0f}
    }};
};

} // namespace guitardsp::hq
