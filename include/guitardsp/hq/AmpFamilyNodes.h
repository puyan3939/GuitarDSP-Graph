#pragma once

#include "ReferenceAmpTopologyNode.h"

#include <array>

namespace guitardsp::hq {

class AmpFamilyReferenceBase : public graph::AudioNode {
public:
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::amp; }
    void prepare(const graph::PrepareSpec& spec) override {
        core_.prepare(spec);
        applyFamily();
    }
    void reset() noexcept override { core_.reset(); }
    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        applyFamily();
        core_.process(input, output, numSamples);
    }
    int latencySamples() const noexcept override { return core_.latencySamples(); }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        return i < descriptors_.size() ? core_.parameterValue(i) : 0.0f;
    }
    bool setParameterValue(std::size_t i, float value) noexcept override {
        return i < descriptors_.size() && core_.setParameterValue(i, value);
    }

protected:
    virtual float fixedPowerTube() const noexcept = 0;
    virtual float fixedToneStack() const noexcept = 0;
    virtual float fixedToneDriver() const noexcept = 0;
    virtual float fixedFeedbackVoicing() const noexcept = 0;

    void applyFamily() noexcept {
        core_.setParameterValue(7, fixedPowerTube());
        core_.setParameterValue(8, fixedToneStack());
        core_.setParameterValue(9, fixedToneDriver());
        core_.setParameterValue(10, fixedFeedbackVoicing());
    }

    ReferenceAmpTopologyNode core_;

private:
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

class BritishPlexiFamilyNode final : public AmpFamilyReferenceBase {
public:
    std::string_view typeName() const noexcept override { return "British Plexi Family Reference"; }
protected:
    float fixedPowerTube() const noexcept override { return 0.0f; }
    float fixedToneStack() const noexcept override { return 1.0f; }
    float fixedToneDriver() const noexcept override { return 1.0f; }
    float fixedFeedbackVoicing() const noexcept override { return 1.0f; }
};

class AmericanCleanFamilyNode final : public AmpFamilyReferenceBase {
public:
    std::string_view typeName() const noexcept override { return "American Clean Family Reference"; }
protected:
    float fixedPowerTube() const noexcept override { return 1.0f; }
    float fixedToneStack() const noexcept override { return 2.0f; }
    float fixedToneDriver() const noexcept override { return 2.0f; }
    float fixedFeedbackVoicing() const noexcept override { return 2.0f; }
};

} // namespace guitardsp::hq
