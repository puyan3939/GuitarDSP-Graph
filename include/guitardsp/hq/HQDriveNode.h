#pragma once
#include "ADAA.h"
#include "DiodeClipper.h"
#include "Oversampler.h"
#include "QualityPolicy.h"
#include "guitardsp/graph/AudioNode.h"
#include <array>
#include <atomic>
#include <cmath>

namespace guitardsp::hq {

class HQDriveNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "HQ Drive"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        spec_ = spec;
        const auto q = qualityFor(spec.quality, category());
        oversampler_.prepare(spec.channels, spec.maximumBlockSize, q.oversamplingFactor, q.resamplerTaps);
        diode_.assign(static_cast<std::size_t>(spec.channels), {});
        adaa_.assign(static_cast<std::size_t>(spec.channels), {});
        reset();
    }

    void reset() noexcept override {
        oversampler_.reset();
        for (auto& d : diode_) d.reset();
        for (auto& a : adaa_) a.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const float drive = drive_.load(std::memory_order_relaxed);
        const float mix = mix_.load(std::memory_order_relaxed);
        const float level = std::pow(10.0f, levelDb_.load(std::memory_order_relaxed) / 20.0f);
        const float pre = 1.0f + 19.0f * drive * drive;
        const bool adaa = adaaEnabled_.load(std::memory_order_relaxed) >= 0.5f;

        oversampler_.process(input, output, numSamples, [&](int ch, float x) noexcept {
            float y = x * pre;
            if (clipMode_.load(std::memory_order_relaxed) < 0.5f)
                y = diode_[static_cast<std::size_t>(ch)].process(y);
            else
                y = adaa ? adaa_[static_cast<std::size_t>(ch)].process(y) : std::tanh(y);
            return level * ((1.0f - mix) * x + mix * y);
        });
    }

    int latencySamples() const noexcept override { return oversampler_.latencySamples(); }

    std::size_t parameterCount() const noexcept override { return descriptors_.size(); }
    graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override { return i < descriptors_.size() ? descriptors_[i] : graph::ParameterDescriptor{}; }
    float parameterValue(std::size_t i) const noexcept override {
        switch (i) { case 0: return drive_.load(); case 1: return mix_.load(); case 2: return levelDb_.load(); case 3: return clipMode_.load(); case 4: return adaaEnabled_.load(); default: return 0.0f; }
    }
    bool setParameterValue(std::size_t i, float v) noexcept override {
        if (i >= descriptors_.size()) return false;
        v = graph::clampParameter(descriptors_[i], v);
        switch (i) { case 0: drive_.store(v); break; case 1: mix_.store(v); break; case 2: levelDb_.store(v); break; case 3: clipMode_.store(v); break; case 4: adaaEnabled_.store(v); break; default: return false; }
        return true;
    }

private:
    graph::PrepareSpec spec_{};
    Oversampler oversampler_;
    std::vector<ImplicitDiodeClipper> diode_;
    std::vector<ADAATanh> adaa_;
    std::atomic<float> drive_{0.45f}, mix_{1.0f}, levelDb_{0.0f}, clipMode_{0.0f}, adaaEnabled_{1.0f};

    static constexpr std::array<graph::ParameterDescriptor, 5> descriptors_{{
        {"drive","Drive",0.0f,1.0f,0.45f,graph::ParameterUnit::percent,1.0f},
        {"mix","Mix",0.0f,1.0f,1.0f,graph::ParameterUnit::percent,1.0f},
        {"level","Level",-24.0f,12.0f,0.0f,graph::ParameterUnit::decibels,1.0f},
        {"mode","Clip Mode",0.0f,1.0f,0.0f,graph::ParameterUnit::generic,1.0f},
        {"adaa","ADAA",0.0f,1.0f,1.0f,graph::ParameterUnit::generic,1.0f}
    }};
};

} // namespace guitardsp::hq
