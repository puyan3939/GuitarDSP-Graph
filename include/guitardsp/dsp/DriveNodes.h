#pragma once

#include "CircuitPrimitives.h"
#include "guitardsp/graph/AudioNode.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace guitardsp::dsp {

class DS1PrototypeNode final : public guitardsp::graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "DS1Prototype"; }
    guitardsp::graph::NodeCategory category() const noexcept override { return guitardsp::graph::NodeCategory::drive; }
    std::size_t parameterCount() const noexcept override { return 3; }
    guitardsp::graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        using namespace guitardsp::graph; static constexpr ParameterDescriptor p[]={{"distortion","Distortion",0.0f,1.0f,0.55f,ParameterUnit::percent,1.0f},{"tone","Tone",0.0f,1.0f,0.58f,ParameterUnit::percent,1.0f},{"level","Level",0.0f,1.5f,0.72f,ParameterUnit::generic,1.0f}};return i<3?p[i]:ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {switch(i){case 0:return distortion_.load();case 1:return tone_.load();case 2:return level_.load();default:return 0.0f;}}
    bool setParameterValue(std::size_t i,float v) noexcept override { if(i>=3)return false;v=guitardsp::graph::clampParameter(parameterDescriptor(i),v);if(i==0)setDistortion(v);else if(i==1)setTone(v);else setLevel(v);return true;}

    void setDistortion(float v) noexcept { distortion_.store(std::clamp(v, 0.0f, 1.0f), std::memory_order_relaxed); }
    void setTone(float v) noexcept { tone_.store(std::clamp(v, 0.0f, 1.0f), std::memory_order_relaxed); }
    void setLevel(float v) noexcept { level_.store(std::clamp(v, 0.0f, 1.5f), std::memory_order_relaxed); }

    void prepare(const guitardsp::graph::PrepareSpec& spec) override {
        sampleRate_ = spec.sampleRate; channels_ = std::clamp(spec.channels, 1, 2);
        for (int ch = 0; ch < 2; ++ch) {
            inputHp_[ch].prepare(sampleRate_); preEmphasis_[ch].prepare(sampleRate_);
            toneLow_[ch].prepare(sampleRate_); toneHigh_[ch].prepare(sampleRate_); dc_[ch].prepare(sampleRate_);
        }
        reset();
    }

    void reset() noexcept override {
        for (int ch = 0; ch < 2; ++ch) {
            inputHp_[ch].reset(); preEmphasis_[ch].reset(); toneLow_[ch].reset(); toneHigh_[ch].reset(); dc_[ch].reset();
        }
    }

    void process(const guitardsp::graph::AudioBuffer& input, guitardsp::graph::AudioBuffer& output, int numSamples) noexcept override {
        const float distortion = distortion_.load(std::memory_order_relaxed);
        const float tone = tone_.load(std::memory_order_relaxed);
        const float level = level_.load(std::memory_order_relaxed);
        const float gain = std::pow(10.0f, (20.0f + 28.0f * distortion) / 20.0f);
        const float clipMix = 0.72f + 0.25f * distortion;
        const int chs = std::min({channels_, input.channels(), output.channels()});

        for (int ch = 0; ch < chs; ++ch) {
            const float* src = input.channel(ch); float* dst = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) {
                const float coupled = inputHp_[ch].highPass(src[i], 24.0f);
                const float upper = preEmphasis_[ch].highPass(coupled, 720.0f);
                const float amplified = gain * (0.20f * coupled + 0.80f * upper);
                const float diode = solveAntiparallelDiodes(amplified, 2200.0f, 2.0e-9f, 0.052f);
                float clipped = amplified + clipMix * (diode - amplified);
                const float low = toneLow_[ch].lowPass(clipped, 950.0f);
                const float high = toneHigh_[ch].highPass(clipped, 1050.0f);
                const float lowWeight = 1.0f - 0.78f * tone;
                const float highWeight = 0.18f + 1.05f * tone;
                clipped = lowWeight * low + highWeight * high;
                dst[i] = dc_[ch].process(clipped * (0.08f + 0.92f * level));
            }
        }
    }

private:
    std::atomic<float> distortion_{0.55f}, tone_{0.58f}, level_{0.72f};
    double sampleRate_ = 48000.0;
    int channels_ = 2;
    std::array<OnePoleState, 2> inputHp_, preEmphasis_, toneLow_, toneHigh_;
    std::array<DcBlocker, 2> dc_;
};

} // namespace guitardsp::dsp
