#pragma once

#include "guitardsp/graph/AudioNode.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace guitardsp::dsp {

using guitardsp::graph::AudioBuffer;
using guitardsp::graph::AudioNode;
using guitardsp::graph::NodeCategory;
using guitardsp::graph::PrepareSpec;

class OnePoleFilterNode final : public AudioNode {
public:
    enum class Mode { lowPass, highPass };
    explicit OnePoleFilterNode(Mode mode, float cutoffHz = 1000.0f) : mode_(mode), cutoffHz_(cutoffHz) {}
    void setCutoffHz(float hz) noexcept { cutoffHz_.store(hz, std::memory_order_relaxed); }
    std::string_view typeName() const noexcept override { return mode_ == Mode::lowPass ? "LowPass" : "HighPass"; }
    void prepare(const PrepareSpec& spec) override { sampleRate_ = spec.sampleRate; channels_ = std::clamp(spec.channels, 1, 2); reset(); }
    void reset() noexcept override { z_.fill(0.0f); }
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const float hz = std::clamp(cutoffHz_.load(std::memory_order_relaxed), 5.0f, static_cast<float>(0.45 * sampleRate_));
        const float a = std::exp(-2.0f * 3.14159265358979323846f * hz / static_cast<float>(sampleRate_));
        const int chs = std::min({channels_, input.channels(), output.channels()});
        for (int ch = 0; ch < chs; ++ch) {
            const float* src = input.channel(ch); float* dst = output.channel(ch); float z = z_[static_cast<std::size_t>(ch)];
            for (int i = 0; i < numSamples; ++i) {
                z = (1.0f - a) * src[i] + a * z;
                dst[i] = mode_ == Mode::lowPass ? z : src[i] - z;
            }
            z_[static_cast<std::size_t>(ch)] = z;
        }
    }
private:
    Mode mode_;
    std::atomic<float> cutoffHz_{1000.0f};
    double sampleRate_ = 48000.0;
    int channels_ = 2;
    std::array<float, 2> z_{};
};

class CompressorNode final : public AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Compressor"; }
    NodeCategory category() const noexcept override { return NodeCategory::dynamics; }
    void prepare(const PrepareSpec& spec) override { sampleRate_ = spec.sampleRate; channels_ = std::clamp(spec.channels, 1, 2); reset(); }
    void reset() noexcept override { envelope_.fill(0.0f); sidechainLow_.fill(0.0f); }

    void setThresholdDb(float v) noexcept { thresholdDb_.store(v, std::memory_order_relaxed); }
    void setRatio(float v) noexcept { ratio_.store(v, std::memory_order_relaxed); }
    void setAttackMs(float v) noexcept { attackMs_.store(v, std::memory_order_relaxed); }
    void setReleaseMs(float v) noexcept { releaseMs_.store(v, std::memory_order_relaxed); }
    void setMakeupDb(float v) noexcept { makeupDb_.store(v, std::memory_order_relaxed); }
    void setMix(float v) noexcept { mix_.store(v, std::memory_order_relaxed); }
    void setSidechainHpHz(float v) noexcept { sidechainHpHz_.store(v, std::memory_order_relaxed); }

    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const float threshold = std::clamp(thresholdDb_.load(std::memory_order_relaxed), -80.0f, 0.0f);
        const float ratio = std::clamp(ratio_.load(std::memory_order_relaxed), 1.0f, 20.0f);
        const float attack = std::clamp(attackMs_.load(std::memory_order_relaxed), 0.05f, 200.0f);
        const float release = std::clamp(releaseMs_.load(std::memory_order_relaxed), 5.0f, 3000.0f);
        const float makeup = dbToGain(std::clamp(makeupDb_.load(std::memory_order_relaxed), -18.0f, 18.0f));
        const float mix = std::clamp(mix_.load(std::memory_order_relaxed), 0.0f, 1.0f);
        const float hp = std::clamp(sidechainHpHz_.load(std::memory_order_relaxed), 10.0f, 1000.0f);
        const float hpA = std::exp(-2.0f * pi * hp / static_cast<float>(sampleRate_));
        const float attackA = timeCoeff(attack);
        const float releaseA = timeCoeff(release);
        const int chs = std::min({channels_, input.channels(), output.channels()});

        for (int ch = 0; ch < chs; ++ch) {
            const float* src = input.channel(ch); float* dst = output.channel(ch);
            float env = envelope_[static_cast<std::size_t>(ch)];
            float low = sidechainLow_[static_cast<std::size_t>(ch)];
            for (int i = 0; i < numSamples; ++i) {
                low = (1.0f - hpA) * src[i] + hpA * low;
                const float detector = std::abs(src[i] - low);
                const float coeff = detector > env ? attackA : releaseA;
                env = coeff * env + (1.0f - coeff) * detector;
                const float envDb = 20.0f * std::log10(std::max(env, 1.0e-9f));
                const float over = std::max(0.0f, envDb - threshold);
                const float gainDb = -over * (1.0f - 1.0f / ratio);
                const float wet = src[i] * dbToGain(gainDb) * makeup;
                dst[i] = src[i] + mix * (wet - src[i]);
            }
            envelope_[static_cast<std::size_t>(ch)] = env;
            sidechainLow_[static_cast<std::size_t>(ch)] = low;
        }
    }

private:
    static constexpr float pi = 3.14159265358979323846f;
    [[nodiscard]] float timeCoeff(float ms) const noexcept {
        return std::exp(-1.0f / (0.001f * ms * static_cast<float>(sampleRate_)));
    }
    static float dbToGain(float db) noexcept { return std::pow(10.0f, db / 20.0f); }

    std::atomic<float> thresholdDb_{-18.0f}, ratio_{4.0f}, attackMs_{12.0f}, releaseMs_{120.0f};
    std::atomic<float> makeupDb_{0.0f}, mix_{1.0f}, sidechainHpHz_{60.0f};
    double sampleRate_ = 48000.0;
    int channels_ = 2;
    std::array<float, 2> envelope_{};
    std::array<float, 2> sidechainLow_{};
};

class TransientEnhancerNode final : public AudioNode {
public:
    std::string_view typeName() const noexcept override { return "TransientEnhancer"; }
    NodeCategory category() const noexcept override { return NodeCategory::dynamics; }
    void prepare(const PrepareSpec& spec) override { sampleRate_ = spec.sampleRate; channels_ = std::clamp(spec.channels, 1, 2); reset(); }
    void reset() noexcept override { fast_.fill(0.0f); slow_.fill(0.0f); hpLow_.fill(0.0f); }

    void setAmount(float v) noexcept { amount_.store(v, std::memory_order_relaxed); }
    void setBrightnessHz(float v) noexcept { brightnessHz_.store(v, std::memory_order_relaxed); }
    void setDecayMs(float v) noexcept { decayMs_.store(v, std::memory_order_relaxed); }
    void setDrive(float v) noexcept { drive_.store(v, std::memory_order_relaxed); }

    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const float amount = std::clamp(amount_.load(std::memory_order_relaxed), 0.0f, 2.0f);
        const float hpHz = std::clamp(brightnessHz_.load(std::memory_order_relaxed), 500.0f, 10000.0f);
        const float decayMs = std::clamp(decayMs_.load(std::memory_order_relaxed), 8.0f, 250.0f);
        const float drive = std::clamp(drive_.load(std::memory_order_relaxed), 0.0f, 4.0f);
        const float fastA = std::exp(-1.0f / (0.001f * 1.5f * static_cast<float>(sampleRate_)));
        const float slowA = std::exp(-1.0f / (0.001f * decayMs * static_cast<float>(sampleRate_)));
        const float hpA = std::exp(-2.0f * pi * hpHz / static_cast<float>(sampleRate_));
        const int chs = std::min({channels_, input.channels(), output.channels()});

        for (int ch = 0; ch < chs; ++ch) {
            const float* src = input.channel(ch); float* dst = output.channel(ch);
            float fast = fast_[static_cast<std::size_t>(ch)];
            float slow = slow_[static_cast<std::size_t>(ch)];
            float low = hpLow_[static_cast<std::size_t>(ch)];
            for (int i = 0; i < numSamples; ++i) {
                const float mag = std::abs(src[i]);
                fast = fastA * fast + (1.0f - fastA) * mag;
                slow = slowA * slow + (1.0f - slowA) * mag;
                const float transient = std::clamp((fast - slow) * 8.0f, 0.0f, 1.0f);
                low = (1.0f - hpA) * src[i] + hpA * low;
                const float high = src[i] - low;
                const float harmonics = std::tanh(high * (1.0f + drive * 3.0f));
                dst[i] = src[i] + amount * transient * harmonics;
            }
            fast_[static_cast<std::size_t>(ch)] = fast;
            slow_[static_cast<std::size_t>(ch)] = slow;
            hpLow_[static_cast<std::size_t>(ch)] = low;
        }
    }
private:
    static constexpr float pi = 3.14159265358979323846f;
    std::atomic<float> amount_{0.45f}, brightnessHz_{2200.0f}, decayMs_{55.0f}, drive_{0.35f};
    double sampleRate_ = 48000.0;
    int channels_ = 2;
    std::array<float, 2> fast_{};
    std::array<float, 2> slow_{};
    std::array<float, 2> hpLow_{};
};

} // namespace guitardsp::dsp
