#pragma once
#include "AudioNode.h"
#include "AudioBuffer.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace guitardsp::graph {

class GainNode final : public AudioNode {
public:
    explicit GainNode(float gain = 1.0f) : gain_(gain) {}
    void setGain(float g) noexcept { gain_ = g; }
    float gain() const noexcept { return gain_; }
    const char* name() const noexcept override { return "Gain"; }
    int latencySamples() const noexcept override { return 0; }
    void prepare(double, int, int) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const int channels = std::min(input.channels(), output.channels());
        for (int ch = 0; ch < channels; ++ch) {
            const auto* in = input.channel(ch);
            auto* out = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) out[i] = in[i] * gain_;
        }
    }
private:
    float gain_ = 1.0f;
};

class PolarityNode final : public AudioNode {
public:
    explicit PolarityNode(bool inverted = false) : inverted_(inverted) {}
    void setInverted(bool v) noexcept { inverted_ = v; }
    const char* name() const noexcept override { return "Polarity"; }
    int latencySamples() const noexcept override { return 0; }
    void prepare(double, int, int) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const float g = inverted_ ? -1.0f : 1.0f;
        const int channels = std::min(input.channels(), output.channels());
        for (int ch = 0; ch < channels; ++ch) {
            const auto* in = input.channel(ch);
            auto* out = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) out[i] = in[i] * g;
        }
    }
private:
    bool inverted_ = false;
};

class DelayNode final : public AudioNode {
public:
    explicit DelayNode(int delaySamples = 0) : delaySamples_(std::max(0, delaySamples)) {}
    void setDelaySamples(int samples) noexcept { delaySamples_ = std::max(0, samples); }
    const char* name() const noexcept override { return "Delay"; }
    int latencySamples() const noexcept override { return delaySamples_; }
    void prepare(double, int maxBlockSize, int channels) override {
        maxBlock_ = maxBlockSize;
        channels_ = channels;
        const int size = std::max(1, delaySamples_ + maxBlockSize + 1);
        lines_.assign(static_cast<std::size_t>(channels_), std::vector<float>(static_cast<std::size_t>(size), 0.0f));
        write_.assign(static_cast<std::size_t>(channels_), 0);
    }
    void reset() noexcept override {
        for (auto& l : lines_) std::fill(l.begin(), l.end(), 0.0f);
        std::fill(write_.begin(), write_.end(), 0);
    }
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const int channels = std::min({input.channels(), output.channels(), channels_});
        if (delaySamples_ == 0) { output.copyFrom(input); return; }
        for (int ch = 0; ch < channels; ++ch) {
            auto& line = lines_[static_cast<std::size_t>(ch)];
            auto& w = write_[static_cast<std::size_t>(ch)];
            const auto* in = input.channel(ch);
            auto* out = output.channel(ch);
            const int size = static_cast<int>(line.size());
            for (int i = 0; i < numSamples; ++i) {
                line[static_cast<std::size_t>(w)] = in[i];
                int r = w - delaySamples_;
                while (r < 0) r += size;
                out[i] = line[static_cast<std::size_t>(r)];
                if (++w >= size) w = 0;
            }
        }
    }
private:
    int delaySamples_ = 0;
    int maxBlock_ = 0;
    int channels_ = 0;
    std::vector<std::vector<float>> lines_;
    std::vector<int> write_;
};

class PassthroughNode final : public AudioNode {
public:
    const char* name() const noexcept override { return "Passthrough"; }
    int latencySamples() const noexcept override { return 0; }
    void prepare(double, int, int) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int) noexcept override { output.copyFrom(input); }
};

} // namespace guitardsp::graph
