#pragma once
#include "AudioNode.h"
#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

namespace guitardsp::graph {

inline void copyBlock(const AudioBlock& input, AudioBlock& output, std::size_t n) noexcept {
    std::copy_n(input.left.data(), n, output.left.data());
    std::copy_n(input.right.data(), n, output.right.data());
}

class GainNode final : public AudioNode {
public:
    explicit GainNode(float gain = 1.0f) : gain_(gain) {}
    void setGain(float g) noexcept { gain_ = g; }
    [[nodiscard]] float gain() const noexcept { return gain_; }
    [[nodiscard]] std::string_view typeName() const noexcept override { return "Gain"; }
    void prepare(double, std::size_t) override {}
    void reset() noexcept override {}
    void process(const ProcessContext& c, const AudioBlock& input, AudioBlock& output) noexcept override {
        for (std::size_t i = 0; i < c.numSamples; ++i) {
            output.left[i] = input.left[i] * gain_;
            output.right[i] = input.right[i] * gain_;
        }
    }
private:
    float gain_ = 1.0f;
};

class PolarityNode final : public AudioNode {
public:
    explicit PolarityNode(bool inverted = false) : inverted_(inverted) {}
    void setInverted(bool v) noexcept { inverted_ = v; }
    [[nodiscard]] std::string_view typeName() const noexcept override { return "Polarity"; }
    void prepare(double, std::size_t) override {}
    void reset() noexcept override {}
    void process(const ProcessContext& c, const AudioBlock& input, AudioBlock& output) noexcept override {
        const float g = inverted_ ? -1.0f : 1.0f;
        for (std::size_t i = 0; i < c.numSamples; ++i) {
            output.left[i] = input.left[i] * g;
            output.right[i] = input.right[i] * g;
        }
    }
private:
    bool inverted_ = false;
};

class DelayNode final : public AudioNode {
public:
    explicit DelayNode(std::size_t delaySamples = 0) : delaySamples_(delaySamples) {}
    void setDelaySamples(std::size_t samples) noexcept { delaySamples_ = samples; }
    [[nodiscard]] std::string_view typeName() const noexcept override { return "Delay"; }
    [[nodiscard]] std::size_t latencySamples() const noexcept override { return delaySamples_; }
    void prepare(double, std::size_t maximumBlockSize) override {
        const auto size = std::max<std::size_t>(1, delaySamples_ + maximumBlockSize + 1);
        left_.assign(size, 0.0f); right_.assign(size, 0.0f); write_ = 0;
    }
    void reset() noexcept override {
        std::fill(left_.begin(), left_.end(), 0.0f);
        std::fill(right_.begin(), right_.end(), 0.0f);
        write_ = 0;
    }
    void process(const ProcessContext& c, const AudioBlock& input, AudioBlock& output) noexcept override {
        if (delaySamples_ == 0) { copyBlock(input, output, c.numSamples); return; }
        const auto size = left_.size();
        for (std::size_t i = 0; i < c.numSamples; ++i) {
            left_[write_] = input.left[i]; right_[write_] = input.right[i];
            const auto read = (write_ + size - (delaySamples_ % size)) % size;
            output.left[i] = left_[read]; output.right[i] = right_[read];
            write_ = (write_ + 1) % size;
        }
    }
private:
    std::size_t delaySamples_ = 0, write_ = 0;
    std::vector<float> left_, right_;
};

class PassthroughNode final : public AudioNode {
public:
    [[nodiscard]] std::string_view typeName() const noexcept override { return "Passthrough"; }
    void prepare(double, std::size_t) override {}
    void reset() noexcept override {}
    void process(const ProcessContext& c, const AudioBlock& input, AudioBlock& output) noexcept override { copyBlock(input, output, c.numSamples); }
};

} // namespace guitardsp::graph
