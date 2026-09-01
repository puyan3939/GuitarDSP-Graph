#pragma once

#include "AudioNode.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <vector>

namespace guitardsp::graph {

class GainNode final : public AudioNode {
public:
    explicit GainNode(float gain = 1.0f) : gain_(gain) {}
    void setGain(float gain) noexcept { gain_.store(gain, std::memory_order_relaxed); }
    [[nodiscard]] float gain() const noexcept { return gain_.load(std::memory_order_relaxed); }
    std::string_view typeName() const noexcept override { return "Gain"; }
    std::size_t parameterCount() const noexcept override { return 1; }
    ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override { return i==0 ? ParameterDescriptor{"gain","Gain",0.0f,4.0f,1.0f,ParameterUnit::generic,1.0f} : ParameterDescriptor{}; }
    float parameterValue(std::size_t i) const noexcept override { return i==0 ? gain() : 0.0f; }
    bool setParameterValue(std::size_t i,float v) noexcept override { if(i!=0)return false; setGain(clampParameter(parameterDescriptor(0),v)); return true; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const float g = gain();
        output.copyFrom(input, numSamples);
        const int chs = std::min(input.channels(), output.channels());
        const int n = std::min(numSamples, output.samples());
        for (int ch = 0; ch < chs; ++ch) {
            float* d = output.channel(ch);
            for (int i = 0; i < n; ++i) d[i] *= g;
        }
    }
private:
    std::atomic<float> gain_{1.0f};
};

// Distinct names let the realtime host automate parallel branch balances
// independently without accidentally touching every generic gain in a graph.
class BranchLevelNode final : public AudioNode {
public:
    enum class Branch { guitar, bass };

    explicit BranchLevelNode(Branch branch = Branch::guitar, float level = 1.0f)
        : branch_(branch), level_(std::clamp(level, 0.0f, 2.0f)) {}

    std::string_view typeName() const noexcept override {
        return branch_ == Branch::guitar ? "Guitar Branch Level" : "Bass Branch Level";
    }
    std::size_t parameterCount() const noexcept override { return 1; }
    ParameterDescriptor parameterDescriptor(std::size_t index) const noexcept override {
        return index == 0
            ? ParameterDescriptor{"level", "Branch Level", 0.0f, 2.0f, 1.0f,
                                  ParameterUnit::generic, 1.0f}
            : ParameterDescriptor{};
    }
    float parameterValue(std::size_t index) const noexcept override {
        return index == 0 ? level_.load(std::memory_order_relaxed) : 0.0f;
    }
    bool setParameterValue(std::size_t index, float value) noexcept override {
        if (index != 0) return false;
        level_.store(clampParameter(parameterDescriptor(0), value), std::memory_order_relaxed);
        return true;
    }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        output.copyFrom(input, numSamples);
        const float level = level_.load(std::memory_order_relaxed);
        const int count = std::min(numSamples, output.samples());
        for (int channel = 0; channel < output.channels(); ++channel) {
            float* destination = output.channel(channel);
            for (int index = 0; index < count; ++index) destination[index] *= level;
        }
    }

private:
    Branch branch_;
    std::atomic<float> level_{1.0f};
};

class PolarityNode final : public AudioNode {
public:
    explicit PolarityNode(bool inverted = false) : inverted_(inverted) {}
    void setInverted(bool value) noexcept { inverted_.store(value, std::memory_order_relaxed); }
    std::string_view typeName() const noexcept override { return "Polarity"; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        const float g = inverted_.load(std::memory_order_relaxed) ? -1.0f : 1.0f;
        output.copyFrom(input, numSamples);
        const int chs = std::min(input.channels(), output.channels());
        for (int ch = 0; ch < chs; ++ch) {
            float* d = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) d[i] *= g;
        }
    }
private:
    std::atomic<bool> inverted_{false};
};

class PanNode final : public AudioNode {
public:
    explicit PanNode(float pan = 0.0f) : pan_(std::clamp(pan, -1.0f, 1.0f)) {}
    void setPan(float value) noexcept { pan_.store(std::clamp(value, -1.0f, 1.0f), std::memory_order_relaxed); }
    std::string_view typeName() const noexcept override { return "Pan"; }
    std::size_t parameterCount() const noexcept override { return 1; }
    ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override { return i==0 ? ParameterDescriptor{"pan","Pan",-1.0f,1.0f,0.0f,ParameterUnit::generic,1.0f} : ParameterDescriptor{}; }
    float parameterValue(std::size_t i) const noexcept override { return i==0 ? pan_.load(std::memory_order_relaxed) : 0.0f; }
    bool setParameterValue(std::size_t i,float v) noexcept override { if(i!=0)return false; setPan(v); return true; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        output.copyFrom(input, numSamples);
        if (output.channels() < 2) return;
        constexpr float halfPi = 1.57079632679489661923f;
        const float p = 0.5f * (pan_.load(std::memory_order_relaxed) + 1.0f);
        const float leftGain = std::cos(p * halfPi);
        const float rightGain = std::sin(p * halfPi);
        float* l = output.channel(0); float* r = output.channel(1);
        for (int i = 0; i < numSamples; ++i) { l[i] *= leftGain; r[i] *= rightGain; }
    }
private:
    std::atomic<float> pan_{0.0f};
};

class DelayNode final : public AudioNode {
public:
    explicit DelayNode(int delaySamples = 0) : delaySamples_(std::max(0, delaySamples)) {}
    std::string_view typeName() const noexcept override { return "Delay"; }
    void prepare(const PrepareSpec& spec) override {
        channels_ = std::max(1, spec.channels);
        const int size = std::max(2, delaySamples_ + spec.maximumBlockSize + 2);
        lines_.assign(static_cast<std::size_t>(channels_), std::vector<float>(static_cast<std::size_t>(size), 0.0f));
        write_.assign(static_cast<std::size_t>(channels_), 0);
    }
    void reset() noexcept override {
        for (auto& line : lines_) std::fill(line.begin(), line.end(), 0.0f);
        std::fill(write_.begin(), write_.end(), 0);
    }
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        if (delaySamples_ == 0) { output.copyFrom(input, numSamples); return; }
        const int chs = std::min({channels_, input.channels(), output.channels()});
        for (int ch = 0; ch < chs; ++ch) {
            auto& line = lines_[static_cast<std::size_t>(ch)];
            int& w = write_[static_cast<std::size_t>(ch)];
            const int size = static_cast<int>(line.size());
            const float* src = input.channel(ch); float* dst = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) {
                line[static_cast<std::size_t>(w)] = src[i];
                int r = w - delaySamples_;
                while (r < 0) r += size;
                dst[i] = line[static_cast<std::size_t>(r)];
                if (++w >= size) w = 0;
            }
        }
    }
    int latencySamples() const noexcept override { return delaySamples_; }
private:
    int delaySamples_ = 0;
    int channels_ = 0;
    std::vector<std::vector<float>> lines_;
    std::vector<int> write_;
};

class TapSink {
public:
    virtual ~TapSink() = default;
    virtual void push(const AudioBuffer& block, int numSamples) noexcept = 0;
};

class TapNode final : public AudioNode {
public:
    explicit TapNode(TapSink* sink = nullptr) : sink_(sink) {}
    void setSink(TapSink* sink) noexcept { sink_.store(sink, std::memory_order_release); }
    std::string_view typeName() const noexcept override { return "Tap"; }
    NodeCategory category() const noexcept override { return NodeCategory::analysis; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        output.copyFrom(input, numSamples);
        if (auto* sink = sink_.load(std::memory_order_acquire)) sink->push(output, numSamples);
    }
private:
    std::atomic<TapSink*> sink_{nullptr};
};

class SplitNode final : public AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Split"; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override { output.copyFrom(input, numSamples); }
};

class MergeNode final : public AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Merge"; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override { output.copyFrom(input, numSamples); }
};

class DirectOutNode final : public AudioNode {
public:
    explicit DirectOutNode(float level = 1.0f) : level_(level) {}
    void setLevel(float level) noexcept { level_.store(level, std::memory_order_relaxed); }
    std::string_view typeName() const noexcept override { return "DirectOut"; }
    std::size_t parameterCount() const noexcept override { return 1; }
    ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override { return i==0 ? ParameterDescriptor{"level","Level",0.0f,2.0f,1.0f,ParameterUnit::generic,1.0f} : ParameterDescriptor{}; }
    float parameterValue(std::size_t i) const noexcept override { return i==0 ? level_.load(std::memory_order_relaxed) : 0.0f; }
    bool setParameterValue(std::size_t i,float v) noexcept override { if(i!=0)return false; setLevel(clampParameter(parameterDescriptor(0),v)); return true; }
    NodeCategory category() const noexcept override { return NodeCategory::io; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        output.copyFrom(input, numSamples);
        const float g = level_.load(std::memory_order_relaxed);
        for (int ch = 0; ch < output.channels(); ++ch) {
            float* d = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) d[i] *= g;
        }
    }
private:
    std::atomic<float> level_{1.0f};
};

} // namespace guitardsp::graph
