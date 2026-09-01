#pragma once

#include "AudioNode.h"
#include <algorithm>
#include <atomic>

namespace guitardsp::graph {

class OutputBusNode final : public AudioNode {
public:
    explicit OutputBusNode(int bus = 0) : bus_(std::max(0, bus)) {}
    std::string_view typeName() const noexcept override { return "OutputBus"; }
    NodeCategory category() const noexcept override { return NodeCategory::io; }
    int physicalOutputBusIndex() const noexcept override { return bus_.load(std::memory_order_relaxed); }
    std::size_t parameterCount() const noexcept override { return 2; }
    ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        static constexpr ParameterDescriptor p[] = {
            {"bus","Output Bus",0.0f,15.0f,0.0f,ParameterUnit::generic,1.0f},
            {"level","Level",0.0f,2.0f,1.0f,ParameterUnit::generic,1.0f}};
        return i < 2 ? p[i] : ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        if (i == 0) return static_cast<float>(bus_.load(std::memory_order_relaxed));
        if (i == 1) return level_.load(std::memory_order_relaxed);
        return 0.0f;
    }
    bool setParameterValue(std::size_t i, float value) noexcept override {
        if (i == 0) { bus_.store(static_cast<int>(clampParameter(parameterDescriptor(0), value) + 0.5f), std::memory_order_relaxed); return true; }
        if (i == 1) { level_.store(clampParameter(parameterDescriptor(1), value), std::memory_order_relaxed); return true; }
        return false;
    }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept override {
        output.copyFrom(input, numSamples);
        const float gain = level_.load(std::memory_order_relaxed);
        for (int ch = 0; ch < output.channels(); ++ch) {
            float* d = output.channel(ch);
            for (int i = 0; i < numSamples; ++i) d[i] *= gain;
        }
    }
private:
    std::atomic<int> bus_{0};
    std::atomic<float> level_{1.0f};
};

} // namespace guitardsp::graph
