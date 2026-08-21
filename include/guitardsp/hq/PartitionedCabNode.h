#pragma once

#include "StreamingConvolver.h"
#include "guitardsp/graph/AudioNode.h"
#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

namespace guitardsp::hq {

// Long-IR cabinet node built on the streaming partitioned convolver.
// IR changes are control-thread operations. Audio processing is allocation-free.
class PartitionedCabNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Partitioned Cab IR"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::cab; }

    void setImpulseResponse(std::vector<float> impulse) {
        if (impulse.empty()) impulse.push_back(1.0f);
        impulse_ = std::move(impulse);
        if (prepared_) {
            for (auto& convolver : channels_) convolver.setImpulseResponse(impulse_);
        }
    }

    void setPartitionSize(int samples) noexcept { requestedPartitionSize_ = std::clamp(samples, 16, 1024); }

    void prepare(const graph::PrepareSpec& spec) override {
        channels_.assign(static_cast<std::size_t>(std::max(1, spec.channels)), {});
        const int maximumImpulse = std::max(requestedPartitionSize_, static_cast<int>(impulse_.size()));
        for (auto& convolver : channels_) {
            convolver.prepare(requestedPartitionSize_, maximumImpulse);
            convolver.setImpulseResponse(impulse_);
        }
        prepared_ = true;
    }

    void reset() noexcept override {
        for (auto& convolver : channels_) convolver.reset();
    }

    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples) noexcept override {
        const int channels = std::min({input.channels(), output.channels(), static_cast<int>(channels_.size())});
        for (int ch = 0; ch < channels; ++ch)
            channels_[static_cast<std::size_t>(ch)].process(input.channel(ch), output.channel(ch), numSamples);
        for (int ch = channels; ch < output.channels(); ++ch)
            std::fill(output.channel(ch), output.channel(ch) + numSamples, 0.0f);
    }

    int latencySamples() const noexcept override { return requestedPartitionSize_; }

private:
    std::vector<float> impulse_ {1.0f};
    std::vector<StreamingPartitionedConvolver> channels_;
    int requestedPartitionSize_ = 64;
    bool prepared_ = false;
};

} // namespace guitardsp::hq
