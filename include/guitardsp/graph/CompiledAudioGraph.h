#pragma once
#include "AudioBuffer.h"
#include "Graph.h"
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace guitardsp::graph {

class CompiledAudioGraph {
public:
    struct DelayLine {
        std::vector<float> left, right;
        std::size_t write = 0;
        std::size_t delay = 0;
        void prepare(std::size_t delaySamples, std::size_t maxBlockSize);
        void reset() noexcept;
        void addDelayed(const AudioBuffer& source, AudioBuffer& dest, std::size_t numSamples) noexcept;
    };

    struct UpstreamRuntime {
        NodeId id = 0;
        DelayLine compensation;
    };

    struct NodeRuntime {
        NodeId id = 0;
        AudioNode* node = nullptr;
        std::vector<UpstreamRuntime> upstream;
        AudioBuffer mixInput;
        AudioBuffer output;
    };

    bool build(Graph& graph, double sampleRate, std::size_t maxBlockSize);
    void reset() noexcept;
    void process(const AudioBuffer& externalInput, AudioBuffer& externalOutput, std::size_t numSamples) noexcept;

    [[nodiscard]] std::size_t totalLatencySamples() const noexcept { return totalLatencySamples_; }
    [[nodiscard]] const std::vector<NodeId>& order() const noexcept { return order_; }

private:
    NodeRuntime* runtime(NodeId id) noexcept;
    const NodeRuntime* runtime(NodeId id) const noexcept;

    std::vector<NodeId> order_;
    std::vector<NodeRuntime> nodes_;
    std::unordered_map<NodeId, std::size_t> runtimeIndex_;
    std::vector<NodeId> sinks_;
    double sampleRate_ = 48000.0;
    std::size_t maxBlockSize_ = 0;
    std::size_t totalLatencySamples_ = 0;
};

} // namespace guitardsp::graph
