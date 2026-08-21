#pragma once
#include "AudioBuffer.h"
#include "Graph.h"
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace guitardsp::graph {

class CompiledAudioGraph {
public:
    struct NodeRuntime {
        NodeId id = 0;
        AudioNode* node = nullptr;
        std::vector<NodeId> upstream;
        AudioBuffer mixInput;
        AudioBuffer output;
    };

    bool build(Graph& graph, double sampleRate, int maxBlockSize, int channels);
    void reset() noexcept;
    void process(const AudioBuffer& externalInput, AudioBuffer& externalOutput, int numSamples) noexcept;

    int totalLatencySamples() const noexcept { return totalLatencySamples_; }
    const std::vector<NodeId>& order() const noexcept { return order_; }

private:
    NodeRuntime* runtime(NodeId id) noexcept;
    const NodeRuntime* runtime(NodeId id) const noexcept;

    std::vector<NodeId> order_;
    std::vector<NodeRuntime> nodes_;
    std::unordered_map<NodeId, std::size_t> runtimeIndex_;
    std::vector<NodeId> sinks_;
    int maxBlockSize_ = 0;
    int channels_ = 0;
    int totalLatencySamples_ = 0;
};

} // namespace guitardsp::graph
