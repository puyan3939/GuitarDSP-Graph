#pragma once

#include "AudioBuffer.h"
#include "DelayCompensator.h"
#include "Graph.h"
#include <unordered_map>
#include <vector>

namespace guitardsp::graph {

class CompiledAudioGraph {
public:
    struct InputEdgeRuntime { NodeId source=0; int sourcePort=0; DelayCompensator compensation; };
    struct InputPortRuntime { std::vector<InputEdgeRuntime> upstream; AudioBuffer mix; };
    struct NodeRuntime {
        NodeId id=0; AudioNode* node=nullptr;
        std::vector<InputPortRuntime> inputs;
        std::vector<AudioBuffer> outputs;
        std::vector<const AudioBuffer*> inputPointers;
        std::vector<AudioBuffer*> outputPointers;
    };
    struct SinkRuntime { NodeId source=0; int sourcePort=0; DelayCompensator compensation; };

    bool build(Graph& graph,double sampleRate,int maxBlockSize,int channels,ProcessingQuality quality=ProcessingQuality::high);
    void reset() noexcept;
    void process(const AudioBuffer& externalInput,AudioBuffer& externalOutput,int numSamples) noexcept;
    [[nodiscard]] int totalLatencySamples()const noexcept{return totalLatencySamples_;}
    [[nodiscard]] const std::vector<NodeId>& order()const noexcept{return order_;}
private:
    NodeRuntime* runtime(NodeId id)noexcept;
    const NodeRuntime* runtime(NodeId id)const noexcept;
    std::vector<NodeId> order_;std::vector<NodeRuntime> nodes_;std::unordered_map<NodeId,std::size_t> runtimeIndex_;std::vector<SinkRuntime>sinks_;
    int maxBlockSize_=0,channels_=0,totalLatencySamples_=0;
};

} // namespace guitardsp::graph
