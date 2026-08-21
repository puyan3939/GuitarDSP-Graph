#include "guitardsp/graph/CompiledAudioGraph.h"
#include <algorithm>
#include <unordered_set>

namespace guitardsp::graph {

CompiledAudioGraph::NodeRuntime* CompiledAudioGraph::runtime(NodeId id) noexcept {
    const auto it = runtimeIndex_.find(id);
    return it == runtimeIndex_.end() ? nullptr : &nodes_[it->second];
}
const CompiledAudioGraph::NodeRuntime* CompiledAudioGraph::runtime(NodeId id) const noexcept {
    const auto it = runtimeIndex_.find(id);
    return it == runtimeIndex_.end() ? nullptr : &nodes_[it->second];
}

bool CompiledAudioGraph::build(Graph& graph, double sampleRate, int maxBlockSize, int channels) {
    const auto plan = graph.compile();
    if (!plan.valid) return false;

    maxBlockSize_ = maxBlockSize;
    channels_ = channels;
    totalLatencySamples_ = plan.totalLatencySamples;
    order_ = plan.order;
    nodes_.clear(); runtimeIndex_.clear(); sinks_.clear();
    nodes_.reserve(order_.size());

    const auto& connections = graph.connections();
    std::unordered_set<NodeId> hasOutgoing;
    for (const auto& c : connections) hasOutgoing.insert(c.from);

    for (const auto id : order_) {
        auto* node = graph.node(id);
        if (node == nullptr) return false;
        NodeRuntime r;
        r.id = id; r.node = node;
        r.mixInput.resize(channels, maxBlockSize);
        r.output.resize(channels, maxBlockSize);
        for (const auto& c : connections) if (c.to == id) r.upstream.push_back(c.from);
        node->prepare(sampleRate, maxBlockSize, channels);
        runtimeIndex_[id] = nodes_.size();
        nodes_.push_back(std::move(r));
        if (hasOutgoing.find(id) == hasOutgoing.end()) sinks_.push_back(id);
    }
    return !nodes_.empty();
}

void CompiledAudioGraph::reset() noexcept {
    for (auto& r : nodes_) {
        r.mixInput.clear(); r.output.clear();
        if (r.node != nullptr) r.node->reset();
    }
}

void CompiledAudioGraph::process(const AudioBuffer& externalInput, AudioBuffer& externalOutput, int numSamples) noexcept {
    if (numSamples <= 0 || numSamples > maxBlockSize_) return;
    externalOutput.clear();
    for (auto& r : nodes_) {
        r.mixInput.clear();
        if (r.upstream.empty()) {
            r.mixInput.copyFrom(externalInput);
        } else {
            for (const auto upstreamId : r.upstream) {
                if (const auto* u = runtime(upstreamId)) r.mixInput.addFrom(u->output);
            }
        }
        r.output.clear();
        r.node->process(r.mixInput, r.output, numSamples);
    }
    if (sinks_.empty()) return;
    for (const auto sinkId : sinks_) if (const auto* s = runtime(sinkId)) externalOutput.addFrom(s->output);
}

} // namespace guitardsp::graph
