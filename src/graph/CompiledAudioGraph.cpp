#include "guitardsp/graph/CompiledAudioGraph.h"

#include <algorithm>
#include <unordered_map>
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

bool CompiledAudioGraph::build(Graph& graph, double sampleRate, int maxBlockSize, int channels,
                               ProcessingQuality quality) {
    if (maxBlockSize <= 0 || channels <= 0) return false;
    const auto result = graph.compile();
    if (!result.ok) return false;

    maxBlockSize_ = maxBlockSize;
    channels_ = channels;
    totalLatencySamples_ = graph.maximumGraphLatencySamples();
    order_ = graph.schedule();
    nodes_.clear(); runtimeIndex_.clear(); sinks_.clear();
    nodes_.reserve(order_.size());

    std::unordered_map<NodeId, std::vector<NodeId>> incoming;
    std::unordered_set<NodeId> hasOutgoing;
    for (const auto& c : graph.connections()) {
        incoming[c.to].push_back(c.from);
        hasOutgoing.insert(c.from);
    }

    const PrepareSpec spec{sampleRate, maxBlockSize, channels, quality};
    for (const NodeId id : order_) {
        auto* node = graph.node(id);
        if (node == nullptr) return false;
        NodeRuntime r;
        r.id = id;
        r.node = node;
        r.mixInput.resize(channels, maxBlockSize);
        r.output.resize(channels, maxBlockSize);

        int maxParentLatency = 0;
        if (const auto it = incoming.find(id); it != incoming.end()) {
            for (const NodeId parent : it->second)
                maxParentLatency = std::max(maxParentLatency, graph.cumulativeLatencySamples(parent).value_or(0));
            r.upstream.reserve(it->second.size());
            for (const NodeId parent : it->second) {
                InputEdgeRuntime edge;
                edge.source = parent;
                edge.compensation.prepare(channels, totalLatencySamples_, maxBlockSize);
                edge.compensation.setDelaySamples(maxParentLatency - graph.cumulativeLatencySamples(parent).value_or(0));
                r.upstream.push_back(std::move(edge));
            }
        }
        node->prepare(spec);
        runtimeIndex_[id] = nodes_.size();
        nodes_.push_back(std::move(r));
    }

    for (const NodeId id : order_) {
        if (hasOutgoing.contains(id)) continue;
        SinkRuntime sink;
        sink.source = id;
        sink.compensation.prepare(channels, totalLatencySamples_, maxBlockSize);
        sink.compensation.setDelaySamples(totalLatencySamples_ - graph.cumulativeLatencySamples(id).value_or(0));
        sinks_.push_back(std::move(sink));
    }

    reset();
    return !nodes_.empty() && !sinks_.empty();
}

void CompiledAudioGraph::reset() noexcept {
    for (auto& r : nodes_) {
        r.mixInput.clear(); r.output.clear();
        for (auto& edge : r.upstream) edge.compensation.reset();
        if (r.node != nullptr) r.node->reset();
    }
    for (auto& sink : sinks_) sink.compensation.reset();
}

void CompiledAudioGraph::process(const AudioBuffer& externalInput, AudioBuffer& externalOutput, int numSamples) noexcept {
    if (numSamples <= 0 || numSamples > maxBlockSize_) return;
    externalOutput.clear(numSamples);

    for (auto& r : nodes_) {
        r.mixInput.clear(numSamples);
        if (r.upstream.empty()) {
            r.mixInput.copyFrom(externalInput, numSamples);
        } else {
            for (auto& edge : r.upstream)
                if (const auto* u = runtime(edge.source)) edge.compensation.processAdd(u->output, r.mixInput, numSamples);
        }

        r.output.clear(numSamples);
        if (r.node->isMuted()) {
            continue;
        }
        if (r.node->isBypassed()) {
            r.output.copyFrom(r.mixInput, numSamples);
            continue;
        }
        r.node->process(r.mixInput, r.output, numSamples);
    }

    for (auto& sink : sinks_)
        if (const auto* s = runtime(sink.source)) sink.compensation.processAdd(s->output, externalOutput, numSamples);
}

} // namespace guitardsp::graph
