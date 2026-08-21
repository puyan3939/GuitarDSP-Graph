#include "guitardsp/graph/CompiledAudioGraph.h"
#include <algorithm>
#include <unordered_set>

namespace guitardsp::graph {

void CompiledAudioGraph::DelayLine::prepare(std::size_t delaySamples, std::size_t maxBlockSize) {
    delay = delaySamples;
    const auto size = std::max<std::size_t>(1, delay + maxBlockSize + 1);
    left.assign(size, 0.0f); right.assign(size, 0.0f); write = 0;
}
void CompiledAudioGraph::DelayLine::reset() noexcept {
    std::fill(left.begin(), left.end(), 0.0f);
    std::fill(right.begin(), right.end(), 0.0f);
    write = 0;
}
void CompiledAudioGraph::DelayLine::addDelayed(const AudioBuffer& source, AudioBuffer& dest, std::size_t numSamples) noexcept {
    if (delay == 0) { dest.addFrom(source); return; }
    const auto size = left.size();
    const auto* sl = source.channel(0); const auto* sr = source.channel(1);
    auto* dl = dest.channel(0); auto* dr = dest.channel(1);
    for (std::size_t i = 0; i < numSamples; ++i) {
        left[write] = sl[i]; right[write] = sr[i];
        const auto read = (write + size - (delay % size)) % size;
        dl[i] += left[read]; dr[i] += right[read];
        write = (write + 1) % size;
    }
}

CompiledAudioGraph::NodeRuntime* CompiledAudioGraph::runtime(NodeId id) noexcept {
    const auto it = runtimeIndex_.find(id);
    return it == runtimeIndex_.end() ? nullptr : &nodes_[it->second];
}
const CompiledAudioGraph::NodeRuntime* CompiledAudioGraph::runtime(NodeId id) const noexcept {
    const auto it = runtimeIndex_.find(id);
    return it == runtimeIndex_.end() ? nullptr : &nodes_[it->second];
}

bool CompiledAudioGraph::build(Graph& graph, double sampleRate, std::size_t maxBlockSize) {
    const auto result = graph.compile();
    if (!result.ok || maxBlockSize == 0) return false;

    sampleRate_ = sampleRate; maxBlockSize_ = maxBlockSize;
    totalLatencySamples_ = graph.maximumGraphLatencySamples();
    order_ = graph.schedule();
    nodes_.clear(); runtimeIndex_.clear(); sinks_.clear();
    nodes_.reserve(order_.size());

    const auto& connections = graph.connections();
    std::unordered_set<NodeId> hasOutgoing;
    for (const auto& c : connections) hasOutgoing.insert(c.from);

    for (const auto id : order_) {
        auto* n = graph.node(id);
        if (n == nullptr) return false;
        NodeRuntime r; r.id = id; r.node = n;
        r.mixInput.resize(2, static_cast<int>(maxBlockSize));
        r.output.resize(2, static_cast<int>(maxBlockSize));

        std::size_t maxParentLatency = 0;
        for (const auto& c : connections) if (c.to == id) {
            const auto l = graph.cumulativeLatencySamples(c.from).value_or(0);
            maxParentLatency = std::max(maxParentLatency, l);
        }
        for (const auto& c : connections) if (c.to == id) {
            UpstreamRuntime u; u.id = c.from;
            const auto l = graph.cumulativeLatencySamples(c.from).value_or(0);
            u.compensation.prepare(maxParentLatency - l, maxBlockSize);
            r.upstream.push_back(std::move(u));
        }

        n->prepare(sampleRate, maxBlockSize);
        runtimeIndex_[id] = nodes_.size();
        nodes_.push_back(std::move(r));
        if (!hasOutgoing.contains(id)) sinks_.push_back(id);
    }
    return !nodes_.empty();
}

void CompiledAudioGraph::reset() noexcept {
    for (auto& r : nodes_) {
        r.mixInput.clear(); r.output.clear();
        for (auto& u : r.upstream) u.compensation.reset();
        if (r.node != nullptr) r.node->reset();
    }
}

void CompiledAudioGraph::process(const AudioBuffer& externalInput, AudioBuffer& externalOutput, std::size_t numSamples) noexcept {
    if (numSamples == 0 || numSamples > maxBlockSize_ || externalInput.channels() < 2 || externalOutput.channels() < 2) return;
    externalOutput.clear();
    const ProcessContext context{sampleRate_, numSamples};

    for (auto& r : nodes_) {
        r.mixInput.clear();
        if (r.upstream.empty()) r.mixInput.copyFrom(externalInput);
        else for (auto& upstream : r.upstream)
            if (const auto* source = runtime(upstream.id)) upstream.compensation.addDelayed(source->output, r.mixInput, numSamples);

        r.output.clear();
        AudioBlock in{{r.mixInput.channel(0), numSamples}, {r.mixInput.channel(1), numSamples}};
        AudioBlock out{{r.output.channel(0), numSamples}, {r.output.channel(1), numSamples}};
        r.node->process(context, in, out);
    }

    for (const auto id : sinks_) if (const auto* sink = runtime(id)) externalOutput.addFrom(sink->output);
}

} // namespace guitardsp::graph
