#include "guitardsp/graph/Graph.h"

#include <algorithm>
#include <deque>

namespace guitardsp::graph {

NodeId Graph::addNode(std::unique_ptr<AudioNode> newNode) {
    if (!newNode) return 0;
    const NodeId id = nextId_++;
    nodes_.emplace(id, std::move(newNode));
    executionOrder_.clear(); cumulativeLatencies_.clear(); maxGraphLatency_ = 0;
    return id;
}

bool Graph::removeNode(NodeId id) {
    if (nodes_.erase(id) == 0) return false;
    edges_.erase(std::remove_if(edges_.begin(), edges_.end(), [id](const Connection& e) {
        return e.from == id || e.to == id;
    }), edges_.end());
    executionOrder_.clear(); cumulativeLatencies_.clear(); maxGraphLatency_ = 0;
    return true;
}

bool Graph::connect(NodeId from, NodeId to) {
    if (from == 0 || to == 0 || from == to || !nodes_.contains(from) || !nodes_.contains(to)) return false;
    const auto duplicate = std::find_if(edges_.begin(), edges_.end(), [from, to](const Connection& e) {
        return e.from == from && e.to == to;
    });
    if (duplicate != edges_.end()) return false;
    edges_.push_back({from, to});
    executionOrder_.clear(); cumulativeLatencies_.clear(); maxGraphLatency_ = 0;
    return true;
}

bool Graph::disconnect(NodeId from, NodeId to) {
    const auto oldSize = edges_.size();
    edges_.erase(std::remove_if(edges_.begin(), edges_.end(), [from, to](const Connection& e) {
        return e.from == from && e.to == to;
    }), edges_.end());
    if (edges_.size() == oldSize) return false;
    executionOrder_.clear(); cumulativeLatencies_.clear(); maxGraphLatency_ = 0;
    return true;
}

void Graph::clear() {
    nodes_.clear(); edges_.clear(); executionOrder_.clear(); cumulativeLatencies_.clear();
    maxGraphLatency_ = 0; nextId_ = 1;
}

AudioNode* Graph::node(NodeId id) noexcept {
    const auto it = nodes_.find(id);
    return it == nodes_.end() ? nullptr : it->second.get();
}

const AudioNode* Graph::node(NodeId id) const noexcept {
    const auto it = nodes_.find(id);
    return it == nodes_.end() ? nullptr : it->second.get();
}

ValidationResult Graph::compile() {
    executionOrder_.clear(); cumulativeLatencies_.clear(); maxGraphLatency_ = 0;
    if (nodes_.empty()) return {false, "Graph contains no nodes"};

    std::unordered_map<NodeId, int> indegree;
    std::unordered_map<NodeId, std::vector<NodeId>> outgoing;
    std::unordered_map<NodeId, std::vector<NodeId>> incoming;
    indegree.reserve(nodes_.size()); outgoing.reserve(nodes_.size()); incoming.reserve(nodes_.size());
    for (const auto& [id, nodePtr] : nodes_) {
        (void)nodePtr;
        indegree[id] = 0;
    }
    for (const auto& edge : edges_) {
        if (!nodes_.contains(edge.from) || !nodes_.contains(edge.to))
            return {false, "Graph contains a connection to a missing node"};
        ++indegree[edge.to];
        outgoing[edge.from].push_back(edge.to);
        incoming[edge.to].push_back(edge.from);
    }

    std::vector<NodeId> roots;
    roots.reserve(nodes_.size());
    for (const auto& [id, degree] : indegree) if (degree == 0) roots.push_back(id);
    std::sort(roots.begin(), roots.end());
    std::deque<NodeId> ready(roots.begin(), roots.end());

    while (!ready.empty()) {
        const NodeId id = ready.front(); ready.pop_front();
        executionOrder_.push_back(id);
        auto destinations = outgoing[id];
        std::sort(destinations.begin(), destinations.end());
        for (const NodeId destination : destinations) {
            auto& degree = indegree[destination];
            if (--degree == 0) ready.push_back(destination);
        }
    }

    if (executionOrder_.size() != nodes_.size()) {
        executionOrder_.clear();
        return {false, "Graph contains a cycle"};
    }

    for (const NodeId id : executionOrder_) {
        int upstreamMaximum = 0;
        if (const auto parentIt = incoming.find(id); parentIt != incoming.end()) {
            for (const NodeId parent : parentIt->second)
                if (const auto latencyIt = cumulativeLatencies_.find(parent); latencyIt != cumulativeLatencies_.end())
                    upstreamMaximum = std::max(upstreamMaximum, latencyIt->second);
        }
        const auto* current = node(id);
        const int own = current != nullptr ? std::max(0, current->latencySamples()) : 0;
        const int total = upstreamMaximum + own;
        cumulativeLatencies_[id] = total;
        maxGraphLatency_ = std::max(maxGraphLatency_, total);
    }
    return {true, {}};
}

std::optional<int> Graph::cumulativeLatencySamples(NodeId id) const {
    const auto it = cumulativeLatencies_.find(id);
    if (it == cumulativeLatencies_.end()) return std::nullopt;
    return it->second;
}

} // namespace guitardsp::graph
