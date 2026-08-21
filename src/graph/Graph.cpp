#include "guitardsp/graph/Graph.h"

#include <algorithm>
#include <deque>
#include <unordered_set>

namespace guitardsp::graph
{
NodeId Graph::addNode(std::unique_ptr<AudioNode> newNode)
{
    if (!newNode)
        return 0;

    const auto id = nextId++;
    nodes.emplace(id, std::move(newNode));
    executionOrder.clear();
    cumulativeLatencies.clear();
    maxGraphLatency = 0;
    return id;
}

bool Graph::removeNode(NodeId id)
{
    const auto erased = nodes.erase(id) > 0;
    if (!erased)
        return false;

    edges.erase(std::remove_if(edges.begin(), edges.end(), [id](const Connection& edge)
    {
        return edge.from == id || edge.to == id;
    }), edges.end());

    executionOrder.clear();
    cumulativeLatencies.clear();
    maxGraphLatency = 0;
    return true;
}

bool Graph::connect(NodeId from, NodeId to)
{
    if (from == 0 || to == 0 || from == to || !nodes.contains(from) || !nodes.contains(to))
        return false;

    const auto duplicate = std::find_if(edges.begin(), edges.end(), [from, to](const Connection& edge)
    {
        return edge.from == from && edge.to == to;
    });

    if (duplicate != edges.end())
        return false;

    edges.push_back({from, to});
    executionOrder.clear();
    cumulativeLatencies.clear();
    maxGraphLatency = 0;
    return true;
}

bool Graph::disconnect(NodeId from, NodeId to)
{
    const auto oldSize = edges.size();
    edges.erase(std::remove_if(edges.begin(), edges.end(), [from, to](const Connection& edge)
    {
        return edge.from == from && edge.to == to;
    }), edges.end());

    const bool changed = edges.size() != oldSize;
    if (changed)
    {
        executionOrder.clear();
        cumulativeLatencies.clear();
        maxGraphLatency = 0;
    }
    return changed;
}

void Graph::clear()
{
    nodes.clear();
    edges.clear();
    executionOrder.clear();
    cumulativeLatencies.clear();
    maxGraphLatency = 0;
    nextId = 1;
}

AudioNode* Graph::node(NodeId id) noexcept
{
    const auto it = nodes.find(id);
    return it == nodes.end() ? nullptr : it->second.get();
}

const AudioNode* Graph::node(NodeId id) const noexcept
{
    const auto it = nodes.find(id);
    return it == nodes.end() ? nullptr : it->second.get();
}

ValidationResult Graph::compile()
{
    executionOrder.clear();
    cumulativeLatencies.clear();
    maxGraphLatency = 0;

    std::unordered_map<NodeId, std::size_t> indegree;
    std::unordered_map<NodeId, std::vector<NodeId>> outgoing;
    std::unordered_map<NodeId, std::vector<NodeId>> incoming;

    indegree.reserve(nodes.size());
    outgoing.reserve(nodes.size());
    incoming.reserve(nodes.size());

    for (const auto& [id, _] : nodes)
        indegree[id] = 0;

    for (const auto& edge : edges)
    {
        if (!nodes.contains(edge.from) || !nodes.contains(edge.to))
            return {false, "Graph contains a connection to a missing node"};

        ++indegree[edge.to];
        outgoing[edge.from].push_back(edge.to);
        incoming[edge.to].push_back(edge.from);
    }

    std::deque<NodeId> ready;
    for (const auto& [id, degree] : indegree)
        if (degree == 0)
            ready.push_back(id);

    while (!ready.empty())
    {
        const NodeId id = ready.front();
        ready.pop_front();
        executionOrder.push_back(id);

        for (const NodeId destination : outgoing[id])
        {
            auto& degree = indegree[destination];
            if (--degree == 0)
                ready.push_back(destination);
        }
    }

    if (executionOrder.size() != nodes.size())
    {
        executionOrder.clear();
        return {false, "Graph contains a cycle"};
    }

    for (const NodeId id : executionOrder)
    {
        std::size_t upstreamMaximum = 0;
        const auto parentIt = incoming.find(id);
        if (parentIt != incoming.end())
        {
            for (const NodeId parent : parentIt->second)
            {
                const auto latencyIt = cumulativeLatencies.find(parent);
                if (latencyIt != cumulativeLatencies.end())
                    upstreamMaximum = std::max(upstreamMaximum, latencyIt->second);
            }
        }

        const auto* current = node(id);
        const auto total = upstreamMaximum + (current != nullptr ? current->latencySamples() : 0);
        cumulativeLatencies[id] = total;
        maxGraphLatency = std::max(maxGraphLatency, total);
    }

    return {true, {}};
}

std::optional<std::size_t> Graph::cumulativeLatencySamples(NodeId id) const
{
    const auto it = cumulativeLatencies.find(id);
    if (it == cumulativeLatencies.end())
        return std::nullopt;
    return it->second;
}
}
