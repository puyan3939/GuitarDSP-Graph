#pragma once

#include "AudioNode.h"
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace guitardsp::graph
{
struct Connection
{
    NodeId from = 0;
    NodeId to = 0;
};

struct ValidationResult
{
    bool ok = false;
    std::string message;
};

class Graph
{
public:
    NodeId addNode(std::unique_ptr<AudioNode> node);
    bool removeNode(NodeId id);
    bool connect(NodeId from, NodeId to);
    bool disconnect(NodeId from, NodeId to);
    void clear();

    [[nodiscard]] AudioNode* node(NodeId id) noexcept;
    [[nodiscard]] const AudioNode* node(NodeId id) const noexcept;
    [[nodiscard]] const std::vector<Connection>& connections() const noexcept { return edges; }
    [[nodiscard]] const std::vector<NodeId>& schedule() const noexcept { return executionOrder; }

    ValidationResult compile();

    [[nodiscard]] std::optional<std::size_t> cumulativeLatencySamples(NodeId id) const;
    [[nodiscard]] std::size_t maximumGraphLatencySamples() const noexcept { return maxGraphLatency; }

private:
    NodeId nextId = 1;
    std::unordered_map<NodeId, std::unique_ptr<AudioNode>> nodes;
    std::vector<Connection> edges;
    std::vector<NodeId> executionOrder;
    std::unordered_map<NodeId, std::size_t> cumulativeLatencies;
    std::size_t maxGraphLatency = 0;
};
}
