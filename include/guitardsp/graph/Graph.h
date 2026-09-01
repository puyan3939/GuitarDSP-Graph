#pragma once

#include "AudioNode.h"
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace guitardsp::graph {

struct Connection {
    NodeId from = 0;
    int fromPort = 0;
    NodeId to = 0;
    int toPort = 0;
};

struct ValidationResult { bool ok = false; std::string message; };

class Graph {
public:
    NodeId addNode(std::unique_ptr<AudioNode> node);
    bool removeNode(NodeId id);
    bool connect(NodeId from, NodeId to) { return connect(from, 0, to, 0); }
    bool connect(NodeId from, int fromPort, NodeId to, int toPort);
    bool disconnect(NodeId from, NodeId to) { return disconnect(from, 0, to, 0); }
    bool disconnect(NodeId from, int fromPort, NodeId to, int toPort);
    void clear();

    [[nodiscard]] AudioNode* node(NodeId id) noexcept;
    [[nodiscard]] const AudioNode* node(NodeId id) const noexcept;
    [[nodiscard]] const std::vector<Connection>& connections() const noexcept { return edges_; }
    [[nodiscard]] const std::vector<NodeId>& schedule() const noexcept { return executionOrder_; }
    ValidationResult compile();
    [[nodiscard]] std::optional<int> cumulativeLatencySamples(NodeId id) const;
    [[nodiscard]] int maximumGraphLatencySamples() const noexcept { return maxGraphLatency_; }

private:
    NodeId nextId_ = 1;
    std::unordered_map<NodeId, std::unique_ptr<AudioNode>> nodes_;
    std::vector<Connection> edges_;
    std::vector<NodeId> executionOrder_;
    std::unordered_map<NodeId, int> cumulativeLatencies_;
    int maxGraphLatency_ = 0;
};

} // namespace guitardsp::graph
