#pragma once

#include "Graph.h"
#include "GraphDocument.h"
#include "NodeRegistry.h"
#include <string>
#include <unordered_map>

namespace guitardsp::graph {

struct BuildResult {
    bool ok = false;
    std::string message;
    std::unordered_map<NodeId, NodeId> documentToRuntimeId;
};

BuildResult buildGraphFromDocument(const GraphDocument& document, const NodeRegistry& registry, Graph& destination);
bool applyScene(const SceneDocument& scene, const std::unordered_map<NodeId, NodeId>& idMap, Graph& graph) noexcept;

} // namespace guitardsp::graph
