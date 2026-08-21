#include "guitardsp/graph/GraphBuilder.h"

namespace guitardsp::graph {

BuildResult buildGraphFromDocument(const GraphDocument& document, const NodeRegistry& registry, Graph& destination) {
    destination.clear();
    BuildResult result;
    result.documentToRuntimeId.reserve(document.nodes.size());

    for (const auto& nodeDoc : document.nodes) {
        if (nodeDoc.id == 0) return {false, "Document contains node id 0", {}};
        if (result.documentToRuntimeId.contains(nodeDoc.id)) return {false, "Document contains duplicate node id", {}};
        auto node = registry.create(nodeDoc.typeId);
        if (!node) return {false, "Unknown node type: " + nodeDoc.typeId, {}};
        node->setBypassed(nodeDoc.bypassed); node->setMuted(nodeDoc.muted);
        for (const auto& value : nodeDoc.parameters) {
            const int index = node->parameterIndex(value.id);
            if (index >= 0) node->setParameterValue(static_cast<std::size_t>(index), value.value);
        }
        const NodeId runtimeId = destination.addNode(std::move(node));
        result.documentToRuntimeId.emplace(nodeDoc.id, runtimeId);
    }

    for (const auto& edge : document.connections) {
        const auto from = result.documentToRuntimeId.find(edge.from);
        const auto to = result.documentToRuntimeId.find(edge.to);
        if (from == result.documentToRuntimeId.end() || to == result.documentToRuntimeId.end())
            return {false, "Connection references missing node", {}};
        if (!destination.connect(from->second, to->second))
            return {false, "Invalid or duplicate connection", {}};
    }

    const auto validation = destination.compile();
    if (!validation.ok) return {false, validation.message, {}};
    result.ok = true;
    return result;
}

bool applyScene(const SceneDocument& scene, const std::unordered_map<NodeId, NodeId>& idMap, Graph& graph) noexcept {
    bool changed = false;
    for (const auto& state : scene.nodes) {
        const auto mapping = idMap.find(state.id);
        if (mapping == idMap.end()) continue;
        auto* node = graph.node(mapping->second);
        if (!node) continue;
        node->setBypassed(state.bypassed); node->setMuted(state.muted);
        for (const auto& value : state.parameters) {
            const int index = node->parameterIndex(value.id);
            if (index >= 0) changed |= node->setParameterValue(static_cast<std::size_t>(index), value.value);
        }
        changed = true;
    }
    return changed;
}

} // namespace guitardsp::graph
