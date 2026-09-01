#include "guitardsp/graph/RigTopologyBuilder.h"

#include <utility>

namespace guitardsp::graph {
namespace {

NodeId appendNode(GraphDocument& document, NodeId& nextId, const ChainNodeSpec& spec) {
    const NodeId id = nextId++;
    NodeDocument node;
    node.id = id;
    node.typeId = spec.typeId;
    node.parameters = spec.parameters;
    document.nodes.push_back(std::move(node));
    return id;
}

// Appends a serial chain after (previous, previousPort). previous == 0 means the
// chain's first node has no upstream connection (it becomes a graph root).
void appendChain(GraphDocument& document, NodeId& nextId, NodeId& previous, int& previousPort,
                 const std::vector<ChainNodeSpec>& chain) {
    for (const auto& spec : chain) {
        const NodeId id = appendNode(document, nextId, spec);
        if (previous != 0) document.connections.push_back({previous, previousPort, id, 0});
        previous = id;
        previousPort = 0;
    }
}

} // namespace

GraphDocument buildTopologyDocument(const RoutingTopology& topology) {
    GraphDocument document;
    NodeId nextId = 1;

    NodeId head = 0;
    int headPort = 0;
    appendChain(document, nextId, head, headPort, topology.chain);

    NodeId splitter = 0;
    if (topology.splitter) {
        splitter = appendNode(document, nextId, *topology.splitter);
        if (head != 0) document.connections.push_back({head, headPort, splitter, 0});
    }

    std::vector<std::pair<NodeId, int>> branchOutputs;
    branchOutputs.reserve(topology.branches.size());
    for (const auto& branch : topology.branches) {
        NodeId branchHead = splitter;
        int branchPort = branch.sourcePort;
        appendChain(document, nextId, branchHead, branchPort, branch.chain);
        if (branchHead != 0) branchOutputs.emplace_back(branchHead, branchPort);
    }

    if (topology.merge && !branchOutputs.empty()) {
        const NodeId merge = appendNode(document, nextId, *topology.merge);
        for (const auto& [outputId, outputPort] : branchOutputs)
            document.connections.push_back({outputId, outputPort, merge, 0});
    }
    // With no merge, each branch tail keeps its unconnected output port; the
    // compiled audio graph automatically sums unconnected output ports onto the
    // output bus, so multiple branches still mix together without one.

    // A splitter with no branches, or a topology with nothing at all, still needs
    // a valid root/sink so the result always compiles to a runnable graph.
    if (document.nodes.empty())
        appendNode(document, nextId, ChainNodeSpec{"utility.gain", {}});

    return document;
}

} // namespace guitardsp::graph
