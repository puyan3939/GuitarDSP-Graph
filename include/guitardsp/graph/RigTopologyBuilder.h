#pragma once

#include "GraphDocument.h"

#include <optional>
#include <string>
#include <vector>

namespace guitardsp::graph {

// A single node in a routing chain, identified by its NodeRegistry typeId so
// callers never need to hand-manage NodeId numbering or connection wiring.
struct ChainNodeSpec {
    std::string typeId;
    std::vector<ParameterValueDocument> parameters{};
};

// One branch downstream of a splitter: which of the splitter's output ports it
// reads from, and an arbitrary ordered chain of nodes to run in series.
struct RoutingBranch {
    int sourcePort = 0;
    std::vector<ChainNodeSpec> chain;
};

// Describes an optionally-branched signal path: a serial chain, an optional
// splitter feeding one or more branches (each an arbitrary node chain), and an
// optional merge that recombines the branches into a single output. This lets
// callers assign any registered node type to any branch instead of being
// limited to a fixed guitar/bass split, while reusing the existing
// SplitNode/MergeNode/BranchLevelNode implementations unchanged.
struct RoutingTopology {
    std::vector<ChainNodeSpec> chain;
    std::optional<ChainNodeSpec> splitter;
    std::vector<RoutingBranch> branches;
    std::optional<ChainNodeSpec> merge;
};

// Builds a GraphDocument (nodes + connections) from a RoutingTopology. The
// result is ready to hand to buildGraphFromDocument()/GraphBuilder. Returns a
// document containing a single passthrough gain node if the topology would
// otherwise be empty, so the result always compiles to a valid graph.
[[nodiscard]] GraphDocument buildTopologyDocument(const RoutingTopology& topology);

} // namespace guitardsp::graph
