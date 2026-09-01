#include "guitardsp/graph/GraphBuilder.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/graph/RealtimeGraphHost.h"
#include "guitardsp/graph/RigTopologyBuilder.h"

#include <cmath>
#include <iostream>
#include <numbers>
#include <string_view>

using namespace guitardsp::graph;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

bool graphContains(const Graph& graph, std::string_view typeName) {
    for (const auto id : graph.schedule())
        if (const auto* node = graph.node(id); node != nullptr && node->typeName() == typeName)
            return true;
    return false;
}
} // namespace

int main() {
    bool ok = true;
    auto registry = NodeRegistry::createBuiltins();

    // Arbitrary branch assignment: a MIYAVI-style rig routes an octave divider's
    // output directly into a bass amp chain, while the drive pedal sits on the
    // *other* branch. The fixed SignalRouting enum in prepareLiveRig() could
    // never express this (it always pins pedal/amp/cab to the guitar branch and
    // octave/bass-amp to the bass branch); RoutingTopology assigns node chains to
    // branches freely.
    {
        RoutingTopology topology;
        topology.splitter = ChainNodeSpec{"route.split", {}};

        RoutingBranch pedalBranch;
        pedalBranch.sourcePort = 0;
        pedalBranch.chain = {{"drive.ts808_circuit_hq", {}}, {"route.guitar_level", {}}};

        RoutingBranch octaveIntoBassAmpBranch;
        octaveIntoBassAmpBranch.sourcePort = 0;
        octaveIntoBassAmpBranch.chain = {
            {"pitch.octave_down_mono", {}},
            {"amp.bass_reference_hq", {}},
            {"route.bass_level", {}},
        };

        topology.branches = {pedalBranch, octaveIntoBassAmpBranch};
        topology.merge = ChainNodeSpec{"route.merge", {}};

        const auto document = buildTopologyDocument(topology);
        ok &= require(document.nodes.size() == 7,
                      "custom topology emits one document node per splitter/chain/merge entry");

        Graph graph;
        const auto build = buildGraphFromDocument(document, registry, graph);
        ok &= require(build.ok, "GraphBuilder accepts an arbitrary branch/chain topology");
        ok &= require(graphContains(graph, "Split") && graphContains(graph, "Merge")
                          && graphContains(graph, "Monophonic Octave Down")
                          && graphContains(graph, "Bass Amp Reference"),
                      "octave divider and bass amp both appear on a branch that is not the fixed guitar/bass split");

        NodeId octaveId = 0, bassAmpId = 0;
        for (const auto& nodeDoc : document.nodes) {
            if (nodeDoc.typeId == "pitch.octave_down_mono") octaveId = build.documentToRuntimeId.at(nodeDoc.id);
            if (nodeDoc.typeId == "amp.bass_reference_hq") bassAmpId = build.documentToRuntimeId.at(nodeDoc.id);
        }
        bool octaveFeedsBassAmpDirectly = false;
        for (const auto& edge : graph.connections())
            if (edge.from == octaveId && edge.to == bassAmpId) octaveFeedsBassAmpDirectly = true;
        ok &= require(octaveFeedsBassAmpDirectly,
                      "octave node connects directly into the bass amp node within its branch chain");

        auto prepared = RealtimeGraphHost{}.prepare(document, registry, 48000.0, 64, 1);
        ok &= require(prepared != nullptr, "custom MIYAVI-style topology compiles into a runnable prepared graph");
        if (prepared) {
            AudioBuffer input(1, 64), output(1, 64);
            bool finite = true;
            float energy = 0.0f;
            for (int block = 0; block < 4; ++block) {
                for (int i = 0; i < 64; ++i) {
                    const int sample = block * 64 + i;
                    input.channel(0)[i] = 0.15f * std::sin(
                        2.0f * std::numbers::pi_v<float> * 220.0f
                        * static_cast<float>(sample) / 48000.0f);
                }
                prepared->runtime.process(input, output, 64);
                for (int i = 0; i < 64; ++i) {
                    finite &= std::isfinite(output.channel(0)[i]);
                    energy += output.channel(0)[i] * output.channel(0)[i];
                }
            }
            ok &= require(finite && energy > 1.0e-8f,
                          "custom MIYAVI-style routing produces finite, audible output");
        }
    }

    // Three independent branches with no explicit merge node still sum onto the
    // output bus automatically, so RoutingTopology is not limited to the
    // built-in two-branch guitar/bass pattern.
    {
        RoutingTopology topology;
        topology.splitter = ChainNodeSpec{"route.split", {}};
        topology.branches.push_back({0, {{"utility.gain", {{"gain", 0.5f}}}}});
        topology.branches.push_back({0, {{"utility.gain", {{"gain", 0.25f}}}}});
        topology.branches.push_back({0, {{"utility.gain", {{"gain", 0.1f}}}}});

        const auto document = buildTopologyDocument(topology);
        auto prepared = RealtimeGraphHost{}.prepare(document, registry, 48000.0, 32, 1);
        ok &= require(prepared != nullptr, "three-branch topology without an explicit merge node prepares");
        if (prepared) {
            AudioBuffer input(1, 32), output(1, 32);
            input.clear();
            for (int i = 0; i < 32; ++i) input.channel(0)[i] = 1.0f;
            prepared->runtime.process(input, output, 32);
            ok &= require(std::abs(output.channel(0)[0] - 0.85f) < 1.0e-5f,
                          "unconnected branch outputs auto-sum onto the output bus (0.5 + 0.25 + 0.1)");
        }
    }

    // An empty topology still yields a valid, compilable passthrough graph
    // instead of an invalid empty document.
    {
        const auto document = buildTopologyDocument(RoutingTopology{});
        ok &= require(document.nodes.size() == 1 && document.nodes.front().typeId == "utility.gain",
                      "an empty topology falls back to a single passthrough node");
        Graph graph;
        const auto build = buildGraphFromDocument(document, registry, graph);
        ok &= require(build.ok, "fallback passthrough document compiles");
    }

    // Unknown node types anywhere in a custom chain are rejected by GraphBuilder,
    // not silently dropped, keeping generalized routing as safe as the fixed rig.
    {
        RoutingTopology topology;
        topology.chain = {{"does.not.exist", {}}};
        const auto document = buildTopologyDocument(topology);
        Graph graph;
        const auto build = buildGraphFromDocument(document, registry, graph);
        ok &= require(!build.ok, "unknown node type in a custom chain is rejected during document build");
    }

    return ok ? 0 : 1;
}
