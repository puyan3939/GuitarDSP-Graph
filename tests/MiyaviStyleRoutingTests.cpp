// Integration test for issue #23: build a MIYAVI-style rig purely with the
// generic routing API added for issue #18 (RoutingTopology / GraphDocument /
// NodeRegistry) instead of the fixed LiveRigSettings enum, and verify it
// actually works end to end:
//   input -> Split -> [drive + amp + cabinet]        (ordinary guitar chain)
//                   -> [octave down + bass amp + cab] (bass-amp branch)
//         -> Merge -> output
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/graph/GraphBuilder.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/graph/RealtimeGraphHost.h"
#include "guitardsp/graph/RigTopologyBuilder.h"
#include "guitardsp/hq/BassAmpNode.h"
#include "guitardsp/hq/CabinetChainNode.h"
#include "guitardsp/hq/Measurement.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

using namespace guitardsp;

namespace {

bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

bool graphContains(const graph::Graph& g, std::string_view typeName) {
    for (const auto id : g.schedule())
        if (const auto* node = g.node(id); node != nullptr && node->typeName() == typeName)
            return true;
    return false;
}

graph::GraphDocument buildMiyaviStyleDocument(float guitarBranchLevel, float bassBranchLevel) {
    graph::RoutingTopology topology;
    topology.splitter = graph::ChainNodeSpec{"route.split", {}};

    graph::RoutingBranch guitarBranch;
    guitarBranch.sourcePort = 0;
    guitarBranch.chain = {
        {"drive.ts808_circuit_hq", {}},
        {"amp.reference_hq", {{"gain", 0.6f}}},
        {"cab.chain_hq", {}},
        {"route.guitar_level", {{"level", guitarBranchLevel}}},
    };

    graph::RoutingBranch bassBranch;
    bassBranch.sourcePort = 0;
    bassBranch.chain = {
        {"pitch.octave_down_mono", {}},
        {"amp.bass_reference_hq", {}},
        {"cab.bass_reference_hq", {}},
        {"route.bass_level", {{"level", bassBranchLevel}}},
    };

    topology.branches = {guitarBranch, bassBranch};
    topology.merge = graph::ChainNodeSpec{"route.merge", {}};
    return graph::buildTopologyDocument(topology);
}

// Cabinet impulse responses are runtime data, not a scalar parameter, so
// GraphDocument/ParameterValueDocument cannot express them; the built runtime
// node has to be located and downcast after buildGraphFromDocument()/prepare().
void installReferenceCabinets(const graph::GraphDocument& document, graph::PreparedGraph& prepared,
                              double sampleRate) {
    for (const auto& nodeDoc : document.nodes) {
        const graph::NodeId runtimeId = prepared.documentToRuntimeId.at(nodeDoc.id);
        auto* node = prepared.graph.node(runtimeId);
        if (nodeDoc.typeId == "cab.chain_hq") {
            static_cast<hq::CabinetChainNode*>(node)->setImpulseResponse(
                app::makeReferenceCabinetImpulse(sampleRate));
        } else if (nodeDoc.typeId == "cab.bass_reference_hq") {
            static_cast<hq::BassCabinetNode*>(node)->setImpulseResponse(
                app::makeReferenceBassCabinetImpulse(sampleRate));
        }
    }
}

std::unique_ptr<graph::PreparedGraph> prepareMiyaviRig(const graph::GraphDocument& document,
                                                        const graph::NodeRegistry& registry,
                                                        double sampleRate, int blockSize) {
    auto prepared = graph::RealtimeGraphHost{}.prepare(document, registry, sampleRate, blockSize, 1,
                                                        graph::ProcessingQuality::eco);
    if (prepared) {
        installReferenceCabinets(document, *prepared, sampleRate);
        prepared->runtime.reset();
    }
    return prepared;
}

} // namespace

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;
    constexpr int blockSize = 256;
    constexpr int blocks = 64; // 16384 samples total
    auto registry = graph::NodeRegistry::createBuiltins();

    {
        const auto document = buildMiyaviStyleDocument(0.8f, 1.0f);
        ok &= require(document.nodes.size() == 10,
                      "split + 4-node guitar chain + 4-node bass chain + merge is 10 document nodes");

        auto prepared = prepareMiyaviRig(document, registry, sampleRate, blockSize);
        ok &= require(prepared != nullptr, "MIYAVI-style split/merge topology compiles to a runnable graph");
        if (prepared) {
            ok &= require(graphContains(prepared->graph, "Split") && graphContains(prepared->graph, "Merge"),
                          "graph contains the splitter and merge nodes");
            ok &= require(graphContains(prepared->graph, "TS808 Circuit")
                              && graphContains(prepared->graph, "Reference Amp Topology")
                              && graphContains(prepared->graph, "Speaker Dynamics + Partitioned Cab"),
                          "guitar branch keeps the existing drive + amp + cabinet chain");
            ok &= require(graphContains(prepared->graph, "Monophonic Octave Down")
                              && graphContains(prepared->graph, "Bass Amp Reference")
                              && graphContains(prepared->graph, "Bass Cabinet Reference"),
                          "bass branch runs the octave divider into a dedicated bass amp/cab chain");
            ok &= require(prepared->runtime.totalLatencySamples() >= 64,
                          "differing cabinet latencies across branches are compensated automatically");
        }
    }

    // Isolate the bass branch to prove the octave-down signal actually reaches
    // the merged output, not just that the graph compiles.
    {
        const auto document = buildMiyaviStyleDocument(0.0f, 1.0f);
        auto prepared = prepareMiyaviRig(document, registry, sampleRate, blockSize);
        ok &= require(prepared != nullptr, "bass-only mix prepares");
        if (prepared) {
            graph::AudioBuffer input(1, blockSize), output(1, blockSize);
            std::vector<float> captured;
            captured.reserve(static_cast<std::size_t>(blockSize * blocks));
            for (int block = 0; block < blocks; ++block) {
                for (int i = 0; i < blockSize; ++i) {
                    const int sample = block * blockSize + i;
                    input.channel(0)[i] = 0.2f * static_cast<float>(std::sin(
                        2.0 * std::numbers::pi * 220.0 * static_cast<double>(sample) / sampleRate));
                }
                prepared->runtime.process(input, output, blockSize);
                for (int i = 0; i < blockSize; ++i) captured.push_back(output.channel(0)[i]);
            }
            const auto tail = std::span<const float>(captured).subspan(captured.size() / 2U);
            const float sub = hq::singleBinMagnitude(tail, sampleRate, 110.0);
            const float source = hq::singleBinMagnitude(tail, sampleRate, 220.0);
            std::cout << "DIAG miyavi-bass-only sub110=" << sub << " source220=" << source << '\n';
            ok &= require(sub > 0.001f && sub > 2.0f * source,
                          "the merged output carries the octave-down bass branch, not just the split guitar signal");
        }
    }

    // Isolate the guitar branch: finite, non-trivial energy confirms the
    // existing drive/amp/cab chain still renders correctly as one arm of a
    // user-composed split, not just as the sole chain inside prepareLiveRig().
    {
        const auto document = buildMiyaviStyleDocument(1.0f, 0.0f);
        auto prepared = prepareMiyaviRig(document, registry, sampleRate, blockSize);
        ok &= require(prepared != nullptr, "guitar-only mix prepares");
        if (prepared) {
            graph::AudioBuffer input(1, blockSize), output(1, blockSize);
            bool finite = true;
            double energy = 0.0;
            for (int block = 0; block < blocks; ++block) {
                for (int i = 0; i < blockSize; ++i) {
                    const int sample = block * blockSize + i;
                    input.channel(0)[i] = 0.2f * static_cast<float>(std::sin(
                        2.0 * std::numbers::pi * 220.0 * static_cast<double>(sample) / sampleRate));
                }
                prepared->runtime.process(input, output, blockSize);
                for (int i = 0; i < blockSize; ++i) {
                    finite &= std::isfinite(output.channel(0)[i]);
                    energy += static_cast<double>(output.channel(0)[i]) * output.channel(0)[i];
                }
            }
            ok &= require(finite && energy > 1.0e-6,
                          "the split guitar drive/amp/cab branch still renders finite, audible output");
        }
    }

    return ok ? 0 : 1;
}
