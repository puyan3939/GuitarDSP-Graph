#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/graph/UtilityNodes.h"
#include "guitardsp/dsp/BasicNodes.h"
#include "guitardsp/dsp/DriveNodes.h"
#include "guitardsp/dsp/KeyedGateNode.h"
#include "guitardsp/dsp/ConvolutionNode.h"
#include "guitardsp/graph/AdvancedRoutingNodes.h"
#include "guitardsp/graph/IONodes.h"
#include "guitardsp/hq/AmpFamilyNodes.h"
#include "guitardsp/hq/BD2TopologyNode.h"
#include "guitardsp/hq/CabinetChainNode.h"
#include "guitardsp/hq/DS1CircuitNode.h"
#include "guitardsp/hq/DS1TopologyNode.h"
#include "guitardsp/hq/PartitionedCabNode.h"
#include "guitardsp/hq/ReferenceAmpTopologyNode.h"
#include "guitardsp/hq/SpeakerDynamicsNode.h"
#include "guitardsp/hq/TS808CircuitNode.h"
#include "guitardsp/hq/TS808TopologyNode.h"
#include "guitardsp/hq/TwoTransistorFuzzNode.h"

namespace guitardsp::graph {

NodeRegistry NodeRegistry::createBuiltins() {
    NodeRegistry r;
    r.registerType("utility.gain", [] { return std::make_unique<GainNode>(); });
    r.registerType("utility.polarity", [] { return std::make_unique<PolarityNode>(); });
    r.registerType("utility.pan", [] { return std::make_unique<PanNode>(); });
    r.registerType("route.split", [] { return std::make_unique<SplitNode>(); });
    r.registerType("route.merge", [] { return std::make_unique<MergeNode>(); });
    r.registerType("route.direct", [] { return std::make_unique<DirectOutNode>(); });
    r.registerType("filter.hp", [] { return std::make_unique<dsp::OnePoleFilterNode>(dsp::OnePoleFilterNode::Mode::highPass, 80.0f); });
    r.registerType("filter.lp", [] { return std::make_unique<dsp::OnePoleFilterNode>(dsp::OnePoleFilterNode::Mode::lowPass, 8000.0f); });
    r.registerType("dynamics.compressor", [] { return std::make_unique<dsp::CompressorNode>(); });
    r.registerType("dynamics.transient", [] { return std::make_unique<dsp::TransientEnhancerNode>(); });
    r.registerType("drive.ds1_prototype", [] { return std::make_unique<dsp::DS1PrototypeNode>(); });
    r.registerType("drive.ds1_hq", [] { return std::make_unique<hq::DS1TopologyNode>(); });
    r.registerType("drive.ds1_circuit_hq", [] { return std::make_unique<hq::DS1CircuitNode>(); });
    r.registerType("drive.ts808_hq", [] { return std::make_unique<hq::TS808TopologyNode>(); });
    r.registerType("drive.ts808_circuit_hq", [] { return std::make_unique<hq::TS808CircuitNode>(); });
    r.registerType("drive.bd2_hq", [] { return std::make_unique<hq::BD2TopologyNode>(); });
    r.registerType("drive.fuzz_two_transistor", [] { return std::make_unique<hq::TwoTransistorFuzzNode>(); });
    r.registerType("amp.reference_hq", [] { return std::make_unique<hq::ReferenceAmpTopologyNode>(); });
    r.registerType("amp.british_plexi_family_hq", [] { return std::make_unique<hq::BritishPlexiFamilyNode>(); });
    r.registerType("amp.american_clean_family_hq", [] { return std::make_unique<hq::AmericanCleanFamilyNode>(); });
    r.registerType("dynamics.keyed_gate", [] { return std::make_unique<dsp::KeyedGateNode>(); });
    r.registerType("cab.fir", [] { return std::make_unique<dsp::ConvolutionNode>(); });
    r.registerType("cab.partitioned_hq", [] { return std::make_unique<hq::PartitionedCabNode>(); });
    r.registerType("cab.speaker_dynamics_hq", [] { return std::make_unique<hq::SpeakerDynamicsNode>(); });
    r.registerType("cab.chain_hq", [] { return std::make_unique<hq::CabinetChainNode>(); });
    r.registerType("route.crossover", [] { return std::make_unique<CrossoverSplitNode>(); });
    r.registerType("io.output", [] { return std::make_unique<OutputBusNode>(); });
    return r;
}

} // namespace guitardsp::graph
