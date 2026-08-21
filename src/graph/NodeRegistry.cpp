#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/graph/UtilityNodes.h"
#include "guitardsp/graph/AdvancedRoutingNodes.h"
#include "guitardsp/dsp/BasicNodes.h"
#include "guitardsp/dsp/DriveNodes.h"
#include "guitardsp/dsp/KeyedGateNode.h"
#include "guitardsp/dsp/ConvolutionNode.h"

namespace guitardsp::graph {

NodeRegistry NodeRegistry::createBuiltins() {
    NodeRegistry r;
    r.registerType("utility.gain", [] { return std::make_unique<GainNode>(); });
    r.registerType("utility.polarity", [] { return std::make_unique<PolarityNode>(); });
    r.registerType("utility.pan", [] { return std::make_unique<PanNode>(); });
    r.registerType("route.split", [] { return std::make_unique<SplitNode>(); });
    r.registerType("route.merge", [] { return std::make_unique<MergeNode>(); });
    r.registerType("route.direct", [] { return std::make_unique<DirectOutNode>(); });
    r.registerType("route.crossover", [] { return std::make_unique<CrossoverSplitNode>(); });
    r.registerType("filter.hp", [] { return std::make_unique<dsp::OnePoleFilterNode>(dsp::OnePoleFilterNode::Mode::highPass, 80.0f); });
    r.registerType("filter.lp", [] { return std::make_unique<dsp::OnePoleFilterNode>(dsp::OnePoleFilterNode::Mode::lowPass, 8000.0f); });
    r.registerType("dynamics.compressor", [] { return std::make_unique<dsp::CompressorNode>(); });
    r.registerType("dynamics.transient", [] { return std::make_unique<dsp::TransientEnhancerNode>(); });
    r.registerType("dynamics.keyed_gate", [] { return std::make_unique<dsp::KeyedGateNode>(); });
    r.registerType("drive.ds1_prototype", [] { return std::make_unique<dsp::DS1PrototypeNode>(); });
    r.registerType("cab.fir", [] { return std::make_unique<dsp::ConvolutionNode>(); });
    return r;
}

} // namespace guitardsp::graph
