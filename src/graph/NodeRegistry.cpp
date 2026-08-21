#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/graph/UtilityNodes.h"
#include "guitardsp/dsp/BasicNodes.h"
#include "guitardsp/dsp/DriveNodes.h"

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
    return r;
}

} // namespace guitardsp::graph
