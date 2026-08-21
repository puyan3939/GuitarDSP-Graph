#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/graph/UtilityNodes.h"
#include "guitardsp/hq/AmpFamilyNodes.h"
#include "guitardsp/hq/CabinetChainNode.h"
#include "guitardsp/hq/DS1CircuitNode.h"
#include "guitardsp/hq/ReferenceAmpTopologyNode.h"
#include "guitardsp/hq/TS808CircuitNode.h"

#include <algorithm>
#include <memory>
#include <utility>

namespace guitardsp::app {
namespace {

void setPedalParameters(graph::AudioNode& node, const LiveRigSettings& settings) {
    node.setParameterValue(0, settings.pedalDrive);
    node.setParameterValue(1, settings.pedalTone);
    node.setParameterValue(2, settings.pedalLevel);
}

void setAmpParameters(graph::AudioNode& node, const LiveRigSettings& settings) {
    node.setParameterValue(0, settings.ampGain);
    node.setParameterValue(1, settings.ampBass);
    node.setParameterValue(2, settings.ampMid);
    node.setParameterValue(3, settings.ampTreble);
    node.setParameterValue(4, settings.ampMaster);
    node.setParameterValue(5, settings.ampPresence);
    node.setParameterValue(6, settings.ampOutputDb);
}

} // namespace

std::unique_ptr<graph::PreparedGraph> prepareLiveRig(const LiveRigSettings& settings,
                                                     double sampleRate,
                                                     int maximumBlockSize,
                                                     int channels) {
    if (sampleRate <= 0.0 || maximumBlockSize <= 0 || channels <= 0) return nullptr;

    auto prepared = std::make_unique<graph::PreparedGraph>();
    auto& graph = prepared->graph;
    graph::NodeId previous = 0;

    const auto append = [&](std::unique_ptr<graph::AudioNode> node) mutable -> graph::NodeId {
        const auto id = graph.addNode(std::move(node));
        if (id == 0) return 0;
        if (previous != 0 && !graph.connect(previous, id)) return 0;
        previous = id;
        return id;
    };

    if (settings.pedal == PedalModel::ts808Circuit) {
        auto pedal = std::make_unique<hq::TS808CircuitNode>();
        setPedalParameters(*pedal, settings);
        if (append(std::move(pedal)) == 0) return nullptr;
    } else if (settings.pedal == PedalModel::ds1Circuit) {
        auto pedal = std::make_unique<hq::DS1CircuitNode>();
        setPedalParameters(*pedal, settings);
        if (append(std::move(pedal)) == 0) return nullptr;
    }

    if (settings.ampEnabled) {
        std::unique_ptr<graph::AudioNode> amp;
        switch (settings.amp) {
            case AmpModel::reference:
                amp = std::make_unique<hq::ReferenceAmpTopologyNode>();
                break;
            case AmpModel::britishPlexiFamily:
                amp = std::make_unique<hq::BritishPlexiFamilyNode>();
                break;
            case AmpModel::americanCleanFamily:
                amp = std::make_unique<hq::AmericanCleanFamilyNode>();
                break;
        }
        if (!amp) return nullptr;
        setAmpParameters(*amp, settings);
        if (append(std::move(amp)) == 0) return nullptr;
    }

    if (settings.cabinetEnabled) {
        auto cab = std::make_unique<hq::CabinetChainNode>();
        cab->setPartitionSize(std::clamp(settings.cabinetPartitionSize, 16, 1024));
        cab->setImpulseResponse(settings.cabinetImpulse.empty()
            ? makeReferenceCabinetImpulse(sampleRate)
            : settings.cabinetImpulse);
        cab->setParameterValue(0, settings.speakerCompression);
        cab->setParameterValue(1, settings.speakerExcursion);
        cab->setParameterValue(2, settings.speakerResonance);
        cab->setParameterValue(3, settings.cabinetOutputDb);
        if (append(std::move(cab)) == 0) return nullptr;
    }

    // A completely bypassed rig still needs one root/sink so the realtime graph
    // remains a valid allocation-free pass-through rather than a special host case.
    if (previous == 0) {
        if (append(std::make_unique<graph::GainNode>(1.0f)) == 0) return nullptr;
    }

    if (!prepared->runtime.build(graph, sampleRate, maximumBlockSize, channels, settings.quality))
        return nullptr;
    return prepared;
}

} // namespace guitardsp::app
