#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/graph/AdvancedRoutingNodes.h"
#include "guitardsp/graph/UtilityNodes.h"
#include "guitardsp/hq/AmpFamilyNodes.h"
#include "guitardsp/hq/BassAmpNode.h"
#include "guitardsp/hq/CabinetChainNode.h"
#include "guitardsp/hq/DS1CircuitNode.h"
#include "guitardsp/hq/OctaveDownNode.h"
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
    if (settings.amp == AmpModel::reference) {
        node.setParameterValue(7, settings.ampPowerTube);
        node.setParameterValue(8, settings.ampToneStack);
        node.setParameterValue(9, settings.ampToneDriver);
        node.setParameterValue(10, settings.ampFeedbackVoicing);
    }
}

std::unique_ptr<graph::AudioNode> makePedal(const LiveRigSettings& settings) {
    std::unique_ptr<graph::AudioNode> pedal;
    if (settings.pedal == PedalModel::ts808Circuit)
        pedal = std::make_unique<hq::TS808CircuitNode>();
    else if (settings.pedal == PedalModel::ds1Circuit)
        pedal = std::make_unique<hq::DS1CircuitNode>();
    if (pedal) setPedalParameters(*pedal, settings);
    return pedal;
}

std::unique_ptr<graph::AudioNode> makeAmp(const LiveRigSettings& settings) {
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
    if (amp) setAmpParameters(*amp, settings);
    return amp;
}

std::unique_ptr<hq::CabinetChainNode> makeGuitarCabinet(const LiveRigSettings& settings,
                                                       double sampleRate) {
    auto cab = std::make_unique<hq::CabinetChainNode>();
    cab->setPartitionSize(std::clamp(settings.cabinetPartitionSize, 16, 1024));
    cab->setImpulseResponse(settings.cabinetImpulse.empty()
        ? makeReferenceCabinetImpulse(sampleRate)
        : settings.cabinetImpulse);
    cab->setParameterValue(0, settings.speakerCompression);
    cab->setParameterValue(1, settings.speakerExcursion);
    cab->setParameterValue(2, settings.speakerResonance);
    cab->setParameterValue(3, settings.cabinetOutputDb);
    cab->setParameterValue(4, settings.cabinetMix);
    cab->setParameterValue(5, settings.cabinetLowCutHz);
    cab->setParameterValue(6, settings.cabinetHighCutHz);
    return cab;
}

} // namespace

std::unique_ptr<graph::PreparedGraph> prepareLiveRig(const LiveRigSettings& settings,
                                                     double sampleRate,
                                                     int maximumBlockSize,
                                                     int channels) {
    if (sampleRate <= 0.0 || maximumBlockSize <= 0 || channels <= 0) return nullptr;

    auto prepared = std::make_unique<graph::PreparedGraph>();
    auto& graph = prepared->graph;
    graph::NodeId guitar = 0;
    int guitarPort = 0;

    auto append = [&](graph::NodeId& previous, int& previousPort,
                      std::unique_ptr<graph::AudioNode> node) -> graph::NodeId {
        const auto id = graph.addNode(std::move(node));
        if (id == 0) return 0;
        if (previous != 0 && !graph.connect(previous, previousPort, id, 0)) return 0;
        previous = id;
        previousPort = 0;
        return id;
    };

    graph::NodeId splitter = 0;
    if (settings.signalRouting != SignalRouting::serialGuitar) {
        if (settings.signalRouting == SignalRouting::crossoverOctaveBass) {
            auto crossover = std::make_unique<graph::CrossoverSplitNode>();
            crossover->setParameterValue(0, settings.crossoverFrequency);
            splitter = graph.addNode(std::move(crossover));
            guitarPort = 1; // High band to guitar, low band to octave/bass.
        } else {
            splitter = graph.addNode(std::make_unique<graph::SplitNode>());
        }
        if (splitter == 0) return nullptr;
        guitar = splitter;
    }

    if (auto pedal = makePedal(settings)) {
        if (append(guitar, guitarPort, std::move(pedal)) == 0) return nullptr;
    }
    if (settings.ampEnabled) {
        auto amp = makeAmp(settings);
        if (!amp) return nullptr;
        if (append(guitar, guitarPort, std::move(amp)) == 0) return nullptr;
    }

    if (settings.cabinetEnabled) {
        if (append(guitar, guitarPort, makeGuitarCabinet(settings, sampleRate)) == 0)
            return nullptr;
    }

    if (splitter != 0) {
        if (append(guitar, guitarPort,
                   std::make_unique<graph::BranchLevelNode>(
                       graph::BranchLevelNode::Branch::guitar,
                       settings.guitarBranchLevel)) == 0)
            return nullptr;

        graph::NodeId bass = splitter;
        int bassPort = 0;
        if (settings.octaveEnabled) {
            auto octave = std::make_unique<hq::OctaveDownNode>();
            octave->setParameterValue(0, settings.octaveMix);
            octave->setParameterValue(1, settings.octaveLevel);
            if (append(bass, bassPort, std::move(octave)) == 0) return nullptr;
        }

        auto bassAmp = std::make_unique<hq::BassAmpNode>();
        bassAmp->setParameterValue(0, settings.bassGain);
        bassAmp->setParameterValue(1, settings.bassTone);
        bassAmp->setParameterValue(2, settings.bassLevel);
        if (append(bass, bassPort, std::move(bassAmp)) == 0) return nullptr;

        if (settings.bassCabinetEnabled) {
            auto bassCab = std::make_unique<hq::BassCabinetNode>();
            bassCab->setPartitionSize(std::clamp(settings.cabinetPartitionSize, 16, 1024));
            bassCab->setImpulseResponse(makeReferenceBassCabinetImpulse(sampleRate));
            bassCab->setParameterValue(0, 0.12f);
            bassCab->setParameterValue(1, 0.10f);
            bassCab->setParameterValue(2, 0.28f);
            if (append(bass, bassPort, std::move(bassCab)) == 0) return nullptr;
        }

        if (append(bass, bassPort,
                   std::make_unique<graph::BranchLevelNode>(
                       graph::BranchLevelNode::Branch::bass,
                       settings.bassBranchLevel)) == 0)
            return nullptr;

        const auto merge = graph.addNode(std::make_unique<graph::MergeNode>());
        if (merge == 0 || !graph.connect(guitar, merge) || !graph.connect(bass, merge))
            return nullptr;
        guitar = merge;
    }

    // A completely bypassed rig still needs one root/sink so the realtime graph
    // remains a valid allocation-free pass-through rather than a special host case.
    if (guitar == 0) {
        if (append(guitar, guitarPort, std::make_unique<graph::GainNode>(1.0f)) == 0)
            return nullptr;
    }

    if (!prepared->runtime.build(graph, sampleRate, maximumBlockSize, channels, settings.quality))
        return nullptr;
    return prepared;
}

} // namespace guitardsp::app
