#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/graph/GraphBuilder.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/graph/RigTopologyBuilder.h"
#include "guitardsp/hq/BassAmpNode.h"
#include "guitardsp/hq/CabinetChainNode.h"
#include "guitardsp/hq/CompressorCircuitNode.h"

#include <algorithm>
#include <memory>
#include <utility>

namespace guitardsp::app {
namespace {

using graph::ChainNodeSpec;
using graph::RoutingBranch;
using graph::RoutingTopology;

void setPedalParameters(graph::AudioNode& node, const LiveRigSettings& settings) {
    node.setParameterValue(0, settings.pedalDrive);
    node.setParameterValue(1, settings.pedalTone);
    node.setParameterValue(2, settings.pedalLevel);
}

// PreampCircuitNode/FullAmpCircuitNode are component-level MNA circuits with
// only three controls -- Drive/Bass/Treble -- unlike the parameterized
// reference/family amp models' full Gain/Bass/Mid/Treble/Master/Presence/
// Output(+reference-only power-tube/tone-stack/driver/feedback) set. Reuse
// the Gain/Bass/Treble knobs for Drive/Bass/Treble rather than adding
// dedicated settings fields.
bool isCircuitLevelAmp(AmpModel amp) {
    return amp == AmpModel::preampCircuit || amp == AmpModel::fullAmpCircuit;
}

void setAmpParameters(graph::AudioNode& node, const LiveRigSettings& settings) {
    if (isCircuitLevelAmp(settings.amp)) {
        node.setParameterValue(0, settings.ampGain);
        node.setParameterValue(1, settings.ampBass);
        node.setParameterValue(2, settings.ampTreble);
        return;
    }
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

const char* pedalTypeId(const LiveRigSettings& settings) {
    if (settings.pedal == PedalModel::ts808Circuit) return "drive.ts808_circuit_hq";
    if (settings.pedal == PedalModel::ds1Circuit) return "drive.ds1_circuit_hq";
    return nullptr;
}

const char* ampTypeId(const LiveRigSettings& settings) {
    switch (settings.amp) {
        case AmpModel::reference: return "amp.reference_hq";
        case AmpModel::britishPlexiFamily: return "amp.british_plexi_family_hq";
        case AmpModel::americanCleanFamily: return "amp.american_clean_family_hq";
        case AmpModel::preampCircuit: return "drive.preamp_circuit_hq";
        case AmpModel::fullAmpCircuit: return "amp.full_amp_circuit_hq";
    }
    return nullptr;
}

// The guitar/bass cabinets need a sample-rate-derived impulse response and a
// partition size, which are host-provided and not part of the generic
// ParameterDescriptor set, so they are configured here after node construction
// rather than through GraphDocument parameters.
void configureGuitarCabinet(hq::CabinetChainNode& cab, const LiveRigSettings& settings,
                            double sampleRate) {
    cab.setPartitionSize(std::clamp(settings.cabinetPartitionSize, 16, 1024));
    cab.setImpulseResponse(settings.cabinetImpulse.empty()
        ? makeReferenceCabinetImpulse(sampleRate)
        : settings.cabinetImpulse);
    cab.setParameterValue(0, settings.speakerCompression);
    cab.setParameterValue(1, settings.speakerExcursion);
    cab.setParameterValue(2, settings.speakerResonance);
    cab.setParameterValue(3, settings.cabinetOutputDb);
    cab.setParameterValue(4, settings.cabinetMix);
    cab.setParameterValue(5, settings.cabinetLowCutHz);
    cab.setParameterValue(6, settings.cabinetHighCutHz);
}

void configureBassCabinet(hq::BassCabinetNode& cab, const LiveRigSettings& settings,
                          double sampleRate) {
    cab.setPartitionSize(std::clamp(settings.cabinetPartitionSize, 16, 1024));
    cab.setImpulseResponse(settings.bassCabinetImpulse.empty()
        ? makeReferenceBassCabinetImpulse(sampleRate)
        : settings.bassCabinetImpulse);
    cab.setParameterValue(0, 0.12f);
    cab.setParameterValue(1, 0.10f);
    cab.setParameterValue(2, 0.28f);
}

// Applies LiveRigSettings-derived parameters/setup to every node the topology
// below produced, matched by the same NodeRegistry typeId used to build it.
void applySettings(graph::Graph& graph, const graph::GraphDocument& document,
                   const std::unordered_map<graph::NodeId, graph::NodeId>& documentToRuntimeId,
                   const LiveRigSettings& settings, double sampleRate) {
    for (const auto& nodeDoc : document.nodes) {
        const auto mapping = documentToRuntimeId.find(nodeDoc.id);
        if (mapping == documentToRuntimeId.end()) continue;
        auto* node = graph.node(mapping->second);
        if (!node) continue;

        if (nodeDoc.typeId == "dynamics.compressor_circuit_hq") {
            node->setParameterValue(0, settings.compressorMakeupGain);
        } else if (nodeDoc.typeId == "time.digital_delay") {
            node->setParameterValue(0, settings.delayTimeMs);
            node->setParameterValue(1, settings.delayFeedback);
            node->setParameterValue(2, settings.delayTone);
            node->setParameterValue(3, settings.delayMix);
        } else if (nodeDoc.typeId == "drive.ts808_circuit_hq" || nodeDoc.typeId == "drive.ds1_circuit_hq") {
            setPedalParameters(*node, settings);
        } else if (nodeDoc.typeId == "amp.reference_hq" || nodeDoc.typeId == "amp.british_plexi_family_hq"
                   || nodeDoc.typeId == "amp.american_clean_family_hq"
                   || nodeDoc.typeId == "drive.preamp_circuit_hq"
                   || nodeDoc.typeId == "amp.full_amp_circuit_hq") {
            setAmpParameters(*node, settings);
        } else if (nodeDoc.typeId == "cab.chain_hq") {
            if (auto* cab = dynamic_cast<hq::CabinetChainNode*>(node))
                configureGuitarCabinet(*cab, settings, sampleRate);
        } else if (nodeDoc.typeId == "route.crossover") {
            node->setParameterValue(0, settings.crossoverFrequency);
        } else if (nodeDoc.typeId == "route.guitar_level") {
            node->setParameterValue(0, settings.guitarBranchLevel);
        } else if (nodeDoc.typeId == "route.bass_level") {
            node->setParameterValue(0, settings.bassBranchLevel);
        } else if (nodeDoc.typeId == "pitch.octave_down_mono") {
            node->setParameterValue(0, settings.octaveMix);
            node->setParameterValue(1, settings.octaveLevel);
        } else if (nodeDoc.typeId == "amp.bass_reference_hq") {
            node->setParameterValue(0, settings.bassGain);
            node->setParameterValue(1, settings.bassTone);
            node->setParameterValue(2, settings.bassLevel);
        } else if (nodeDoc.typeId == "cab.bass_reference_hq") {
            if (auto* cab = dynamic_cast<hq::BassCabinetNode*>(node))
                configureBassCabinet(*cab, settings, sampleRate);
        }
    }
}

// Translates the fixed LiveRigSettings knobs (SignalRouting/pedal/amp/cabinet
// enables) into a RoutingTopology, the same general-purpose branch/chain
// description any custom rig can build. This is what keeps prepareLiveRig's
// behaviour byte-for-byte identical while routing it through GraphBuilder.
RoutingTopology buildLiveRigTopology(const LiveRigSettings& settings) {
    std::vector<ChainNodeSpec> mainChain;
    // Dynamics up front (ahead of drive), time-based effects at the tail
    // (after the cabinet) -- the conventional guitar signal-chain ordering
    // agreed in issue #83.
    if (settings.compressorEnabled) mainChain.push_back({"dynamics.compressor_circuit_hq", {}});
    if (const char* pedal = pedalTypeId(settings)) mainChain.push_back({pedal, {}});
    if (settings.ampEnabled) {
        if (const char* amp = ampTypeId(settings)) mainChain.push_back({amp, {}});
    }
    if (settings.cabinetEnabled) mainChain.push_back({"cab.chain_hq", {}});
    if (settings.delayEnabled) mainChain.push_back({"time.digital_delay", {}});

    RoutingTopology topology;
    if (settings.signalRouting == SignalRouting::serialGuitar) {
        topology.chain = std::move(mainChain);
        return topology;
    }

    const bool crossover = settings.signalRouting == SignalRouting::crossoverOctaveBass;
    topology.splitter = ChainNodeSpec{crossover ? "route.crossover" : "route.split", {}};

    RoutingBranch guitarBranch;
    guitarBranch.sourcePort = crossover ? 1 : 0; // crossover: high band to guitar, low band to octave/bass.
    guitarBranch.chain = std::move(mainChain);
    guitarBranch.chain.push_back({"route.guitar_level", {}});

    RoutingBranch bassBranch;
    bassBranch.sourcePort = 0;
    if (settings.octaveEnabled) bassBranch.chain.push_back({"pitch.octave_down_mono", {}});
    bassBranch.chain.push_back({"amp.bass_reference_hq", {}});
    if (settings.bassCabinetEnabled) bassBranch.chain.push_back({"cab.bass_reference_hq", {}});
    bassBranch.chain.push_back({"route.bass_level", {}});

    topology.branches = {std::move(guitarBranch), std::move(bassBranch)};
    topology.merge = ChainNodeSpec{"route.merge", {}};
    return topology;
}

} // namespace

const char* monitorTapPointLabel(MonitorTapPoint point) noexcept {
    switch (point) {
        case MonitorTapPoint::physicalInput: return "INPUT (physical)";
        case MonitorTapPoint::physicalOutput: return "OUTPUT (physical)";
        case MonitorTapPoint::compressorOutput: return "COMPRESSOR out";
        case MonitorTapPoint::pedalOutput: return "CIRCUIT PEDAL out";
        case MonitorTapPoint::ampOutput: return "GUITAR AMPLIFIER out";
        case MonitorTapPoint::cabinetOutput: return "SPEAKER+CABINET out";
        case MonitorTapPoint::delayOutput: return "DELAY out";
        case MonitorTapPoint::octaveOutput: return "OCTAVE (bass branch) out";
        case MonitorTapPoint::bassAmpOutput: return "BASS AMP out";
        case MonitorTapPoint::bassCabinetOutput: return "BASS CABINET out";
    }
    return "";
}

std::vector<MonitorTapPoint> availableMonitorTapPoints(const LiveRigSettings& settings) {
    std::vector<MonitorTapPoint> points{MonitorTapPoint::physicalInput, MonitorTapPoint::physicalOutput};
    if (settings.compressorEnabled) points.push_back(MonitorTapPoint::compressorOutput);
    if (pedalTypeId(settings) != nullptr) points.push_back(MonitorTapPoint::pedalOutput);
    if (settings.ampEnabled && ampTypeId(settings) != nullptr) points.push_back(MonitorTapPoint::ampOutput);
    if (settings.cabinetEnabled) points.push_back(MonitorTapPoint::cabinetOutput);
    if (settings.delayEnabled) points.push_back(MonitorTapPoint::delayOutput);
    if (settings.signalRouting != SignalRouting::serialGuitar) {
        // Bass branch always has amp.bass_reference_hq (see
        // buildLiveRigTopology()); octave/bass-cabinet stages inside it are
        // individually optional.
        if (settings.octaveEnabled) points.push_back(MonitorTapPoint::octaveOutput);
        points.push_back(MonitorTapPoint::bassAmpOutput);
        if (settings.bassCabinetEnabled) points.push_back(MonitorTapPoint::bassCabinetOutput);
    }
    return points;
}

std::unique_ptr<graph::PreparedGraph> prepareLiveRig(const LiveRigSettings& settings,
                                                     double sampleRate,
                                                     int maximumBlockSize,
                                                     int channels) {
    if (sampleRate <= 0.0 || maximumBlockSize <= 0 || channels <= 0) return nullptr;

    const auto document = graph::buildTopologyDocument(buildLiveRigTopology(settings));
    const auto registry = graph::NodeRegistry::createBuiltins();

    auto prepared = std::make_unique<graph::PreparedGraph>();
    auto build = graph::buildGraphFromDocument(document, registry, prepared->graph);
    if (!build.ok) return nullptr;

    applySettings(prepared->graph, document, build.documentToRuntimeId, settings, sampleRate);
    prepared->documentToRuntimeId = std::move(build.documentToRuntimeId);
    // Monitor taps (see RealtimeAudioEngine::resolveMonitorNodeTap()) resolve
    // a SIGNAL CHAIN stage by typeId against this index.
    graph::indexTypeIds(document, *prepared);

    if (!prepared->runtime.build(prepared->graph, sampleRate, maximumBlockSize, channels, settings.quality))
        return nullptr;
    return prepared;
}

} // namespace guitardsp::app
