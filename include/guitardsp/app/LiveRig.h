#pragma once

#include "guitardsp/graph/AudioNode.h"
#include "guitardsp/graph/RealtimeGraphHost.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace guitardsp::app {

enum class PedalModel {
    bypass,
    ts808Circuit,
    ds1Circuit
};

enum class AmpModel {
    reference,
    britishPlexiFamily,
    americanCleanFamily,
    preampCircuit,
    fullAmpCircuit
};

enum class SignalRouting {
    serialGuitar,
    parallelOctaveBass,
    crossoverOctaveBass
};

struct LiveRigSettings {
    graph::ProcessingQuality quality = graph::ProcessingQuality::high;
    PedalModel pedal = PedalModel::ts808Circuit;
    AmpModel amp = AmpModel::reference;
    SignalRouting signalRouting = SignalRouting::serialGuitar;
    bool ampEnabled = true;
    bool cabinetEnabled = true;

    float pedalDrive = 0.45f;
    float pedalTone = 0.50f;
    float pedalLevel = 0.55f;

    float ampGain = 0.35f;
    float ampBass = 0.50f;
    float ampMid = 0.50f;
    float ampTreble = 0.50f;
    float ampMaster = 0.42f;
    float ampPresence = 0.50f;
    float ampOutputDb = -12.0f;
    float ampPowerTube = 0.0f;
    float ampToneStack = 0.0f;
    float ampToneDriver = 0.0f;
    float ampFeedbackVoicing = 0.0f;

    float speakerCompression = 0.20f;
    float speakerExcursion = 0.18f;
    float speakerResonance = 0.35f;
    float cabinetOutputDb = 0.0f;
    float cabinetMix = 1.0f;
    float cabinetLowCutHz = 72.0f;
    float cabinetHighCutHz = 7200.0f;
    bool matchMeasuredCabinetLevel = true;
    int cabinetPartitionSize = 64;

    bool octaveEnabled = true;
    bool bassCabinetEnabled = true;
    float guitarBranchLevel = 1.0f;
    float bassBranchLevel = 0.45f;
    float crossoverFrequency = 180.0f;
    float octaveMix = 1.0f;
    float octaveLevel = 0.85f;
    float bassGain = 0.45f;
    float bassTone = 0.50f;
    float bassLevel = 0.65f;

    // Empty means use the clearly-labelled synthetic reference fallback IR. A
    // measured IR supplied by the JUCE host should already be resampled to the
    // active audio-device sample rate before it reaches this builder.
    std::vector<float> cabinetImpulse;

    // Same fallback semantics as cabinetImpulse, but for the dedicated bass
    // cabinet branch (see configureBassCabinet()).
    std::vector<float> bassCabinetImpulse;
};

// A place a waveform/spectrum monitor window can tap into. physicalInput/
// physicalOutput aren't graph nodes -- RealtimeAudioEngine reads those
// directly from its pre-DSP input block / post-DSP output block. The rest
// name a stage inside the SIGNAL CHAIN and are resolved against the live
// compiled graph by typeId (see monitorTapPointCandidates()).
enum class MonitorTapPoint : std::uint8_t {
    physicalInput,
    physicalOutput,
    pedalOutput,
    ampOutput,
    cabinetOutput,
    octaveOutput,
    bassAmpOutput,
    bassCabinetOutput,
};

// Which typeIds a given tap point could resolve to across every possible
// pedal/amp model choice. RealtimeAudioEngine doesn't track the active
// LiveRigSettings itself (it only ever sees the already-compiled graph), so
// it probes each candidate in turn against the live graph and uses whichever
// one is actually present -- exactly one will be, since pedal/amp selection
// always produces a single node of one of these types. Fixed-size and
// constexpr so this is real-time safe to call from the audio thread.
struct MonitorTapCandidates {
    std::array<const char*, 5> typeIds{};
    int count = 0;
};

[[nodiscard]] constexpr MonitorTapCandidates monitorTapPointCandidates(MonitorTapPoint point) noexcept {
    switch (point) {
        case MonitorTapPoint::pedalOutput:
            return {{"drive.ts808_circuit_hq", "drive.ds1_circuit_hq"}, 2};
        case MonitorTapPoint::ampOutput:
            return {{"amp.reference_hq", "amp.british_plexi_family_hq",
                      "amp.american_clean_family_hq", "drive.preamp_circuit_hq",
                      "amp.full_amp_circuit_hq"}, 5};
        case MonitorTapPoint::cabinetOutput: return {{"cab.chain_hq"}, 1};
        case MonitorTapPoint::octaveOutput: return {{"pitch.octave_down_mono"}, 1};
        case MonitorTapPoint::bassAmpOutput: return {{"amp.bass_reference_hq"}, 1};
        case MonitorTapPoint::bassCabinetOutput: return {{"cab.bass_reference_hq"}, 1};
        case MonitorTapPoint::physicalInput:
        case MonitorTapPoint::physicalOutput:
        default:
            return {};
    }
}

// Short label for a monitor-window tap-selection dropdown.
[[nodiscard]] const char* monitorTapPointLabel(MonitorTapPoint point) noexcept;

// Which tap points are actually reachable for the given settings, in display
// order. physicalInput/physicalOutput are always first. A SIGNAL CHAIN stage
// only appears once the settings actually produce a node for it (bypassed
// pedal, disabled amp/cabinet, serial routing with no bass branch, etc. all
// remove the corresponding entry) so a dropdown built from this never offers
// a tap point that doesn't exist in the live graph.
[[nodiscard]] std::vector<MonitorTapPoint> availableMonitorTapPoints(const LiveRigSettings& settings);

std::unique_ptr<graph::PreparedGraph> prepareLiveRig(const LiveRigSettings& settings,
                                                     double sampleRate,
                                                     int maximumBlockSize,
                                                     int channels);

} // namespace guitardsp::app
