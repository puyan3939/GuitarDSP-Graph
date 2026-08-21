#pragma once

#include "guitardsp/graph/AudioNode.h"
#include "guitardsp/graph/RealtimeGraphHost.h"

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
    americanCleanFamily
};

struct LiveRigSettings {
    graph::ProcessingQuality quality = graph::ProcessingQuality::high;
    PedalModel pedal = PedalModel::ts808Circuit;
    AmpModel amp = AmpModel::reference;
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

    float speakerCompression = 0.20f;
    float speakerExcursion = 0.18f;
    float speakerResonance = 0.35f;
    float cabinetOutputDb = 0.0f;
    int cabinetPartitionSize = 64;

    // Empty means use the clearly-labelled synthetic reference fallback IR. A
    // measured IR supplied by the JUCE host should already be resampled to the
    // active audio-device sample rate before it reaches this builder.
    std::vector<float> cabinetImpulse;
};

std::unique_ptr<graph::PreparedGraph> prepareLiveRig(const LiveRigSettings& settings,
                                                     double sampleRate,
                                                     int maximumBlockSize,
                                                     int channels);

} // namespace guitardsp::app
