// Regression tests for issue #79: LiveRigPreset <-> JSON serialization, so a
// full LiveRig configuration (pedal/amp model, routing, every knob) can be
// saved as a named preset and restored bit for bit. Covers a non-default
// settings blob round-tripping exactly, forward-compatibility (unknown
// fields ignored, missing fields defaulted) and that malformed JSON is
// rejected with an error instead of crashing.
#include "guitardsp/app/LiveRigPresetJson.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace guitardsp::app;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

bool nearlyEqual(float a, float b) { return std::abs(a - b) < 1.0e-5f; }

LiveRigPreset buildSamplePreset() {
    LiveRigPreset preset;
    preset.version = 1;
    preset.name = "Lead Crunch";
    preset.guitarCabinetIrPath = "/home/user/IRs/4x12.wav";
    preset.bassCabinetIrPath = "";

    LiveRigSettings& s = preset.settings;
    s.quality = guitardsp::graph::ProcessingQuality::studio;
    s.pedal = PedalModel::ds1Circuit;
    s.amp = AmpModel::fullAmpCircuit;
    s.signalRouting = SignalRouting::crossoverOctaveBass;
    s.ampEnabled = true;
    s.cabinetEnabled = false;

    s.pedalDrive = 0.71f;
    s.pedalTone = 0.33f;
    s.pedalLevel = 0.62f;

    s.ampGain = 0.81f;
    s.ampBass = 0.44f;
    s.ampMid = 0.29f;
    s.ampTreble = 0.58f;
    s.ampMaster = 0.37f;
    s.ampPresence = 0.66f;
    s.ampOutputDb = -6.5f;
    s.ampPowerTube = 2.0f;
    s.ampToneStack = 1.0f;
    s.ampToneDriver = 2.0f;
    s.ampFeedbackVoicing = 1.0f;

    s.speakerCompression = 0.15f;
    s.speakerExcursion = 0.22f;
    s.speakerResonance = 0.41f;
    s.cabinetOutputDb = 3.5f;
    s.cabinetMix = 0.88f;
    s.cabinetLowCutHz = 88.0f;
    s.cabinetHighCutHz = 6800.0f;
    s.matchMeasuredCabinetLevel = false;
    s.cabinetPartitionSize = 128;

    s.octaveEnabled = false;
    s.bassCabinetEnabled = true;
    s.guitarBranchLevel = 0.93f;
    s.bassBranchLevel = 0.51f;
    s.crossoverFrequency = 210.0f;
    s.octaveMix = 0.72f;
    s.octaveLevel = 0.65f;
    s.bassGain = 0.39f;
    s.bassTone = 0.47f;
    s.bassLevel = 0.58f;
    return preset;
}

bool settingsEqual(const LiveRigSettings& a, const LiveRigSettings& b) {
    return a.quality == b.quality && a.pedal == b.pedal && a.amp == b.amp
        && a.signalRouting == b.signalRouting
        && a.ampEnabled == b.ampEnabled && a.cabinetEnabled == b.cabinetEnabled
        && nearlyEqual(a.pedalDrive, b.pedalDrive) && nearlyEqual(a.pedalTone, b.pedalTone)
        && nearlyEqual(a.pedalLevel, b.pedalLevel)
        && nearlyEqual(a.ampGain, b.ampGain) && nearlyEqual(a.ampBass, b.ampBass)
        && nearlyEqual(a.ampMid, b.ampMid) && nearlyEqual(a.ampTreble, b.ampTreble)
        && nearlyEqual(a.ampMaster, b.ampMaster) && nearlyEqual(a.ampPresence, b.ampPresence)
        && nearlyEqual(a.ampOutputDb, b.ampOutputDb) && nearlyEqual(a.ampPowerTube, b.ampPowerTube)
        && nearlyEqual(a.ampToneStack, b.ampToneStack) && nearlyEqual(a.ampToneDriver, b.ampToneDriver)
        && nearlyEqual(a.ampFeedbackVoicing, b.ampFeedbackVoicing)
        && nearlyEqual(a.speakerCompression, b.speakerCompression)
        && nearlyEqual(a.speakerExcursion, b.speakerExcursion)
        && nearlyEqual(a.speakerResonance, b.speakerResonance)
        && nearlyEqual(a.cabinetOutputDb, b.cabinetOutputDb) && nearlyEqual(a.cabinetMix, b.cabinetMix)
        && nearlyEqual(a.cabinetLowCutHz, b.cabinetLowCutHz)
        && nearlyEqual(a.cabinetHighCutHz, b.cabinetHighCutHz)
        && a.matchMeasuredCabinetLevel == b.matchMeasuredCabinetLevel
        && a.cabinetPartitionSize == b.cabinetPartitionSize
        && a.octaveEnabled == b.octaveEnabled && a.bassCabinetEnabled == b.bassCabinetEnabled
        && nearlyEqual(a.guitarBranchLevel, b.guitarBranchLevel)
        && nearlyEqual(a.bassBranchLevel, b.bassBranchLevel)
        && nearlyEqual(a.crossoverFrequency, b.crossoverFrequency)
        && nearlyEqual(a.octaveMix, b.octaveMix) && nearlyEqual(a.octaveLevel, b.octaveLevel)
        && nearlyEqual(a.bassGain, b.bassGain) && nearlyEqual(a.bassTone, b.bassTone)
        && nearlyEqual(a.bassLevel, b.bassLevel);
}

bool presetsEqual(const LiveRigPreset& a, const LiveRigPreset& b) {
    return a.version == b.version && a.name == b.name
        && a.guitarCabinetIrPath == b.guitarCabinetIrPath
        && a.bassCabinetIrPath == b.bassCabinetIrPath
        && settingsEqual(a.settings, b.settings);
}
} // namespace

int main() {
    bool ok = true;

    {
        const LiveRigPreset original = buildSamplePreset();
        const std::string json = liveRigPresetToJson(original);

        LiveRigPreset restored;
        std::string error;
        ok &= require(liveRigPresetFromJson(json, restored, &error),
                      "sample preset JSON round-trips without error");
        ok &= require(error.empty(), "successful parse clears the error string");
        ok &= require(presetsEqual(original, restored),
                      "restored preset matches the original field-for-field");
    }

    {
        // A default-constructed LiveRigSettings should round-trip too (the
        // "brand new preset" case where every field is at its default).
        LiveRigPreset original;
        original.name = "Defaults";
        const std::string json = liveRigPresetToJson(original);

        LiveRigPreset restored;
        std::string error;
        ok &= require(liveRigPresetFromJson(json, restored, &error),
                      "default-settings preset round-trips without error");
        ok &= require(presetsEqual(original, restored),
                      "restored default preset matches the original field-for-field");
    }

    {
        // Forward-compatibility: an unknown top-level/settings field must not
        // break parsing, and a missing field should fall back to
        // LiveRigSettings' own default member initializer.
        LiveRigPreset restored;
        std::string error;
        const std::string json =
            R"({"version":1,"name":"Partial","futureTopLevelField":42,)"
            R"("settings":{"pedalDrive":0.9,"unknownKnob":1.0}})";
        ok &= require(liveRigPresetFromJson(json, restored, &error),
                      "unknown fields are ignored rather than rejected");
        ok &= require(nearlyEqual(restored.settings.pedalDrive, 0.9f),
                      "known field present in a partial document is still read");
        ok &= require(restored.settings.pedal == PedalModel::ts808Circuit,
                      "field absent from a partial document falls back to LiveRigSettings' default");
    }

    {
        LiveRigPreset restored;
        std::string error;
        ok &= require(!liveRigPresetFromJson("{ this is not valid json", restored, &error),
                      "malformed JSON syntax is rejected instead of crashing");
        ok &= require(!error.empty(), "malformed JSON syntax reports a non-empty error");
    }

    {
        LiveRigPreset restored;
        std::string error;
        ok &= require(!liveRigPresetFromJson("[1, 2, 3]", restored, &error),
                      "a JSON array at the top level is rejected (not an object)");
        ok &= require(!error.empty(), "wrong top-level shape reports a non-empty error");
    }

    {
        LiveRigPreset restored;
        std::string error;
        ok &= require(!liveRigPresetFromJson(R"({"settings":[1,2]})", restored, &error),
                      "a non-object 'settings' value is rejected");
        ok &= require(!error.empty(), "invalid 'settings' shape reports a non-empty error");
    }

    return ok ? 0 : 1;
}
