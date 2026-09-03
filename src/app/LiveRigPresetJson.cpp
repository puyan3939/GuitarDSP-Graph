#include "guitardsp/app/LiveRigPresetJson.h"

#include "guitardsp/circuit/JsonValue.h"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace guitardsp::app {

namespace {

using guitardsp::circuit::JsonParseError;
using guitardsp::circuit::JsonValue;
using guitardsp::circuit::parseJson;

std::string_view qualityToString(graph::ProcessingQuality quality) noexcept {
    switch (quality) {
        case graph::ProcessingQuality::eco: return "eco";
        case graph::ProcessingQuality::live: return "live";
        case graph::ProcessingQuality::high: return "high";
        case graph::ProcessingQuality::studio: return "studio";
    }
    return "high";
}

graph::ProcessingQuality qualityFromString(const std::string& text) noexcept {
    if (text == "eco") return graph::ProcessingQuality::eco;
    if (text == "live") return graph::ProcessingQuality::live;
    if (text == "studio") return graph::ProcessingQuality::studio;
    return graph::ProcessingQuality::high;
}

std::string_view pedalToString(PedalModel pedal) noexcept {
    switch (pedal) {
        case PedalModel::bypass: return "bypass";
        case PedalModel::ts808Circuit: return "ts808Circuit";
        case PedalModel::ds1Circuit: return "ds1Circuit";
    }
    return "ts808Circuit";
}

PedalModel pedalFromString(const std::string& text) noexcept {
    if (text == "bypass") return PedalModel::bypass;
    if (text == "ds1Circuit") return PedalModel::ds1Circuit;
    return PedalModel::ts808Circuit;
}

std::string_view ampToString(AmpModel amp) noexcept {
    switch (amp) {
        case AmpModel::reference: return "reference";
        case AmpModel::britishPlexiFamily: return "britishPlexiFamily";
        case AmpModel::americanCleanFamily: return "americanCleanFamily";
        case AmpModel::preampCircuit: return "preampCircuit";
        case AmpModel::fullAmpCircuit: return "fullAmpCircuit";
    }
    return "reference";
}

AmpModel ampFromString(const std::string& text) noexcept {
    if (text == "britishPlexiFamily") return AmpModel::britishPlexiFamily;
    if (text == "americanCleanFamily") return AmpModel::americanCleanFamily;
    if (text == "preampCircuit") return AmpModel::preampCircuit;
    if (text == "fullAmpCircuit") return AmpModel::fullAmpCircuit;
    return AmpModel::reference;
}

std::string_view routingToString(SignalRouting routing) noexcept {
    switch (routing) {
        case SignalRouting::serialGuitar: return "serialGuitar";
        case SignalRouting::parallelOctaveBass: return "parallelOctaveBass";
        case SignalRouting::crossoverOctaveBass: return "crossoverOctaveBass";
    }
    return "serialGuitar";
}

SignalRouting routingFromString(const std::string& text) noexcept {
    if (text == "parallelOctaveBass") return SignalRouting::parallelOctaveBass;
    if (text == "crossoverOctaveBass") return SignalRouting::crossoverOctaveBass;
    return SignalRouting::serialGuitar;
}

void writeEscapedString(std::string& out, std::string_view value) {
    out.push_back('"');
    for (const char c : value) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out += buf;
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

// %.9g round-trips a 32-bit float exactly (9 significant decimal digits is
// always enough to disambiguate any two distinct float values).
void writeFloat(std::string& out, float value) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(value));
    out += buf;
}

void writeField(std::string& out, std::string_view key, float value) {
    out.push_back(',');
    writeEscapedString(out, key);
    out.push_back(':');
    writeFloat(out, value);
}

void writeField(std::string& out, std::string_view key, bool value) {
    out.push_back(',');
    writeEscapedString(out, key);
    out += ':';
    out += value ? "true" : "false";
}

void writeField(std::string& out, std::string_view key, int value) {
    out.push_back(',');
    writeEscapedString(out, key);
    out += ':';
    out += std::to_string(value);
}

} // namespace

std::string liveRigPresetToJson(const LiveRigPreset& preset) {
    const LiveRigSettings& s = preset.settings;
    std::string out;
    out += "{\"version\":" + std::to_string(preset.version);
    out += ",\"name\":"; writeEscapedString(out, preset.name);
    out += ",\"guitarCabinetIrPath\":"; writeEscapedString(out, preset.guitarCabinetIrPath);
    out += ",\"bassCabinetIrPath\":"; writeEscapedString(out, preset.bassCabinetIrPath);

    out += ",\"settings\":{";
    out += "\"quality\":"; writeEscapedString(out, qualityToString(s.quality));
    out += ",\"pedal\":"; writeEscapedString(out, pedalToString(s.pedal));
    out += ",\"amp\":"; writeEscapedString(out, ampToString(s.amp));
    out += ",\"signalRouting\":"; writeEscapedString(out, routingToString(s.signalRouting));
    writeField(out, "ampEnabled", s.ampEnabled);
    writeField(out, "cabinetEnabled", s.cabinetEnabled);

    writeField(out, "pedalDrive", s.pedalDrive);
    writeField(out, "pedalTone", s.pedalTone);
    writeField(out, "pedalLevel", s.pedalLevel);

    writeField(out, "ampGain", s.ampGain);
    writeField(out, "ampBass", s.ampBass);
    writeField(out, "ampMid", s.ampMid);
    writeField(out, "ampTreble", s.ampTreble);
    writeField(out, "ampMaster", s.ampMaster);
    writeField(out, "ampPresence", s.ampPresence);
    writeField(out, "ampOutputDb", s.ampOutputDb);
    writeField(out, "ampPowerTube", s.ampPowerTube);
    writeField(out, "ampToneStack", s.ampToneStack);
    writeField(out, "ampToneDriver", s.ampToneDriver);
    writeField(out, "ampFeedbackVoicing", s.ampFeedbackVoicing);

    writeField(out, "speakerCompression", s.speakerCompression);
    writeField(out, "speakerExcursion", s.speakerExcursion);
    writeField(out, "speakerResonance", s.speakerResonance);
    writeField(out, "cabinetOutputDb", s.cabinetOutputDb);
    writeField(out, "cabinetMix", s.cabinetMix);
    writeField(out, "cabinetLowCutHz", s.cabinetLowCutHz);
    writeField(out, "cabinetHighCutHz", s.cabinetHighCutHz);
    writeField(out, "matchMeasuredCabinetLevel", s.matchMeasuredCabinetLevel);
    writeField(out, "cabinetPartitionSize", s.cabinetPartitionSize);

    writeField(out, "octaveEnabled", s.octaveEnabled);
    writeField(out, "bassCabinetEnabled", s.bassCabinetEnabled);
    writeField(out, "guitarBranchLevel", s.guitarBranchLevel);
    writeField(out, "bassBranchLevel", s.bassBranchLevel);
    writeField(out, "crossoverFrequency", s.crossoverFrequency);
    writeField(out, "octaveMix", s.octaveMix);
    writeField(out, "octaveLevel", s.octaveLevel);
    writeField(out, "bassGain", s.bassGain);
    writeField(out, "bassTone", s.bassTone);
    writeField(out, "bassLevel", s.bassLevel);
    out += "}}";
    return out;
}

bool liveRigPresetFromJson(std::string_view json, LiveRigPreset& outPreset, std::string* error) {
    JsonValue root;
    try {
        root = parseJson(json);
    } catch (const JsonParseError& e) {
        if (error != nullptr) *error = e.what();
        return false;
    }
    if (!root.isObject()) {
        if (error != nullptr) *error = "preset must be a JSON object";
        return false;
    }

    LiveRigPreset preset;
    preset.version = root["version"].asInt(1);
    preset.name = root.has("name") ? root["name"].asString() : std::string();
    preset.guitarCabinetIrPath = root.has("guitarCabinetIrPath")
        ? root["guitarCabinetIrPath"].asString() : std::string();
    preset.bassCabinetIrPath = root.has("bassCabinetIrPath")
        ? root["bassCabinetIrPath"].asString() : std::string();

    const JsonValue& settingsJson = root["settings"];
    if (!settingsJson.isNull() && !settingsJson.isObject()) {
        if (error != nullptr) *error = "'settings' must be a JSON object";
        return false;
    }

    LiveRigSettings& s = preset.settings;
    if (settingsJson.isObject()) {
        s.quality = settingsJson.has("quality")
            ? qualityFromString(settingsJson["quality"].asString()) : s.quality;
        s.pedal = settingsJson.has("pedal")
            ? pedalFromString(settingsJson["pedal"].asString()) : s.pedal;
        s.amp = settingsJson.has("amp")
            ? ampFromString(settingsJson["amp"].asString()) : s.amp;
        s.signalRouting = settingsJson.has("signalRouting")
            ? routingFromString(settingsJson["signalRouting"].asString()) : s.signalRouting;
        s.ampEnabled = settingsJson["ampEnabled"].asBool(s.ampEnabled);
        s.cabinetEnabled = settingsJson["cabinetEnabled"].asBool(s.cabinetEnabled);

        s.pedalDrive = settingsJson["pedalDrive"].asFloat(s.pedalDrive);
        s.pedalTone = settingsJson["pedalTone"].asFloat(s.pedalTone);
        s.pedalLevel = settingsJson["pedalLevel"].asFloat(s.pedalLevel);

        s.ampGain = settingsJson["ampGain"].asFloat(s.ampGain);
        s.ampBass = settingsJson["ampBass"].asFloat(s.ampBass);
        s.ampMid = settingsJson["ampMid"].asFloat(s.ampMid);
        s.ampTreble = settingsJson["ampTreble"].asFloat(s.ampTreble);
        s.ampMaster = settingsJson["ampMaster"].asFloat(s.ampMaster);
        s.ampPresence = settingsJson["ampPresence"].asFloat(s.ampPresence);
        s.ampOutputDb = settingsJson["ampOutputDb"].asFloat(s.ampOutputDb);
        s.ampPowerTube = settingsJson["ampPowerTube"].asFloat(s.ampPowerTube);
        s.ampToneStack = settingsJson["ampToneStack"].asFloat(s.ampToneStack);
        s.ampToneDriver = settingsJson["ampToneDriver"].asFloat(s.ampToneDriver);
        s.ampFeedbackVoicing = settingsJson["ampFeedbackVoicing"].asFloat(s.ampFeedbackVoicing);

        s.speakerCompression = settingsJson["speakerCompression"].asFloat(s.speakerCompression);
        s.speakerExcursion = settingsJson["speakerExcursion"].asFloat(s.speakerExcursion);
        s.speakerResonance = settingsJson["speakerResonance"].asFloat(s.speakerResonance);
        s.cabinetOutputDb = settingsJson["cabinetOutputDb"].asFloat(s.cabinetOutputDb);
        s.cabinetMix = settingsJson["cabinetMix"].asFloat(s.cabinetMix);
        s.cabinetLowCutHz = settingsJson["cabinetLowCutHz"].asFloat(s.cabinetLowCutHz);
        s.cabinetHighCutHz = settingsJson["cabinetHighCutHz"].asFloat(s.cabinetHighCutHz);
        s.matchMeasuredCabinetLevel =
            settingsJson["matchMeasuredCabinetLevel"].asBool(s.matchMeasuredCabinetLevel);
        s.cabinetPartitionSize = settingsJson["cabinetPartitionSize"].asInt(s.cabinetPartitionSize);

        s.octaveEnabled = settingsJson["octaveEnabled"].asBool(s.octaveEnabled);
        s.bassCabinetEnabled = settingsJson["bassCabinetEnabled"].asBool(s.bassCabinetEnabled);
        s.guitarBranchLevel = settingsJson["guitarBranchLevel"].asFloat(s.guitarBranchLevel);
        s.bassBranchLevel = settingsJson["bassBranchLevel"].asFloat(s.bassBranchLevel);
        s.crossoverFrequency = settingsJson["crossoverFrequency"].asFloat(s.crossoverFrequency);
        s.octaveMix = settingsJson["octaveMix"].asFloat(s.octaveMix);
        s.octaveLevel = settingsJson["octaveLevel"].asFloat(s.octaveLevel);
        s.bassGain = settingsJson["bassGain"].asFloat(s.bassGain);
        s.bassTone = settingsJson["bassTone"].asFloat(s.bassTone);
        s.bassLevel = settingsJson["bassLevel"].asFloat(s.bassLevel);
    }

    outPreset = std::move(preset);
    if (error != nullptr) error->clear();
    return true;
}

bool savePresetToFile(const LiveRigPreset& preset, const std::string& path, std::string* error) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        if (error != nullptr) *error = "unable to open file for writing: " + path;
        return false;
    }
    file << liveRigPresetToJson(preset);
    if (!file) {
        if (error != nullptr) *error = "failed writing preset to file: " + path;
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}

bool loadPresetFromFile(const std::string& path, LiveRigPreset& outPreset, std::string* error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (error != nullptr) *error = "unable to open preset file: " + path;
        return false;
    }
    std::ostringstream contents;
    contents << file.rdbuf();
    return liveRigPresetFromJson(contents.str(), outPreset, error);
}

} // namespace guitardsp::app
