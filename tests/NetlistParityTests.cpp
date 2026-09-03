// Parity check between the hand-written component-level pedal circuits
// (TS808Circuit, DS1Circuit) and the equivalent data-driven JSON netlists
// loaded through NetlistLoader.h (data/circuits/ts808.json, ds1.json). See
// docs/CIRCUIT_NETLIST_FORMAT.md for the netlist format itself.
//
// This does not merely check that both circuits produce "similar sounding"
// output: it replays the exact same node/component creation order so the
// underlying MNA unknown numbering is identical, then asserts sample-by-
// sample agreement to a tight tolerance across a matrix of drive/tone/level
// settings, matching the "NetlistParityCheck" pattern requested for the
// amp/cabinet netlist follow-up.

#include "guitardsp/circuit/CompressorCircuit.h"
#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/NetlistLoader.h"
#include "guitardsp/circuit/PowerAmpCircuit.h"
#include "guitardsp/circuit/PreampCircuit.h"
#include "guitardsp/circuit/TS808Circuit.h"

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace guitardsp;

namespace {

bool require(bool condition, const std::string& name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

#ifndef GUITARDSP_NETLIST_DATA_DIR
#define GUITARDSP_NETLIST_DATA_DIR "data/circuits"
#endif

constexpr double sampleRate = 48000.0;
constexpr double pi = 3.14159265358979323846;

std::vector<float> sineBurst(int count, float amplitude, float frequencyHz, int phaseOffsetSamples = 0) {
    std::vector<float> samples(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double t = static_cast<double>(i + phaseOffsetSamples) / sampleRate;
        samples[static_cast<std::size_t>(i)] =
            amplitude * static_cast<float>(std::sin(2.0 * pi * frequencyHz * t));
    }
    return samples;
}

struct Comparison {
    bool ok = true;
    float maxAbsDifference = 0.0f;
    float maxAbsReference = 0.0f;
};

Comparison compare(const std::vector<float>& reference, const std::vector<float>& candidate,
                   float tolerance) {
    Comparison result;
    result.ok = reference.size() == candidate.size();
    for (std::size_t i = 0; i < reference.size() && result.ok; ++i) {
        const float diff = std::abs(reference[i] - candidate[i]);
        result.maxAbsDifference = std::max(result.maxAbsDifference, diff);
        result.maxAbsReference = std::max(result.maxAbsReference, std::abs(reference[i]));
        if (!std::isfinite(reference[i]) || !std::isfinite(candidate[i]) || diff > tolerance) {
            result.ok = false;
        }
    }
    return result;
}

bool checkTs808Parity(float drive, float tone, float level) {
    circuit::TS808Circuit reference;
    if (!reference.prepare(sampleRate)) return require(false, "TS808 reference prepare()");
    reference.setControls(drive, tone, level);

    circuit::NetlistCircuit candidate;
    std::string error;
    if (!candidate.loadFromFile(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/ts808.json", &error))
        return require(false, "TS808 netlist load: " + error);
    if (!candidate.prepare(sampleRate, &error))
        return require(false, "TS808 netlist prepare(): " + error);
    candidate.setControl("drive", drive);
    candidate.setControl("tone", tone);
    candidate.setControl("level", level);

    // Let the potentiometer ramp and coupling capacitors settle identically
    // on both sides before the sample-by-sample comparison.
    const auto settle = sineBurst(4000, 0.12f, 220.0);
    for (float x : settle) {
        reference.processSample(x);
        candidate.processSample(x);
    }

    const auto probe = sineBurst(2000, 0.12f, 220.0, 4000);
    std::vector<float> referenceOut(probe.size());
    std::vector<float> candidateOut(probe.size());
    for (std::size_t i = 0; i < probe.size(); ++i) {
        referenceOut[i] = reference.processSample(probe[i]);
        candidateOut[i] = candidate.processSample(probe[i]);
    }

    const auto cmp = compare(referenceOut, candidateOut, 5.0e-4f);
    const std::string label = "TS808 parity drive=" + std::to_string(drive) +
        " tone=" + std::to_string(tone) + " level=" + std::to_string(level) +
        " (maxDiff=" + std::to_string(cmp.maxAbsDifference) +
        ", maxRef=" + std::to_string(cmp.maxAbsReference) + ")";
    return require(cmp.ok && cmp.maxAbsReference > 1.0e-4f, label);
}

bool checkDs1Parity(float distortion, float tone, float level) {
    circuit::DS1Circuit reference;
    if (!reference.prepare(sampleRate)) return require(false, "DS-1 reference prepare()");
    reference.setControls(distortion, tone, level);

    circuit::NetlistCircuit candidate;
    std::string error;
    if (!candidate.loadFromFile(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/ds1.json", &error))
        return require(false, "DS-1 netlist load: " + error);
    if (!candidate.prepare(sampleRate, &error))
        return require(false, "DS-1 netlist prepare(): " + error);
    candidate.setControl("distortion", distortion);
    candidate.setControl("tone", tone);
    candidate.setControl("level", level);

    const auto settle = sineBurst(4000, 0.12f, 220.0);
    for (float x : settle) {
        reference.processSample(x);
        candidate.processSample(x);
    }

    const auto probe = sineBurst(2000, 0.12f, 220.0, 4000);
    std::vector<float> referenceOut(probe.size());
    std::vector<float> candidateOut(probe.size());
    for (std::size_t i = 0; i < probe.size(); ++i) {
        referenceOut[i] = reference.processSample(probe[i]);
        candidateOut[i] = candidate.processSample(probe[i]);
    }

    const auto cmp = compare(referenceOut, candidateOut, 5.0e-4f);
    const std::string label = "DS-1 parity distortion=" + std::to_string(distortion) +
        " tone=" + std::to_string(tone) + " level=" + std::to_string(level) +
        " (maxDiff=" + std::to_string(cmp.maxAbsDifference) +
        ", maxRef=" + std::to_string(cmp.maxAbsReference) + ")";
    return require(cmp.ok && cmp.maxAbsReference > 1.0e-4f, label);
}

bool checkPreampParity(float bass, float treble) {
    circuit::PreampCircuit reference;
    if (!reference.prepare(sampleRate)) return require(false, "Preamp reference prepare()");
    reference.setControls(bass, treble);

    circuit::NetlistCircuit candidate;
    std::string error;
    if (!candidate.loadFromFile(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/preamp.json", &error))
        return require(false, "Preamp netlist load: " + error);
    if (!candidate.prepare(sampleRate, &error))
        return require(false, "Preamp netlist prepare(): " + error);
    candidate.setControl("bass", bass);
    candidate.setControl("treble", treble);

    // A guitar-level burst, long enough to carry both sides through the same
    // potentiometer ramp and coupling-capacitor transient before comparing.
    const auto settle = sineBurst(4000, 0.12f, 220.0);
    for (float x : settle) {
        reference.processSample(x);
        candidate.processSample(x);
    }

    const auto probe = sineBurst(2000, 0.12f, 220.0, 4000);
    std::vector<float> referenceOut(probe.size());
    std::vector<float> candidateOut(probe.size());
    for (std::size_t i = 0; i < probe.size(); ++i) {
        referenceOut[i] = reference.processSample(probe[i]);
        candidateOut[i] = candidate.processSample(probe[i]);
    }

    const auto cmp = compare(referenceOut, candidateOut, 5.0e-4f);
    const std::string label = "Preamp parity bass=" + std::to_string(bass) +
        " treble=" + std::to_string(treble) +
        " (maxDiff=" + std::to_string(cmp.maxAbsDifference) +
        ", maxRef=" + std::to_string(cmp.maxAbsReference) + ")";
    return require(cmp.ok && cmp.maxAbsReference > 1.0e-4f, label);
}

bool checkPowerAmpParity(float amplitude) {
    circuit::PowerAmpCircuit reference;
    if (!reference.prepare(sampleRate)) return require(false, "PowerAmp reference prepare()");

    circuit::NetlistCircuit candidate;
    std::string error;
    if (!candidate.loadFromFile(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/poweramp.json", &error))
        return require(false, "PowerAmp netlist load: " + error);
    if (!candidate.prepare(sampleRate, &error))
        return require(false, "PowerAmp netlist prepare(): " + error);

    // PowerAmpCircuit exposes no user controls (no pots), so the parity
    // sweep instead varies input drive level -- from a clean guitar-level
    // signal up through hard grid/plate clipping and output-transformer core
    // saturation -- to exercise both the pentode stamp's nonlinearity and the
    // per-sample magnetizing-inductance update identically on both sides.
    const auto settle = sineBurst(4000, amplitude, 220.0);
    for (float x : settle) {
        reference.processSample(x);
        candidate.processSample(x);
    }

    const auto probe = sineBurst(2000, amplitude, 220.0, 4000);
    std::vector<float> referenceOut(probe.size());
    std::vector<float> candidateOut(probe.size());
    for (std::size_t i = 0; i < probe.size(); ++i) {
        referenceOut[i] = reference.processSample(probe[i]);
        candidateOut[i] = candidate.processSample(probe[i]);
    }

    const auto cmp = compare(referenceOut, candidateOut, 5.0e-4f);
    const std::string label = "PowerAmp parity amplitude=" + std::to_string(amplitude) +
        " (maxDiff=" + std::to_string(cmp.maxAbsDifference) +
        ", maxRef=" + std::to_string(cmp.maxAbsReference) + ")";
    return require(cmp.ok && cmp.maxAbsReference > 1.0e-3f, label);
}

bool checkCompressorParity(float amplitude) {
    circuit::CompressorCircuit reference;
    if (!reference.prepare(sampleRate)) return require(false, "Compressor reference prepare()");

    circuit::NetlistCircuit candidate;
    std::string error;
    if (!candidate.loadFromFile(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/compressor.json", &error))
        return require(false, "Compressor netlist load: " + error);
    if (!candidate.prepare(sampleRate, &error))
        return require(false, "Compressor netlist prepare(): " + error);

    // CompressorCircuit exposes no user controls (a fixed LA-2A-style
    // feedback gain cell, like PowerAmpCircuit), so the parity sweep instead
    // varies input level -- from a quiet, uncompressed signal up through
    // levels that drive the LDR well into its lit range -- to exercise the
    // sidechain peak detector, diode rectifier and the per-sample LDR
    // resistance update identically on both sides.
    const auto settle = sineBurst(4000, amplitude, 220.0);
    for (float x : settle) {
        reference.processSample(x);
        candidate.processSample(x);
    }

    const auto probe = sineBurst(2000, amplitude, 220.0, 4000);
    std::vector<float> referenceOut(probe.size());
    std::vector<float> candidateOut(probe.size());
    for (std::size_t i = 0; i < probe.size(); ++i) {
        referenceOut[i] = reference.processSample(probe[i]);
        candidateOut[i] = candidate.processSample(probe[i]);
    }

    const auto cmp = compare(referenceOut, candidateOut, 5.0e-4f);
    const std::string label = "Compressor parity amplitude=" + std::to_string(amplitude) +
        " (maxDiff=" + std::to_string(cmp.maxAbsDifference) +
        ", maxRef=" + std::to_string(cmp.maxAbsReference) + ")";
    return require(cmp.ok, label);
}

} // namespace

int main() {
    bool ok = true;

    {
        circuit::NetlistCircuit malformed;
        std::string error;
        ok &= require(!malformed.loadFromJson("{not json", &error) && !error.empty(),
                      "netlist loader rejects malformed JSON");
    }
    {
        circuit::NetlistCircuit missingPort;
        std::string error;
        ok &= require(missingPort.loadFromJson(R"({"ops":[]})", &error), "netlist loader accepts empty ops");
        ok &= require(!missingPort.prepare(sampleRate, &error) && !error.empty(),
                      "netlist prepare() reports missing ports.input/output");
    }

    constexpr std::array<std::array<float, 3>, 5> settings{{
        {0.0f, 0.0f, 0.0f},
        {0.25f, 0.25f, 0.25f},
        {circuit::TS808Circuit::defaultDrive, circuit::TS808Circuit::defaultTone, circuit::TS808Circuit::defaultLevel},
        {0.75f, 0.75f, 0.75f},
        {1.0f, 1.0f, 1.0f},
    }};
    for (const auto& s : settings) ok &= checkTs808Parity(s[0], s[1], s[2]);

    constexpr std::array<std::array<float, 3>, 5> ds1Settings{{
        {0.0f, 0.0f, 0.0f},
        {0.25f, 0.25f, 0.25f},
        {circuit::DS1Circuit::defaultDistortion, circuit::DS1Circuit::defaultTone, circuit::DS1Circuit::defaultLevel},
        {0.75f, 0.75f, 0.75f},
        {1.0f, 1.0f, 1.0f},
    }};
    for (const auto& s : ds1Settings) ok &= checkDs1Parity(s[0], s[1], s[2]);

    // PreampCircuit::setBass/setTreble clamp a hairline away from the exact
    // 0.0/1.0 mechanical endpoints (see PreampCircuit::clampPotPosition), so
    // the parity sweep mirrors that clamped range rather than the raw [0,1]
    // NetlistCircuit::setControl() would otherwise apply literally.
    constexpr std::array<std::array<float, 2>, 5> preampSettings{{
        {0.01f, 0.01f},
        {0.25f, 0.25f},
        {circuit::PreampCircuit::defaultBass, circuit::PreampCircuit::defaultTreble},
        {0.75f, 0.75f},
        {0.99f, 0.99f},
    }};
    for (const auto& s : preampSettings) ok &= checkPreampParity(s[0], s[1]);

    // Sweeps input drive from clean guitar level up through hard grid/plate
    // clipping and output-transformer core saturation.
    constexpr std::array<float, 4> powerAmpAmplitudes{{0.05f, 0.3f, 1.0f, 3.0f}};
    for (float amplitude : powerAmpAmplitudes) ok &= checkPowerAmpParity(amplitude);

    // Sweeps input level from quiet/uncompressed up through levels that
    // drive the LDR well into its lit range.
    constexpr std::array<float, 4> compressorAmplitudes{{0.02f, 0.2f, 0.8f, 1.5f}};
    for (float amplitude : compressorAmplitudes) ok &= checkCompressorParity(amplitude);

    return ok ? 0 : 1;
}
