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

#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/NetlistLoader.h"
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

    return ok ? 0 : 1;
}
