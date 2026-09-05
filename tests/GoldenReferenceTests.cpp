// Regression test against the committed golden reference files (see
// docs/GOLDEN_REFERENCE.md). Unlike NetlistParityTests.cpp -- which checks
// that the hand-written circuit classes and their JSON netlist equivalents
// agree with *each other* at the current build's output -- this test checks
// that each hand-written circuit still agrees with the *fixed, committed*
// reference recorded under tests/golden/.
//
// This ctest target intentionally does not require bit-exact agreement:
// the golden files are only guaranteed bit-reproducible when built with the
// GOLDEN CMake preset (see CMakePresets.json and docs/GOLDEN_REFERENCE.md),
// and the default ctest build (this target's normal home) is not that
// preset. A different optimization level can shift the last bit or two of
// an intermediate Newton iterate, and because these are iterative nonlinear
// solves with per-sample state (bias caps, LDR envelope, etc.), such a
// difference can compound over a long signal instead of staying at LSB
// scale. So this test uses a loose tolerance intended to catch a broken or
// drifted circuit, not to catch build-flag-level noise. For true bit-exact
// verification, build with `cmake --preset golden`, run tools/golden_gen,
// and diff against tests/golden/ with tools/parity_check.
//
// For TS808/DS-1 this test also runs the JSON netlist equivalent
// (NetlistCircuit) against the same golden file. Since both the
// hand-written class and the netlist must independently agree with the
// fixed reference, they transitively agree with each other -- this is what
// replaces the old "bit-precision parity" purpose for those two circuits,
// now anchored to a fixed file instead of "whatever this build produces".

#include "guitardsp/circuit/CompressorCircuit.h"
#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/FullAmpCircuit.h"
#include "guitardsp/circuit/JsonValue.h"
#include "guitardsp/circuit/NetlistLoader.h"
#include "guitardsp/circuit/PowerAmpCircuit.h"
#include "guitardsp/circuit/PreampCircuit.h"
#include "guitardsp/circuit/TS808Circuit.h"

#include "golden/GoldenSignals.h"
#include "golden/HexFloat.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace guitardsp;
using golden::generateGoldenSignal;
using golden::goldenSignalList;

namespace {

#ifndef GUITARDSP_GOLDEN_PARAMS_DIR
#define GUITARDSP_GOLDEN_PARAMS_DIR "tests/golden/params"
#endif
#ifndef GUITARDSP_GOLDEN_DATA_DIR
#define GUITARDSP_GOLDEN_DATA_DIR "tests/golden"
#endif
#ifndef GUITARDSP_NETLIST_DATA_DIR
#define GUITARDSP_NETLIST_DATA_DIR "data/circuits"
#endif

// Loose on purpose -- see the file header comment above.
constexpr double kToleranceAbsolute = 2.0e-3;

bool require(bool condition, const std::string& name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

std::vector<float> readGoldenFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open golden file " + path);
    std::vector<float> values;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        float value = 0.0f;
        if (!golden::parseHexLine(line, &value)) {
            throw std::runtime_error("malformed golden line in " + path + ": " + line);
        }
        values.push_back(value);
    }
    return values;
}

// MANIFEST.json's "knownBad" list (see docs/GOLDEN_REFERENCE.md) names golden
// cases that are already known not to match -- e.g. ts808's `full` variant
// (drive = tone = level = 1.0) hits a documented Newton-solver divergence at
// that specific extreme parameter corner (issue #88), unrelated to the
// circuit's ordinary behavior. Loading it here lets this test still *report*
// those cases' actual max error (so a further regression is visible) without
// failing the suite on an already-accepted, tracked issue.
std::set<std::string> loadKnownBad() {
    std::set<std::string> knownBad;
    try {
        const circuit::JsonValue manifest =
            circuit::parseJson(readFile(std::string(GUITARDSP_GOLDEN_DATA_DIR) + "/MANIFEST.json"));
        for (const auto& entry : manifest["knownBad"].items()) knownBad.insert(entry.asString());
    } catch (const std::exception&) {
        // No MANIFEST.json / no knownBad field -- treat as an empty list.
    }
    return knownBad;
}

bool compareAgainstGolden(const std::string& goldenPath, const std::vector<float>& candidate,
                           const std::string& caseName, bool knownBad = false) {
    std::vector<float> reference;
    try {
        reference = readGoldenFile(goldenPath);
    } catch (const std::exception& e) {
        return require(false, caseName + " (" + e.what() + ")");
    }

    if (reference.size() != candidate.size()) {
        std::cout << "  " << caseName << ": length mismatch golden=" << reference.size()
                  << " candidate=" << candidate.size() << '\n';
        return knownBad || require(false, caseName);
    }

    double maxAbsError = 0.0;
    std::optional<std::size_t> firstExceeding;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        const double diff = std::abs(static_cast<double>(reference[i]) - static_cast<double>(candidate[i]));
        maxAbsError = std::max(maxAbsError, diff);
        if (!firstExceeding && (diff > kToleranceAbsolute || !std::isfinite(candidate[i]))) {
            firstExceeding = i;
        }
    }

    if (firstExceeding) {
        std::cout << "  " << caseName << ": first divergence at sample " << *firstExceeding
                  << ", max_abs_error=" << maxAbsError << '\n';
    }
    if (knownBad) {
        std::cout << (firstExceeding ? "KNOWNBAD " : "PASS(known-bad, ok) ") << caseName
                  << " max_abs_error=" << maxAbsError << '\n';
        return true;
    }
    return require(!firstExceeding.has_value(), caseName);
}

// Runs `input` through a freshly prepared+control-set circuit instance,
// mirroring how tools/golden_gen produced the committed files.
template <typename Circuit, typename SetControlsFn>
std::vector<float> renderCircuit(SetControlsFn setControls, const std::vector<float>& input) {
    Circuit circuit;
    if (!circuit.prepare(golden::kSampleRateHz)) return {};
    setControls(circuit);
    std::vector<float> output(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) output[i] = circuit.processSample(input[i]);
    return output;
}

std::vector<float> renderNetlist(const std::string& netlistPath,
                                  const std::vector<std::pair<std::string, float>>& controls,
                                  const std::vector<float>& input) {
    circuit::NetlistCircuit candidate;
    std::string error;
    if (!candidate.loadFromFile(netlistPath, &error)) return {};
    if (!candidate.prepare(golden::kSampleRateHz, &error)) return {};
    for (const auto& [name, value] : controls) candidate.setControl(name, value);
    std::vector<float> output(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) output[i] = candidate.processSample(input[i]);
    return output;
}

bool runCircuit(const std::string& circuitName,
                 const std::function<std::vector<float>(const circuit::JsonValue&, const std::vector<float>&)>&
                     renderHandWritten,
                 const std::function<std::vector<float>(const circuit::JsonValue&, const std::vector<float>&)>&
                     renderNetlistOrEmpty,
                 const std::set<std::string>& knownBad) {
    bool ok = true;
    const std::string paramsPath = std::string(GUITARDSP_GOLDEN_PARAMS_DIR) + "/" + circuitName + ".json";
    circuit::JsonValue params;
    try {
        params = circuit::parseJson(readFile(paramsPath));
    } catch (const std::exception& e) {
        return require(false, circuitName + " params load (" + e.what() + ")");
    }
    const circuit::JsonValue& variants = params["variants"];

    for (const auto& [variantName, variantParams] : variants.entries()) {
        for (const auto& signal : goldenSignalList()) {
            const std::vector<float> input = generateGoldenSignal(signal.name);
            // Matches the golden .txt filename (minus extension) so
            // MANIFEST.json's "knownBad" list can name entries directly
            // after the file they exempt.
            const std::string baseName = circuitName + "_" + signal.name + "_" + variantName;
            const std::string goldenPath = std::string(GUITARDSP_GOLDEN_DATA_DIR) + "/" + baseName + ".txt";
            const bool isKnownBad = knownBad.count(baseName) != 0;

            const std::vector<float> handWritten = renderHandWritten(variantParams, input);
            ok &= compareAgainstGolden(goldenPath, handWritten,
                                        circuitName + " " + variantName + " " + signal.name + " (hand-written)",
                                        isKnownBad);

            if (renderNetlistOrEmpty) {
                const std::vector<float> netlist = renderNetlistOrEmpty(variantParams, input);
                ok &= compareAgainstGolden(goldenPath, netlist,
                                            circuitName + " " + variantName + " " + signal.name + " (netlist)",
                                            isKnownBad);
            }
        }
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    const std::set<std::string> knownBad = loadKnownBad();

    ok &= runCircuit(
        "ts808",
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderCircuit<circuit::TS808Circuit>(
                [&](circuit::TS808Circuit& c) {
                    c.setControls(p["drive"].asFloat(), p["tone"].asFloat(), p["level"].asFloat());
                },
                input);
        },
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderNetlist(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/ts808.json",
                                  {{"drive", p["drive"].asFloat()},
                                   {"tone", p["tone"].asFloat()},
                                   {"level", p["level"].asFloat()}},
                                  input);
        },
        knownBad);

    ok &= runCircuit(
        "ds1",
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderCircuit<circuit::DS1Circuit>(
                [&](circuit::DS1Circuit& c) {
                    c.setControls(p["distortion"].asFloat(), p["tone"].asFloat(), p["level"].asFloat());
                },
                input);
        },
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderNetlist(std::string(GUITARDSP_NETLIST_DATA_DIR) + "/ds1.json",
                                  {{"distortion", p["distortion"].asFloat()},
                                   {"tone", p["tone"].asFloat()},
                                   {"level", p["level"].asFloat()}},
                                  input);
        },
        knownBad);

    ok &= runCircuit(
        "preamp",
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderCircuit<circuit::PreampCircuit>(
                [&](circuit::PreampCircuit& c) { c.setControls(p["bass"].asFloat(), p["treble"].asFloat()); },
                input);
        },
        {}, knownBad);

    ok &= runCircuit(
        "fullamp",
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderCircuit<circuit::FullAmpCircuit>(
                [&](circuit::FullAmpCircuit& c) { c.setControls(p["bass"].asFloat(), p["treble"].asFloat()); },
                input);
        },
        {}, knownBad);

    ok &= runCircuit(
        "poweramp",
        [](const circuit::JsonValue&, const std::vector<float>& input) {
            return renderCircuit<circuit::PowerAmpCircuit>([](circuit::PowerAmpCircuit&) {}, input);
        },
        {}, knownBad);

    ok &= runCircuit(
        "compressor",
        [](const circuit::JsonValue&, const std::vector<float>& input) {
            return renderCircuit<circuit::CompressorCircuit>([](circuit::CompressorCircuit&) {}, input);
        },
        {}, knownBad);

    return ok ? 0 : 1;
}
