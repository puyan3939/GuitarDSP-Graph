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

// Loose on purpose -- see the file header comment above. Relative to each
// golden file's own RMS rather than one fixed absolute value: a flat
// absolute tolerance is both too loose to catch a regression in a quiet
// circuit's silence noise floor (e.g. PowerAmp's ~2.3e-5 RMS at silence is
// two orders of magnitude below any reasonable absolute floor) and
// needlessly tight relative to a loud signal's own scale. `kToleranceFloor`
// keeps the effective tolerance from collapsing toward zero for a
// near-silent file, where 0.5% of an already-tiny RMS would otherwise demand
// sub-float-precision agreement.
//
// Calibration note: on this environment's only available toolchain (GCC
// 13.3.0, x86-64, glibc), comparing this circuit/signal/param matrix built
// under the GOLDEN preset (-O2, no fast-math/native/LTO) against a plain
// `-DCMAKE_BUILD_TYPE=Release` build (default -O3, otherwise identical
// flags) produced bit-exact output on every one of the 50 files -- zero
// measured divergence, not just "under tolerance". `kToleranceRelative`
// therefore carries deliberate headroom above the only baseline actually
// measurable here, rather than being pinned to an observed nonzero value;
// see the issue #88 follow-up report for the raw comparison.
constexpr double kToleranceRelative = 5.0e-3;
constexpr double kToleranceFloor = 1.0e-5;

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

bool compareAgainstGolden(const std::string& goldenPath, const std::vector<float>& candidate,
                           const std::string& caseName, bool knownBad) {
    std::vector<float> reference;
    try {
        reference = readGoldenFile(goldenPath);
    } catch (const std::exception& e) {
        return knownBad || require(false, caseName + " (" + e.what() + ")");
    }

    if (reference.size() != candidate.size()) {
        std::cout << "  " << caseName << ": length mismatch golden=" << reference.size()
                  << " candidate=" << candidate.size() << (knownBad ? " (known-bad, not judged)" : "") << '\n';
        return knownBad || require(false, caseName);
    }

    double sumSquares = 0.0;
    for (float v : reference) sumSquares += static_cast<double>(v) * static_cast<double>(v);
    const double referenceRms = std::sqrt(sumSquares / static_cast<double>(reference.size()));
    const double tolerance = std::max(kToleranceFloor, kToleranceRelative * referenceRms);

    double maxAbsError = 0.0;
    std::optional<std::size_t> firstExceeding;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        const double diff = std::abs(static_cast<double>(reference[i]) - static_cast<double>(candidate[i]));
        maxAbsError = std::max(maxAbsError, diff);
        if (!firstExceeding && (diff > tolerance || !std::isfinite(candidate[i]))) {
            firstExceeding = i;
        }
    }

    if (firstExceeding || knownBad) {
        std::cout << "  " << caseName << ": first divergence at sample "
                  << (firstExceeding ? std::to_string(*firstExceeding) : std::string("none"))
                  << ", max_abs_error=" << maxAbsError << ", tolerance=" << tolerance
                  << (knownBad ? " (known-bad, not judged)" : "") << '\n';
    }
    if (knownBad) return true;
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

// MANIFEST.json's "knownBad" array (see tools/golden_gen's isKnownBad()) as
// a set of "tests/golden/<file>.txt" relative paths -- e.g. the TS808 "full"
// variant's documented Newton-divergence corner (docs/GOLDEN_REFERENCE.md's
// "Known caveat" section). Missing/absent entirely on a MANIFEST.json
// predating this field (harmless: an empty set judges every case normally).
std::set<std::string> loadKnownBad() {
    std::set<std::string> knownBad;
    circuit::JsonValue manifest;
    try {
        manifest = circuit::parseJson(readFile(std::string(GUITARDSP_GOLDEN_DATA_DIR) + "/MANIFEST.json"));
    } catch (const std::exception&) {
        return knownBad;
    }
    const circuit::JsonValue& entries = manifest["knownBad"];
    if (!entries.isArray()) return knownBad;
    for (const circuit::JsonValue& entry : entries.items()) knownBad.insert(entry.asString());
    return knownBad;
}

bool runCircuit(const std::string& circuitName, const std::set<std::string>& knownBad,
                 const std::function<std::vector<float>(const circuit::JsonValue&, const std::vector<float>&)>&
                     renderHandWritten,
                 const std::function<std::vector<float>(const circuit::JsonValue&, const std::vector<float>&)>&
                     renderNetlistOrEmpty) {
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
            const std::string fileName = circuitName + "_" + signal.name + "_" + variantName + ".txt";
            const std::string goldenPath = std::string(GUITARDSP_GOLDEN_DATA_DIR) + "/" + fileName;
            const bool isKnownBad = knownBad.count("tests/golden/" + fileName) != 0;

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
        "ts808", knownBad,
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
        });

    ok &= runCircuit(
        "ds1", knownBad,
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
        });

    ok &= runCircuit(
        "preamp", knownBad,
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderCircuit<circuit::PreampCircuit>(
                [&](circuit::PreampCircuit& c) { c.setControls(p["bass"].asFloat(), p["treble"].asFloat()); },
                input);
        },
        {});

    ok &= runCircuit(
        "fullamp", knownBad,
        [](const circuit::JsonValue& p, const std::vector<float>& input) {
            return renderCircuit<circuit::FullAmpCircuit>(
                [&](circuit::FullAmpCircuit& c) { c.setControls(p["bass"].asFloat(), p["treble"].asFloat()); },
                input);
        },
        {});

    ok &= runCircuit(
        "poweramp", knownBad,
        [](const circuit::JsonValue&, const std::vector<float>& input) {
            return renderCircuit<circuit::PowerAmpCircuit>([](circuit::PowerAmpCircuit&) {}, input);
        },
        {});

    ok &= runCircuit(
        "compressor", knownBad,
        [](const circuit::JsonValue&, const std::vector<float>& input) {
            return renderCircuit<circuit::CompressorCircuit>([](circuit::CompressorCircuit&) {}, input);
        },
        {});

    return ok ? 0 : 1;
}
