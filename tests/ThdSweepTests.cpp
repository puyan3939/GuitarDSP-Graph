// THD-vs-drive reference sweep for the component-level MNA circuits
// (issue #81, item 3: "TS808/DS-1/Preamp/PowerAmp等、異なる回路・異なる
// Drive設定でTHDがどう変化するか...記録できる仕組み"). This is a
// recording/reference harness rather than a strict characterization: it
// prints one DIAG line per (circuit, drive) point with the same
// hq::analyzeHarmonics() metrics the app's THD readout and the other
// circuit tests already use (see DS1TopologyTests.cpp, PreampCircuitTests.cpp,
// PowerAmpCircuitTests.cpp), so future circuit-design work has THD-vs-drive
// data to compare against. It asserts only the same weak invariant those
// existing tests already rely on: THD is materially higher at the hottest
// point of the sweep than at the quietest.
#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/PowerAmpCircuit.h"
#include "guitardsp/circuit/PreampCircuit.h"
#include "guitardsp/circuit/TS808Circuit.h"
#include "guitardsp/hq/Measurement.h"

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <vector>

using namespace guitardsp;

namespace {

bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kSampleRate = 48000.0;
constexpr int kSettleSamples = 4096;
constexpr int kMeasureSamples = 4096;

// Settles `circuit` (after `setup` applies the drive/distortion point for
// this sweep step) at a `toneHz` sine of `amplitude`, then returns the
// harmonic metrics of the following measure block, following the same
// settle-then-measure idiom as PreampCircuitTests.cpp/PowerAmpCircuitTests.cpp.
template <typename Circuit, typename SetupFn>
hq::HarmonicMetrics measureThd(Circuit& circuit, SetupFn&& setup, float amplitude, double toneHz) {
    circuit.reset();
    setup(circuit);
    for (int i = 0; i < kSettleSamples; ++i) {
        const float x = amplitude
            * static_cast<float>(std::sin(2.0 * kPi * toneHz * i / kSampleRate));
        circuit.processSample(x);
    }
    std::vector<float> output(static_cast<std::size_t>(kMeasureSamples));
    for (int i = 0; i < kMeasureSamples; ++i) {
        const float x = amplitude * static_cast<float>(
            std::sin(2.0 * kPi * toneHz * (kSettleSamples + i) / kSampleRate));
        output[static_cast<std::size_t>(i)] = circuit.processSample(x);
    }
    return hq::analyzeHarmonics(output, kSampleRate, toneHz, 10);
}

// Runs a THD-vs-`sweepLabel` sweep, prints a DIAG line per point, and
// returns the thdDb values in sweep order so the caller can check the
// quiet-vs-hot invariant.
template <typename Circuit, typename SetupFn>
std::vector<float> runSweep(const char* circuitName, const char* sweepLabel, Circuit& circuit,
                             std::initializer_list<float> sweepPoints, float amplitude,
                             double toneHz, SetupFn&& makeSetup) {
    std::vector<float> thdDb;
    for (float point : sweepPoints) {
        const auto metrics = measureThd(circuit, makeSetup(point), amplitude, toneHz);
        std::cout << "DIAG thd_sweep circuit=" << circuitName << ' ' << sweepLabel << '='
                  << point << " thd_pct=" << (100.0f * metrics.thd)
                  << " thd_db=" << metrics.thdDb << '\n';
        thdDb.push_back(metrics.thdDb);
    }
    return thdDb;
}

}  // namespace

int main() {
    bool ok = true;

    // TS808: sweep the physical drive pot at a fixed, representative
    // pickup-level input.
    {
        circuit::TS808Circuit ts808;
        ok &= require(ts808.prepare(kSampleRate), "TS808 prepares for THD sweep");
        const auto thdDb = runSweep("TS808", "drive", ts808, {0.0f, 0.25f, 0.5f, 0.75f, 1.0f},
                                     0.12f, 220.0,
                                     [](float drive) {
                                         return [drive](circuit::TS808Circuit& c) { c.setDrive(drive); };
                                     });
        for (float db : thdDb) ok &= require(std::isfinite(db), "TS808 THD sweep point is finite");
        ok &= require(thdDb.back() > thdDb.front(),
                      "TS808 THD is materially higher at max drive than min drive");
    }

    // DS-1: sweep the physical distortion pot the same way.
    {
        circuit::DS1Circuit ds1;
        ok &= require(ds1.prepare(kSampleRate), "DS-1 prepares for THD sweep");
        const auto thdDb = runSweep("DS1", "drive", ds1, {0.0f, 0.25f, 0.5f, 0.75f, 1.0f},
                                     0.12f, 220.0,
                                     [](float distortion) {
                                         return [distortion](circuit::DS1Circuit& c) {
                                             c.setDistortion(distortion);
                                         };
                                     });
        for (float db : thdDb) ok &= require(std::isfinite(db), "DS-1 THD sweep point is finite");
        ok &= require(thdDb.back() > thdDb.front(),
                      "DS-1 THD is materially higher at max drive than min drive");
    }

    // Preamp: this single 12AX7 gain stage has no physical drive pot, so
    // "drive" is input level -- the same idiom PreampCircuitTests.cpp uses.
    {
        circuit::PreampCircuit preamp;
        ok &= require(preamp.prepare(kSampleRate), "Preamp prepares for THD sweep");
        std::vector<float> thdDb;
        for (float amplitude : {0.05f, 0.15f, 0.30f, 0.45f, 0.60f}) {
            const auto metrics = measureThd(
                preamp, [](circuit::PreampCircuit&) {}, amplitude, 220.0);
            std::cout << "DIAG thd_sweep circuit=Preamp drive=" << amplitude
                      << " thd_pct=" << (100.0f * metrics.thd)
                      << " thd_db=" << metrics.thdDb << '\n';
            thdDb.push_back(metrics.thdDb);
        }
        for (float db : thdDb) ok &= require(std::isfinite(db), "Preamp THD sweep point is finite");
        ok &= require(thdDb.back() > thdDb.front(),
                      "Preamp THD is materially higher at hot input than quiet input");
    }

    // PowerAmp: same input-level idiom, at the power stage's much larger
    // (preamp/tone-stack-output-scale) input range.
    {
        circuit::PowerAmpCircuit powerAmp;
        ok &= require(powerAmp.prepare(kSampleRate), "PowerAmp prepares for THD sweep");
        std::vector<float> thdDb;
        for (float amplitude : {0.3f, 1.5f, 3.0f, 4.5f, 6.0f}) {
            const auto metrics = measureThd(
                powerAmp, [](circuit::PowerAmpCircuit&) {}, amplitude, 220.0);
            std::cout << "DIAG thd_sweep circuit=PowerAmp drive=" << amplitude
                      << " thd_pct=" << (100.0f * metrics.thd)
                      << " thd_db=" << metrics.thdDb << '\n';
            thdDb.push_back(metrics.thdDb);
        }
        for (float db : thdDb) ok &= require(std::isfinite(db), "PowerAmp THD sweep point is finite");
        ok &= require(thdDb.back() > thdDb.front(),
                      "PowerAmp THD is materially higher at hot input than quiet input");
    }

    std::cout << (ok ? "ALL PASS\n" : "FAILURES PRESENT\n");
    return ok ? 0 : 1;
}
