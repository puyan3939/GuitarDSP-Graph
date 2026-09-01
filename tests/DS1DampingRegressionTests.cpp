// Regression coverage for the confirmed-cycle damping decay in
// MnaCircuitEngineCore::processSample (see issue #14 / #16). A fixed Newton
// damping schedule can turn the per-sample solve into a deterministic map
// with a locally repelling fixed point around DS-1's high-gain distortion
// stage: silent-input samples then settle into an exact repeating Newton
// iterate instead of converging, and the resulting per-sample retracing is
// audible as broadband hiss. Once a repeat fingerprint is confirmed twice,
// the engine now decays the damping every further iteration, which forces a
// shrinking-step sequence toward the true operating point instead.
//
// These tests drive each circuit hard and then cut to silence -- the exact
// scenario (note decay into silence) the hiss was reported in -- and check
// the sample-to-sample "jitter" (RMS of consecutive-sample differences)
// during the silence. A stuck Newton cycle shows up as large, sustained
// jitter; a converging solve shows up as small jitter that only reflects the
// circuit's own (slow, smooth) decay.
#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/TS808Circuit.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

constexpr double sampleRate = 48000.0;
constexpr double pi = 3.14159265358979323846;

float guitarSample(int index) {
    const double phase = 2.0 * pi * 220.0 * static_cast<double>(index) / sampleRate;
    return 0.30f * static_cast<float>(std::sin(phase)) +
           0.06f * static_cast<float>(std::sin(3.0 * phase));
}

// RMS of consecutive-sample differences over `measureSamples` silent samples,
// right after `drivenSamples` of guitarSample() drive. This isolates rapid
// sample-to-sample retracing from the circuit's own smooth release transient.
template <typename Circuit>
double silentJitterRms(Circuit& circuit, int drivenSamples, int measureSamples, bool* finiteOut) {
    for (int i = 0; i < drivenSamples; ++i) circuit.processSample(guitarSample(i));

    bool finite = true;
    float previous = circuit.processSample(0.0f);
    finite &= std::isfinite(previous);
    double sumSquares = 0.0;
    for (int i = 1; i < measureSamples; ++i) {
        const float y = circuit.processSample(0.0f);
        finite &= std::isfinite(y);
        const double diff = static_cast<double>(y) - static_cast<double>(previous);
        sumSquares += diff * diff;
        previous = y;
    }
    if (finiteOut) *finiteOut = finite;
    return std::sqrt(sumSquares / static_cast<double>(measureSamples - 1));
}
} // namespace

int main() {
    bool ok = true;

    {
        // High-gain, bright DS-1 setting (distortion and tone both near
        // maximum): before the damping-decay fix this setting settled into a
        // confirmed Newton cycle and measured ~0.03 RMS of sample-to-sample
        // jitter during silence. With the fix it measures ~0.0006-0.002,
        // roughly a 90%+ reduction. Guard against that regressing back up.
        circuit::DS1Circuit ds;
        ok &= require(ds.prepare(sampleRate), "DS-1 prepares for damping regression check");
        ds.setControls(/*distortion=*/0.95f, /*tone=*/1.0f, /*level=*/0.5f);

        bool finite = false;
        const double jitter = silentJitterRms(ds, 4096, 4000, &finite);
        std::cout << "DIAG ds1 damping_regression bright jitter_rms=" << jitter << '\n';
        ok &= require(finite, "DS-1 stays finite through drive-then-silence at bright/high-gain settings");
        ok &= require(jitter < 0.01, "DS-1 silent jitter stays low at bright/high-gain settings "
                                      "(guards the confirmed-cycle damping decay fix)");
    }

    {
        // TS808 shares the same Newton engine as DS-1. The fix's cycle
        // detector should never fire for it (it never revisits a prior
        // Newton state at these operating points), so its silent jitter
        // should stay in the same tiny range the fix was verified against.
        circuit::TS808Circuit ts;
        ok &= require(ts.prepare(sampleRate), "TS808 prepares for damping regression check");
        ts.setControls(/*drive=*/0.95f, /*tone=*/1.0f, /*level=*/0.5f);

        bool finite = false;
        const double jitter = silentJitterRms(ts, 4096, 4000, &finite);
        std::cout << "DIAG ts808 damping_regression jitter_rms=" << jitter << '\n';
        ok &= require(finite, "TS808 stays finite through drive-then-silence");
        ok &= require(jitter < 0.005, "TS808 silent jitter stays low "
                                       "(confirms the DS-1 damping fix left TS808 unaffected)");
    }

    return ok ? 0 : 1;
}
