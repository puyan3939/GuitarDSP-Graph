#include "guitardsp/circuit/BjtEbersMollSubcircuit.h"
#include "guitardsp/circuit/DiodeParasiticSubcircuit.h"
#include "guitardsp/circuit/TS808Circuit.h"
#include "guitardsp/hq/TS808CircuitNode.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

hq::ResistorSpec resistor(float ohms) {
    hq::ResistorSpec r{};
    r.resistanceOhms = ohms;
    r.tolerancePercent = 0.0f;
    return r;
}

float diodeTransient(float junctionFarads) {
    circuit::MnaCircuitEngine c;
    const auto input = c.addNode();
    const auto node = c.addNode();
    c.addVoltageSource(input, circuit::ground, 0.10f);
    c.addResistor(input, node, resistor(100000.0f));
    auto diode = hq::component_presets::oneN4148();
    diode.junctionCapacitanceFarads = junctionFarads;
    circuit::addDiodeParasiticSubcircuit(c, node, circuit::ground, diode);
    c.prepare(48000.0);
    c.processSample(32, 1.0e-6f);
    return c.voltage(node);
}

double measureAcRms(circuit::TS808Circuit& ts, float level) {
    constexpr double sampleRate = 48000.0;
    constexpr double pi = 3.14159265358979323846;
    constexpr int measurementSamples = 4096;
    ts.setLevel(level);

    // Let the coupling capacitors and the newly rebuilt potentiometer matrix settle
    // before measuring. The actual level metric removes residual DC because the
    // pedal output is AC-coupled and gain should be judged on signal amplitude, not
    // on the power-up discharge offset of C9.
    for (int i = 0; i < 4096; ++i) {
        const float x = 0.12f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));
        ts.processSample(x);
    }

    double sum = 0.0;
    double sumSquares = 0.0;
    for (int i = 0; i < measurementSamples; ++i) {
        const float x = 0.12f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));
        const double y = static_cast<double>(ts.processSample(x));
        sum += y;
        sumSquares += y * y;
    }
    const double mean = sum / static_cast<double>(measurementSamples);
    const double variance = std::max(0.0,
        sumSquares / static_cast<double>(measurementSamples) - mean * mean);
    return std::sqrt(variance);
}
} // namespace

int main() {
    bool ok = true;

    {
        const float small = diodeTransient(2.0e-12f);
        const float large = diodeTransient(10.0e-9f);
        ok &= require(std::isfinite(small) && std::isfinite(large) && large < small * 0.5f,
                      "diode junction capacitance participates in the MNA transient");
    }

    {
        // Isolate the exact bias arrangement used by the TS808 emitter followers.
        // The supplies are source-stepped just like the complete pedal so the test
        // checks the physical operating point rather than an intentionally hostile
        // all-zero -> 9 V Newton jump.
        circuit::MnaCircuitEngine c;
        const auto supply = c.addNode();
        const auto vref = c.addNode();
        const auto base = c.addNode();
        const auto emitter = c.addNode();
        const auto supplySource = c.addVoltageSource(supply, circuit::ground, 0.0f);
        const auto vrefSource = c.addVoltageSource(vref, circuit::ground, 0.0f);
        c.addResistor(vref, base, resistor(510000.0f));
        c.addResistor(emitter, circuit::ground, resistor(10000.0f));

        auto transistor = hq::component_presets::twoN3904();
        transistor.name = "2SC1815-style diagnostic";
        transistor.beta = 350.0f;
        transistor.nominalVbe = 0.62f;
        transistor.saturationVoltage = 0.15f;
        transistor.maxCollectorVoltage = 50.0f;
        transistor.maxCollectorCurrentAmps = 0.15f;
        transistor.inputCapacitanceFarads = 8.0e-12f;
        const auto q = circuit::addBjtEbersMollSubcircuit(c, supply, base, emitter, transistor);

        ok &= require(c.prepare(48000.0), "isolated TS808 BJT buffer prepares");
        c.setNonlinearSolverMode(circuit::MnaCircuitEngine::NonlinearSolverMode::denseReference);
        circuit::MnaCircuitEngine::SolveStats stats{};
        for (int step = 1; step <= 128; ++step) {
            const float t = static_cast<float>(step) / 128.0f;
            c.setVoltageSource(supplySource, 9.0f * t);
            c.setVoltageSource(vrefSource, 4.5f * t);
            stats = c.processSample(40, 1.0e-7f);
            stats = c.processSample(40, 1.0e-7f);
        }
        const float vb = c.voltage(base);
        const float ve = c.voltage(emitter);
        const float iForward = c.currentThroughVoltageSource(q.forwardCurrentSense);
        const float iReverse = c.currentThroughVoltageSource(q.reverseCurrentSense);
        std::cout << "DIAG isolated-bjt vb=" << vb << " ve=" << ve
                  << " if=" << iForward << " ir=" << iReverse
                  << " iter=" << stats.iterations << " converged=" << stats.converged
                  << " singular=" << stats.singular << '\n';
        ok &= require(!stats.singular && stats.converged && std::isfinite(ve) &&
                      ve > 2.5f && ve < 4.5f,
                      "isolated TS808 Ebers-Moll BJT settles as emitter follower");
    }

    {
        circuit::TS808Circuit ts;
        ok &= require(ts.prepare(48000.0), "component-level TS808 prepares");
        ok &= require(ts.engine().sparseNonlinearSolverAvailable(),
                      "TS808 prepares a sparse nonlinear pattern");
        ts.engine().setNonlinearSolverMode(
            circuit::MnaCircuitEngine::NonlinearSolverMode::denseReference);

        for (int i = 0; i < 4096; ++i) ts.processSample(0.0f);

        // Diagnostic node order mirrors TS808Circuit's explicit schematic nodes.
        // This is temporary bring-up instrumentation and will be removed once the
        // remaining Newton-convergence rate is tightened.
        constexpr std::array<circuit::Node, 8> probeNodes{{6, 7, 11, 12, 16, 19, 21, 23}};
        constexpr std::array<const char*, 8> probeNames{{
            "q1e", "clip+", "clipout", "tone+", "toneout", "level", "q3e", "out"}};
        std::array<float, 8> probeMin{};
        std::array<float, 8> probeMax{};
        probeMin.fill(1.0e30f);
        probeMax.fill(-1.0e30f);

        constexpr double pi = 3.14159265358979323846;
        float minimum = 1.0e9f;
        float maximum = -1.0e9f;
        int unconverged = 0;
        bool healthy = true;
        for (int i = 0; i < 2048; ++i) {
            const float phase = static_cast<float>(2.0 * pi * 220.0 * i / 48000.0);
            const float input = 0.18f * std::sin(phase) + 0.03f * std::sin(3.0f * phase);
            const float output = ts.processSample(input);
            const auto stats = ts.lastSolveStats();
            healthy &= !stats.singular && std::isfinite(output);
            unconverged += stats.converged ? 0 : 1;
            minimum = std::min(minimum, output);
            maximum = std::max(maximum, output);
            for (std::size_t p = 0; p < probeNodes.size(); ++p) {
                const float value = ts.engine().voltage(probeNodes[p]);
                probeMin[p] = std::min(probeMin[p], value);
                probeMax[p] = std::max(probeMax[p], value);
            }
        }
        std::cout << "DIAG dense-ts808 min=" << minimum << " max=" << maximum
                  << " unconverged=" << unconverged
                  << " sparse=" << ts.engine().performanceStats().sparseNewtonSolves
                  << " fallback=" << ts.engine().performanceStats().sparseFallbackSolves
                  << " dense=" << ts.engine().performanceStats().generalLinearSolves << '\n';
        for (std::size_t p = 0; p < probeNodes.size(); ++p)
            std::cout << "DIAG stage " << probeNames[p] << " min=" << probeMin[p]
                      << " max=" << probeMax[p] << '\n';

        ok &= require(healthy, "TS808 circuit stays finite and nonsingular under guitar-level drive");
        ok &= require(maximum - minimum > 0.01f,
                      "TS808 circuit produces a driven AC output");
        ok &= require(minimum > -3.0f && maximum < 3.0f,
                      "TS808 AC-coupled output stays in a pedal-scale range");

        const double lowLevel = measureAcRms(ts, 0.15f);
        const double highLevel = measureAcRms(ts, 0.85f);
        std::cout << "DIAG ts808 level_ac_rms low=" << lowLevel << " high=" << highLevel << '\n';
        ok &= require(std::isfinite(lowLevel) && std::isfinite(highLevel) && highLevel > lowLevel * 1.5,
                      "TS808 level potentiometer changes actual AC output");
    }

    {
        circuit::TS808Circuit ts;
        constexpr double oversampledRate = 88200.0;
        ok &= require(ts.prepare(oversampledRate),
                      "full TS808 prepares at the Eco oversampled rate");
        ok &= require(ts.engine().dimension() == 48
                          && ts.engine().sparseNonlinearFactorNonZeros()
                              <= ts.engine().sparseNonlinearOriginalNonZeros() + 40
                          && ts.engine().sparseNonlinearCachedLinearUnknowns() >= 24,
                      "cached linear Schur prefix retains all 48 TS808 component unknowns");
        ts.engine().resetPerformanceStats();
        constexpr int samples = 1024;
        int totalIterations = 0;
        int unconverged = 0;
        for (int i = 0; i < samples; ++i) {
            const float phase = 2.0f * 3.14159265358979323846f * 220.0f
                * static_cast<float>(i) / static_cast<float>(oversampledRate);
            const float output = ts.processSample(0.15f * std::sin(phase));
            const auto solve = ts.lastSolveStats();
            totalIterations += solve.iterations;
            unconverged += solve.converged ? 0 : 1;
            ok &= std::isfinite(output) && !solve.singular;
        }
        const float averageIterations = static_cast<float>(totalIterations)
            / static_cast<float>(samples);
        const auto performance = ts.engine().performanceStats();
        std::cout << "DIAG optimized-ts808 average_iterations=" << averageIterations
                  << " unconverged=" << unconverged
                  << " sparse=" << performance.sparseNewtonSolves
                  << " fallback=" << performance.sparseFallbackSolves << '\n';
        ok &= require(averageIterations < 1.25f && unconverged == 0,
                      "matched residual keeps driven TS808 near one Newton solve per sample");
        ok &= require(performance.sampleRhsAssemblies == performance.samples
                          && performance.sparseNewtonSolves > 0,
                      "TS808 assembles sample history once while preserving full sparse MNA");

        // Compare the matched 20 ppm residual against the previous generic
        // 0.3 ppm residual using two otherwise identical component circuits.
        // This guards the optimization with an actual audio waveform comparison,
        // not only convergence counters.
        circuit::TS808Circuit strictResidual;
        circuit::TS808Circuit matchedResidual;
        ok &= require(strictResidual.prepare(oversampledRate)
                          && matchedResidual.prepare(oversampledRate),
                      "TS808 residual comparison circuits prepare");
        strictResidual.engine().setNonlinearResidualTolerance(3.0e-7f);
        matchedResidual.engine().setNonlinearResidualTolerance(2.0e-5f);
        for (int i = -4096; i < 0; ++i) {
            const float phase = 2.0f * 3.14159265358979323846f * 173.0f
                * static_cast<float>(i) / static_cast<float>(oversampledRate);
            const float input = 0.11f * std::sin(phase)
                + 0.027f * std::sin(phase * 2.71f);
            (void)strictResidual.processSample(input);
            (void)matchedResidual.processSample(input);
        }
        double squaredReference = 0.0;
        double squaredError = 0.0;
        float maximumError = 0.0f;
        constexpr int comparisonSamples = 16384;
        for (int i = 0; i < comparisonSamples; ++i) {
            const float phase = 2.0f * 3.14159265358979323846f * 173.0f
                * static_cast<float>(i) / static_cast<float>(oversampledRate);
            const float input = 0.11f * std::sin(phase)
                + 0.027f * std::sin(phase * 2.71f);
            const float reference = strictResidual.processSample(input);
            const float optimized = matchedResidual.processSample(input);
            const float error = optimized - reference;
            squaredReference += static_cast<double>(reference)
                * static_cast<double>(reference);
            squaredError += static_cast<double>(error) * static_cast<double>(error);
            maximumError = std::max(maximumError, std::abs(error));
        }
        const double relativeRmsError = std::sqrt(squaredError
            / std::max(1.0e-30, squaredReference));
        std::cout << "DIAG ts808-residual-match relative_rms_error="
                  << relativeRmsError << " maximum_error=" << maximumError << '\n';
        // Threshold measured against the analytic DC operating-point solve
        // (establishDcOperatingPoint(), which replaced a fixed-length silent
        // transient warm-up -- see prepare() above): both circuits still
        // start from an identical, deterministic operating point, but that
        // point itself sits at a different exact bit-pattern than the old
        // warm-up produced, and this comparison is inherently sensitive to
        // that starting bit-pattern over a long (20480-sample) AC-driven
        // run -- observed relative RMS error is ~4.2e-4 with either priming
        // approach's own equilibrium, not a monotonic function of DC-solve
        // precision (more homotopy steps measured slightly worse, not
        // better). The threshold below has headroom over the observed value
        // rather than pinning it exactly.
        ok &= require(relativeRmsError < 6.0e-4 && maximumError < 4.0e-4f,
                      "20 ppm TS808 residual matches strict-reference waveform");

        // Quiet input is the real hardware worst case: the high-gain op-amp can
        // alternate between adjacent float-sized operating points after KCL has
        // already converged. A voltage-delta-only test previously spent all 40
        // Newton steps on many silent samples and caused continuous audio XRUNs.
        ts.engine().resetPerformanceStats();
        constexpr int quietSamples = 8192;
        int quietIterations = 0;
        int quietUnconverged = 0;
        for (int i = 0; i < quietSamples; ++i) {
            const float phase = 2.0f * 3.14159265358979323846f * 220.0f
                * static_cast<float>(i) / static_cast<float>(oversampledRate);
            const float output = ts.processSample(0.0035f * std::sin(phase));
            const auto solve = ts.lastSolveStats();
            quietIterations += solve.iterations;
            quietUnconverged += solve.converged ? 0 : 1;
            ok &= std::isfinite(output) && !solve.singular;
        }
        const float averageQuietIterations = static_cast<float>(quietIterations)
            / static_cast<float>(quietSamples);
        const auto quietPerformance = ts.engine().performanceStats();
        std::cout << "DIAG quiet-ts808 average_iterations=" << averageQuietIterations
                  << " unconverged=" << quietUnconverged
                  << " residual_convergences="
                  << quietPerformance.nonlinearResidualConvergences << '\n';
        ok &= require(averageQuietIterations < 1.25f && quietUnconverged == 0,
                      "quiet guitar input converges without Newton-limit cycles");
        ok &= require(quietPerformance.nonlinearResidualConvergences > 0
                          && quietPerformance.sparseNewtonSolves
                              >= quietPerformance.samples,
                      "matched nonlinear KCL residual terminates physically converged samples");

        ts.engine().resetPerformanceStats();
        constexpr int silentSamples = 16384;
        int silentIterations = 0;
        int silentUnconverged = 0;
        for (int i = 0; i < silentSamples; ++i) {
            const float output = ts.processSample(0.0f);
            const auto solve = ts.lastSolveStats();
            silentIterations += solve.iterations;
            silentUnconverged += solve.converged ? 0 : 1;
            ok &= std::isfinite(output) && !solve.singular;
        }
        const auto silentPerformance = ts.engine().performanceStats();
        const float averageSilentIterations = static_cast<float>(silentIterations)
            / static_cast<float>(silentSamples);
        std::cout << "DIAG silent-ts808 average_iterations=" << averageSilentIterations
                  << " unconverged=" << silentUnconverged
                  << " cached_linear_unknowns="
                  << ts.engine().sparseNonlinearCachedLinearUnknowns()
                  << " static_rebuilds=" << silentPerformance.staticCacheRebuilds << '\n';
        ok &= require(averageSilentIterations < 1.25f && silentUnconverged == 0
                          && silentPerformance.staticCacheRebuilds == 0,
                      "prolonged silence preserves cached physical circuit operating point");

        ts.engine().resetPerformanceStats();
        ts.setControls(0.90f, 0.85f, 0.15f);
        for (int sample = 0; sample < 512; ++sample)
            ok &= std::isfinite(ts.processSample(0.02f));
        const auto gesture = ts.engine().performanceStats();
        ok &= require(std::abs(ts.appliedDrive() - 0.90f) < 1.0e-5f
                          && std::abs(ts.appliedTone() - 0.85f) < 1.0e-5f
                          && std::abs(ts.appliedLevel() - 0.15f) < 1.0e-5f
                          && gesture.staticCacheRebuilds < 90,
                      "ultrasonic-rate smoothed pots preserve exact endpoints without per-sample matrix rebuilds");
    }

    {
        hq::TS808CircuitNode node;
        graph::PrepareSpec spec{};
        spec.sampleRate = 48000.0;
        spec.maximumBlockSize = 64;
        spec.channels = 1;
        node.prepare(spec);
        ok &= require(node.prepared(), "TS808 circuit graph node prepares");

        graph::AudioBuffer input(1, 64);
        graph::AudioBuffer output(1, 64);
        for (int i = 0; i < 64; ++i)
            input.channel(0)[i] = 0.10f * std::sin(2.0f * 3.14159265358979323846f * 220.0f * i / 48000.0f);
        node.process(input, output, 64);
        bool finite = true;
        for (int i = 0; i < 64; ++i) finite &= std::isfinite(output.channel(0)[i]);
        ok &= require(finite, "TS808 circuit graph node processes finite audio");
    }

    return ok ? 0 : 1;
}
