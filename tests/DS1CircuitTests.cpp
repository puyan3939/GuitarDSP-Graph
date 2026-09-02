#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/hq/DS1CircuitNode.h"

#include <algorithm>
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
    return 0.14f * static_cast<float>(std::sin(phase)) +
           0.025f * static_cast<float>(std::sin(3.0 * phase));
}

double measureAcRms(circuit::DS1Circuit& ds, float level) {
    ds.setLevel(level);
    for (int i = 0; i < 4096; ++i) ds.processSample(guitarSample(i));

    constexpr int measurementSamples = 4096;
    double sum = 0.0;
    double sumSquares = 0.0;
    for (int i = 0; i < measurementSamples; ++i) {
        const double y = static_cast<double>(ds.processSample(guitarSample(i)));
        sum += y;
        sumSquares += y * y;
    }
    const double mean = sum / static_cast<double>(measurementSamples);
    const double variance = std::max(0.0,
        sumSquares / static_cast<double>(measurementSamples) - mean * mean);
    return std::sqrt(variance);
}

double measureDistortionDifference() {
    circuit::DS1Circuit low;
    circuit::DS1Circuit high;
    if (!low.prepare(sampleRate) || !high.prepare(sampleRate)) return 0.0;
    low.setControls(0.05f, 0.50f, 0.65f);
    high.setControls(0.95f, 0.50f, 0.65f);

    for (int i = 0; i < 4096; ++i) {
        const float x = guitarSample(i);
        low.processSample(x);
        high.processSample(x);
    }

    double sumSquares = 0.0;
    for (int i = 0; i < 2048; ++i) {
        const float x = guitarSample(i);
        const double delta = static_cast<double>(high.processSample(x)) -
                             static_cast<double>(low.processSample(x));
        sumSquares += delta * delta;
    }
    return std::sqrt(sumSquares / 2048.0);
}
} // namespace

int main() {
    bool ok = true;

    {
        circuit::DS1Circuit ds;
        ok &= require(ds.prepare(sampleRate), "component-level DS-1 prepares");
        ok &= require(ds.engine().sparseNonlinearSolverAvailable(),
                      "DS-1 prepares a sparse nonlinear pattern");

        const auto bias = ds.stageVoltages();
        std::cout << "DIAG ds1 bias q1e=" << bias.inputEmitter
                  << " q2b=" << bias.boosterBase
                  << " q2c=" << bias.boosterCollector
                  << " buffer=" << bias.opAmpBuffer
                  << " gain=" << bias.gainOutput
                  << " clip=" << bias.clippingNode
                  << " q3e=" << bias.outputEmitter
                  << " out=" << bias.output << '\n';
        ok &= require(std::isfinite(bias.inputEmitter) &&
                      std::isfinite(bias.boosterBase) &&
                      std::isfinite(bias.boosterCollector) &&
                      bias.inputEmitter > 2.0f && bias.inputEmitter < 4.5f &&
                      bias.boosterBase > 0.35f && bias.boosterBase < 1.5f &&
                      bias.boosterCollector > 0.5f && bias.boosterCollector < 8.5f,
                      "DS-1 transistor stages establish plausible DC operating points");

        float minimum = 1.0e9f;
        float maximum = -1.0e9f;
        bool healthy = true;
        int unconverged = 0;
        for (int i = 0; i < 4096; ++i) {
            const float y = ds.processSample(guitarSample(i));
            const auto stats = ds.lastSolveStats();
            healthy &= !stats.singular && std::isfinite(y);
            unconverged += stats.converged ? 0 : 1;
            minimum = std::min(minimum, y);
            maximum = std::max(maximum, y);
        }
        const auto perf = ds.engine().performanceStats();
        std::cout << "DIAG ds1 min=" << minimum << " max=" << maximum
                  << " unconverged=" << unconverged
                  << " sparse=" << perf.sparseNewtonSolves
                  << " fallback=" << perf.sparseFallbackSolves
                  << " dense=" << perf.generalLinearSolves << '\n';
        ok &= require(healthy, "DS-1 stays finite and nonsingular under guitar-level drive");
        ok &= require(maximum - minimum > 0.01f,
                      "DS-1 produces a driven AC output");
        ok &= require(minimum > -4.0f && maximum < 4.0f,
                      "DS-1 AC-coupled output stays in a pedal-scale range");

        const double lowLevel = measureAcRms(ds, 0.15f);
        const double highLevel = measureAcRms(ds, 0.85f);
        std::cout << "DIAG ds1 level_ac_rms low=" << lowLevel
                  << " high=" << highLevel << '\n';
        ok &= require(std::isfinite(lowLevel) && std::isfinite(highLevel) &&
                      highLevel > lowLevel * 1.5,
                      "DS-1 level potentiometer changes actual AC output");
    }

    {
        const double difference = measureDistortionDifference();
        std::cout << "DIAG ds1 distortion_difference_rms=" << difference << '\n';
        ok &= require(std::isfinite(difference) && difference > 1.0e-3,
                      "DS-1 distortion potentiometer changes the solved waveform");
    }

    {
        hq::DS1CircuitNode node;
        graph::PrepareSpec spec{};
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = 64;
        spec.channels = 1;
        node.prepare(spec);
        ok &= require(node.prepared(), "DS-1 circuit graph node prepares");

        graph::AudioBuffer input(1, 64);
        graph::AudioBuffer output(1, 64);
        for (int i = 0; i < 64; ++i) input.channel(0)[i] = guitarSample(i);
        node.process(input, output, 64);
        bool finite = true;
        for (int i = 0; i < 64; ++i) finite &= std::isfinite(output.channel(0)[i]);
        ok &= require(finite, "DS-1 circuit graph node processes finite audio");
    }

    {
        circuit::DS1Circuit ds;
        constexpr double oversampledRate = 96000.0;
        ok &= require(ds.prepare(oversampledRate) && ds.engine().dimension() == 57
                          && ds.engine().sparseNonlinearCachedLinearUnknowns() >= 25,
                      "Eco DS-1 retains all 57 component unknowns and its exact linear prefix");

        ds.engine().resetPerformanceStats();
        constexpr int quietSamples = 8192;
        int quietIterations = 0;
        int quietUnconverged = 0;
        for (int sample = 0; sample < quietSamples; ++sample) {
            ok &= std::isfinite(ds.processSample(0.0f));
            const auto solve = ds.lastSolveStats();
            quietIterations += solve.iterations;
            quietUnconverged += solve.converged ? 0 : 1;
        }
        const auto quiet = ds.engine().performanceStats();
        const float quietAverage = static_cast<float>(quietIterations)
            / static_cast<float>(quietSamples);
        std::cout << "DIAG quiet-ds1 average_iterations=" << quietAverage
                  << " unconverged=" << quietUnconverged
                  << " residual_convergences=" << quiet.nonlinearResidualConvergences
                  << '\n';
        ok &= require(quietAverage < 2.0f && quietUnconverged == 0
                          && quiet.nonlinearResidualConvergences > 0,
                      "matched 20 ppm DS-1 residual removes silent 40-step Newton limit cycles");

        ds.engine().resetPerformanceStats();
        constexpr int drivenSamples = 8192;
        int drivenIterations = 0;
        int drivenUnconverged = 0;
        for (int sample = 0; sample < drivenSamples; ++sample) {
            const float phase = static_cast<float>(sample) * 0.0143989663f;
            ok &= std::isfinite(ds.processSample(0.10f * std::sin(phase)));
            const auto solve = ds.lastSolveStats();
            drivenIterations += solve.iterations;
            drivenUnconverged += solve.converged ? 0 : 1;
        }
        const float drivenAverage = static_cast<float>(drivenIterations)
            / static_cast<float>(drivenSamples);
        std::cout << "DIAG driven-ds1 average_iterations=" << drivenAverage
                  << " unconverged=" << drivenUnconverged << '\n';
        ok &= require(drivenAverage < 3.5f && drivenUnconverged < drivenSamples / 50,
                      "driven component-level DS-1 converges quickly at its full 2x audio rate");
    }

    {
        // Regression for issue #14/#16: at DS-1's bright/high-gain setting
        // (distortion 0.95, tone 1.0) the fixed-damping Newton loop used to settle
        // into an exact repeating cycle on silent input instead of converging,
        // producing audible broadband hiss (~0.03 RMS sample-to-sample jitter).
        // The fix is now the engine-level backtracking line search in
        // MnaCircuitEngineCore.h (not a DS-1-specific patch), so this exercises
        // the same physical repro -- a loud passage decaying into silence -- and
        // checks the jitter it originally caught, without depending on how the
        // solver internally avoids it.
        circuit::DS1Circuit ds;
        ok &= require(ds.prepare(sampleRate), "bright DS-1 prepares");
        ds.setControls(0.95f, 1.0f, 0.55f);
        for (int i = 0; i < 4096; ++i) ds.processSample(0.8f * guitarSample(i));

        constexpr int silentSamples = 4096;
        double sumSquaredJitter = 0.0;
        bool healthy = true;
        int unconverged = 0;
        float previous = ds.processSample(0.0f);
        for (int i = 1; i < silentSamples; ++i) {
            const float y = ds.processSample(0.0f);
            const auto stats = ds.lastSolveStats();
            healthy &= !stats.singular && std::isfinite(y);
            unconverged += stats.converged ? 0 : 1;
            const double delta = static_cast<double>(y) - static_cast<double>(previous);
            sumSquaredJitter += delta * delta;
            previous = y;
        }
        const double jitterRms = std::sqrt(sumSquaredJitter / static_cast<double>(silentSamples - 1));
        std::cout << "DIAG bright-ds1-silence jitter_rms=" << jitterRms
                  << " unconverged=" << unconverged << '\n';
        ok &= require(healthy, "bright DS-1 stays finite and nonsingular decaying into silence");
        ok &= require(unconverged < silentSamples / 50,
                      "bright DS-1 Newton solve converges on nearly every silent sample");
        ok &= require(jitterRms < 0.01,
                      "bright DS-1 silent-input jitter stays well below the original ~0.03 RMS hiss");
    }

    return ok ? 0 : 1;
}
