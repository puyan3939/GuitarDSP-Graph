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

    return ok ? 0 : 1;
}
