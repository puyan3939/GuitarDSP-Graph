#include "guitardsp/circuit/DiodeParasiticSubcircuit.h"
#include "guitardsp/circuit/TS808Circuit.h"
#include "guitardsp/hq/TS808CircuitNode.h"

#include <algorithm>
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

double measureRms(circuit::TS808Circuit& ts, float level) {
    constexpr double sampleRate = 48000.0;
    constexpr double pi = 3.14159265358979323846;
    ts.setLevel(level);
    for (int i = 0; i < 2048; ++i) {
        const float x = 0.12f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));
        ts.processSample(x);
    }
    double sum = 0.0;
    for (int i = 0; i < 2048; ++i) {
        const float x = 0.12f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));
        const float y = ts.processSample(x);
        sum += static_cast<double>(y) * static_cast<double>(y);
    }
    return std::sqrt(sum / 2048.0);
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
        circuit::TS808Circuit ts;
        ok &= require(ts.prepare(48000.0), "component-level TS808 prepares");
        ok &= require(ts.engine().sparseNonlinearSolverAvailable(),
                      "TS808 prepares a sparse nonlinear pattern");

        // Dense partial-pivot mode is the numerical oracle while the first full
        // pedal is being brought up. If this also fails, the problem is the device/
        // topology model rather than fixed-pattern sparse factorization.
        ts.engine().setNonlinearSolverMode(
            circuit::MnaCircuitEngine::NonlinearSolverMode::denseReference);

        for (int i = 0; i < 4096; ++i) ts.processSample(0.0f);

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
        }
        std::cout << "DIAG dense-ts808 min=" << minimum << " max=" << maximum
                  << " unconverged=" << unconverged
                  << " sparse=" << ts.engine().performanceStats().sparseNewtonSolves
                  << " fallback=" << ts.engine().performanceStats().sparseFallbackSolves
                  << " dense=" << ts.engine().performanceStats().generalLinearSolves << '\n';
        ok &= require(healthy, "TS808 circuit stays finite and nonsingular under guitar-level drive");
        ok &= require(maximum - minimum > 0.01f,
                      "TS808 circuit produces a driven AC output");
        ok &= require(minimum > -3.0f && maximum < 3.0f,
                      "TS808 AC-coupled output stays in a pedal-scale range");

        const double lowLevel = measureRms(ts, 0.15f);
        const double highLevel = measureRms(ts, 0.85f);
        std::cout << "DIAG ts808 level_rms low=" << lowLevel << " high=" << highLevel << '\n';
        ok &= require(std::isfinite(lowLevel) && std::isfinite(highLevel) && highLevel > lowLevel * 1.5,
                      "TS808 level potentiometer changes actual circuit output");
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
