#include "guitardsp/circuit/MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

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

struct LadderFixture {
    circuit::MnaCircuitEngine engine;
    circuit::SourceHandle source{};
    circuit::Node output{};
};

LadderFixture makeDiodeLadder(int stages) {
    LadderFixture fixture;
    const auto input = fixture.engine.addNode();
    fixture.source = fixture.engine.addVoltageSource(input, circuit::ground, 0.0f);

    auto previous = input;
    for (int i = 0; i < stages; ++i) {
        const auto node = fixture.engine.addNode();
        fixture.engine.addResistor(previous, node, resistor(1000.0f + 47.0f * static_cast<float>(i)));
        fixture.engine.addResistor(node, circuit::ground, resistor(22000.0f));
        fixture.engine.addDiode(node, circuit::ground, hq::component_presets::oneN4148());
        previous = node;
    }
    fixture.output = previous;
    return fixture;
}
} // namespace

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;

    auto dense = makeDiodeLadder(18);
    auto sparse = makeDiodeLadder(18);
    dense.engine.setNonlinearSolverMode(circuit::MnaCircuitEngine::NonlinearSolverMode::denseReference);
    sparse.engine.setNonlinearSolverMode(circuit::MnaCircuitEngine::NonlinearSolverMode::sparseFixedPattern);

    ok &= require(dense.engine.prepare(sampleRate), "dense nonlinear reference fixture prepares");
    ok &= require(sparse.engine.prepare(sampleRate), "fixed-pattern sparse nonlinear fixture prepares");
    ok &= require(sparse.engine.sparseNonlinearSolverAvailable(),
                  "nonlinear topology compiles a structural sparse solve pattern");
    ok &= require(sparse.engine.sparseNonlinearOriginalNonZeros() <
                  sparse.engine.dimension() * sparse.engine.dimension(),
                  "compiled nonlinear Jacobian is structurally sparse");
    ok &= require(sparse.engine.sparseNonlinearFactorNonZeros() >=
                  sparse.engine.sparseNonlinearOriginalNonZeros(),
                  "symbolic factor pattern includes required elimination fill");
    ok &= require(sparse.engine.sparseNonlinearFactorDensity() < 0.70f,
                  "representative nonlinear ladder remains sparse after symbolic fill");

    dense.engine.resetPerformanceStats();
    sparse.engine.resetPerformanceStats();
    float maxDifference = 0.0f;
    bool denseHealthy = true;
    bool sparseHealthy = true;
    for (int sample = 0; sample < 512; ++sample) {
        constexpr float pi = 3.14159265358979323846f;
        const float phase = 2.0f * pi * 220.0f * static_cast<float>(sample) /
                            static_cast<float>(sampleRate);
        const float input = 0.90f * std::sin(phase) + 0.25f * std::sin(3.0f * phase);
        dense.engine.setVoltageSource(dense.source, input);
        sparse.engine.setVoltageSource(sparse.source, input);
        const auto denseStats = dense.engine.processSample(32, 1.0e-7f);
        const auto sparseStats = sparse.engine.processSample(32, 1.0e-7f);
        denseHealthy &= !denseStats.singular;
        sparseHealthy &= !sparseStats.singular;
        const float denseOut = dense.engine.voltage(dense.output);
        const float sparseOut = sparse.engine.voltage(sparse.output);
        maxDifference = std::max(maxDifference, std::abs(denseOut - sparseOut));
    }

    const auto densePerf = dense.engine.performanceStats();
    const auto sparsePerf = sparse.engine.performanceStats();
    ok &= require(denseHealthy && sparseHealthy,
                  "dense and sparse nonlinear solvers stay nonsingular across driven transient");
    ok &= require(maxDifference < 2.0e-4f,
                  "fixed-pattern sparse Newton result tracks dense partial-pivot oracle");
    ok &= require(densePerf.generalLinearSolves > 0 && densePerf.sparseNewtonSolves == 0,
                  "dense reference mode remains available as numerical oracle");
    ok &= require(sparsePerf.sparseNewtonSolves > 0,
                  "forced sparse mode executes sparse Newton solves");
    ok &= require(sparsePerf.sparseNewtonSolves + sparsePerf.sparseFallbackSolves >= sparsePerf.samples,
                  "sparse backend accounts for every nonlinear sample through sparse solve or fallback");
    ok &= require(std::isfinite(sparse.engine.voltage(sparse.output)),
                  "sparse nonlinear output remains finite");

    return ok ? 0 : 1;
}
