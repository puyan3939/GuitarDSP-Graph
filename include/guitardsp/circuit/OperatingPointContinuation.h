#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

namespace guitardsp::circuit {

// Control-thread operating-point continuation for nonlinear audio circuits.
//
// This deliberately standardizes the robust startup procedure that the first
// component-level pedals had implemented privately: independent supply sources are
// ramped from zero, each continuation point is solved with the dense pivoting oracle,
// and the final operating point is allowed to settle until selected probe voltages
// stop moving. The resulting MNA solution and capacitor/inductor history become the
// initial state for realtime audio processing.
//
// Accuracy boundary: MnaCircuitEngine currently owns trapezoidal dynamic companion
// models at all times, so this is a time-domain/quasi-DC operating-point continuation,
// not yet a separate SPICE-style static matrix in which capacitors are opened and
// inductors are shorted analytically. It nevertheless produces the physically useful
// settled bias state required before the audio host starts and is intentionally named
// "Continuation" rather than claiming an exact standalone DC formulation.
struct OperatingPointSourceTarget {
    SourceHandle source{};
    float targetVolts = 0.0f;
};

struct OperatingPointOptions {
    int sourceSteps = 128;
    int solvesPerStep = 2;
    int maximumNewtonIterations = 40;
    float newtonTolerance = 1.0e-6f;
    int maximumSettleSamples = 8192;
    int requiredStableSamples = 32;
    float steadyStateVoltageTolerance = 2.0e-6f;
};

struct OperatingPointResult {
    bool converged = false;
    bool singular = false;
    int sourceStepSolves = 0;
    int settleSolves = 0;
    float maximumProbeDelta = 0.0f;
    MnaCircuitEngine::SolveStats lastSolve{};
};

inline OperatingPointResult establishOperatingPoint(
        MnaCircuitEngine& engine,
        std::span<const OperatingPointSourceTarget> sources,
        std::span<const Node> probeNodes,
        OperatingPointOptions options = {}) noexcept {
    OperatingPointResult result{};
    if (!engine.prepared()) {
        result.singular = true;
        return result;
    }

    options.sourceSteps = std::clamp(options.sourceSteps, 1, 4096);
    options.solvesPerStep = std::clamp(options.solvesPerStep, 1, 32);
    options.maximumNewtonIterations = std::clamp(options.maximumNewtonIterations, 1, 40);
    options.newtonTolerance = std::max(1.0e-9f, options.newtonTolerance);
    options.maximumSettleSamples = std::clamp(options.maximumSettleSamples, 0, 1'000'000);
    options.requiredStableSamples = std::clamp(options.requiredStableSamples, 1, 4096);
    options.steadyStateVoltageTolerance =
        std::max(1.0e-9f, options.steadyStateVoltageTolerance);

    const auto previousMode = engine.nonlinearSolverMode();
    engine.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);

    for (const auto& source : sources)
        if (!engine.setVoltageSource(source.source, 0.0f)) {
            result.singular = true;
            engine.setNonlinearSolverMode(previousMode);
            return result;
        }

    for (int step = 1; step <= options.sourceSteps; ++step) {
        const float t = static_cast<float>(step) / static_cast<float>(options.sourceSteps);
        for (const auto& source : sources)
            engine.setVoltageSource(source.source, source.targetVolts * t);

        for (int solve = 0; solve < options.solvesPerStep; ++solve) {
            result.lastSolve = engine.processSample(options.maximumNewtonIterations,
                                                    options.newtonTolerance);
            ++result.sourceStepSolves;
            if (result.lastSolve.singular) {
                result.singular = true;
                engine.setNonlinearSolverMode(previousMode);
                return result;
            }
        }
    }

    if (options.maximumSettleSamples == 0 || probeNodes.empty()) {
        result.converged = result.lastSolve.converged && !result.lastSolve.singular;
        engine.setNonlinearSolverMode(previousMode);
        return result;
    }

    std::vector<float> previousProbeVoltages(probeNodes.size(), 0.0f);
    for (std::size_t i = 0; i < probeNodes.size(); ++i)
        previousProbeVoltages[i] = engine.voltage(probeNodes[i]);

    int stableSamples = 0;
    for (int sample = 0; sample < options.maximumSettleSamples; ++sample) {
        result.lastSolve = engine.processSample(options.maximumNewtonIterations,
                                                options.newtonTolerance);
        ++result.settleSolves;
        if (result.lastSolve.singular) {
            result.singular = true;
            break;
        }

        float maximumDelta = 0.0f;
        for (std::size_t i = 0; i < probeNodes.size(); ++i) {
            const float voltage = engine.voltage(probeNodes[i]);
            if (!std::isfinite(voltage)) {
                result.singular = true;
                maximumDelta = 1.0e30f;
                break;
            }
            maximumDelta = std::max(maximumDelta,
                                    std::abs(voltage - previousProbeVoltages[i]));
            previousProbeVoltages[i] = voltage;
        }
        result.maximumProbeDelta = maximumDelta;
        if (result.singular) break;

        if (maximumDelta <= options.steadyStateVoltageTolerance && result.lastSolve.converged)
            ++stableSamples;
        else
            stableSamples = 0;

        if (stableSamples >= options.requiredStableSamples) {
            result.converged = true;
            break;
        }
    }

    engine.setNonlinearSolverMode(previousMode);
    return result;
}

} // namespace guitardsp::circuit
