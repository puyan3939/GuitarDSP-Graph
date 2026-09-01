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
// Independent supply sources are ramped from zero with the dense pivoting oracle,
// then selected probe nodes are observed until their windowed DC means stop moving.
// Windowed means deliberately reject the tiny sample-to-sample trapezoidal ringing
// that can remain around an otherwise settled bias point.
//
// Accuracy boundary: MnaCircuitEngine currently owns trapezoidal dynamic companion
// models at all times, so this is a time-domain/quasi-DC operating-point continuation,
// not yet a separate SPICE-style static matrix in which capacitors are opened and
// inductors are shorted analytically. The distinction is explicit and tested.
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
    int settleWindowSamples = 32;
    int requiredStableWindows = 4;
    float steadyStateVoltageTolerance = 2.0e-6f;
};

struct OperatingPointResult {
    // `converged` means the observable DC probe means reached the requested stable
    // window without a singular/NaN solve. `lastSolve` separately preserves the
    // final per-sample Newton flag for diagnostics.
    bool converged = false;
    bool singular = false;
    int sourceStepSolves = 0;
    int settleSolves = 0;
    int unconvergedNewtonSolves = 0;
    float maximumProbeDelta = 0.0f;
    MnaCircuitEngine::SolveStats lastSolve{};
};

inline OperatingPointResult establishOperatingPoint(
        MnaCircuitEngine& engine,
        std::span<const OperatingPointSourceTarget> sources,
        std::span<const Node> probeNodes,
        OperatingPointOptions options = {}) {
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
    options.settleWindowSamples = std::clamp(options.settleWindowSamples, 1, 4096);
    options.requiredStableWindows = std::clamp(options.requiredStableWindows, 1, 4096);
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
            if (!result.lastSolve.converged) ++result.unconvergedNewtonSolves;
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

    std::vector<double> windowSums(probeNodes.size(), 0.0);
    std::vector<float> previousMeans(probeNodes.size(), 0.0f);
    bool havePreviousWindow = false;
    int samplesInWindow = 0;
    int stableWindows = 0;

    for (int sample = 0; sample < options.maximumSettleSamples; ++sample) {
        result.lastSolve = engine.processSample(options.maximumNewtonIterations,
                                                options.newtonTolerance);
        ++result.settleSolves;
        if (!result.lastSolve.converged) ++result.unconvergedNewtonSolves;
        if (result.lastSolve.singular) {
            result.singular = true;
            break;
        }

        for (std::size_t i = 0; i < probeNodes.size(); ++i) {
            const float voltage = engine.voltage(probeNodes[i]);
            if (!std::isfinite(voltage)) {
                result.singular = true;
                break;
            }
            windowSums[i] += static_cast<double>(voltage);
        }
        if (result.singular) break;

        ++samplesInWindow;
        if (samplesInWindow < options.settleWindowSamples) continue;

        float maximumMeanDelta = 0.0f;
        for (std::size_t i = 0; i < probeNodes.size(); ++i) {
            const float mean = static_cast<float>(windowSums[i] /
                static_cast<double>(options.settleWindowSamples));
            if (havePreviousWindow)
                maximumMeanDelta = std::max(maximumMeanDelta,
                                             std::abs(mean - previousMeans[i]));
            previousMeans[i] = mean;
            windowSums[i] = 0.0;
        }
        samplesInWindow = 0;
        result.maximumProbeDelta = maximumMeanDelta;

        if (!havePreviousWindow) {
            havePreviousWindow = true;
            continue;
        }

        if (maximumMeanDelta <= options.steadyStateVoltageTolerance)
            ++stableWindows;
        else
            stableWindows = 0;

        if (stableWindows >= options.requiredStableWindows) {
            result.converged = true;
            break;
        }
    }

    engine.setNonlinearSolverMode(previousMode);
    return result;
}

} // namespace guitardsp::circuit
