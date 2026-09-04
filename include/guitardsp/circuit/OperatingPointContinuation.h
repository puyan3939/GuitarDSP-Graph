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
// Two related but distinct mechanisms live in this file:
//
// - establishOperatingPoint() ramps independent supply sources from zero under
//   the *trapezoidal* dynamic companion model (MnaCircuitEngine's normal
//   real-time stamps) and then watches selected probe nodes' windowed DC means
//   until they stop moving. This is a time-domain/quasi-DC continuation, not an
//   analytic DC solve -- it is still watching an exponentially decaying
//   transient settle, just through a windowed mean that rejects trapezoidal
//   ringing. It remains useful when a caller genuinely wants to observe
//   settling in the time domain (e.g. AudioReadinessTests.cpp).
//
// - establishDcOperatingPoint() instead ramps supply sources under
//   MnaCircuitEngine::setOperatingPointMode(true) -- the engine's analytic
//   SPICE-style .OP boundary condition (capacitors opened, inductors shorted,
//   no trapezoidal history term at all). Because that system has no memory,
//   repeated solves at a fixed source value converge to a genuine algebraic
//   fixed point in a handful of Newton iterations; there is no settling time
//   constant to wait out, and therefore no settle-window/hard-sample-cap
//   heuristic here at all. This is what should be used to prime a circuit's
//   capacitor/inductor state before real-time processing begins.
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

struct DcOperatingPointOptions {
    int sourceSteps = 128;
    int solvesPerStep = 2;
    int maximumNewtonIterations = 40;
    float newtonTolerance = 1.0e-6f;
};

struct DcOperatingPointResult {
    // `converged` reflects the last homotopy-step solve's Newton flag under the
    // analytic DC boundary condition. Because that system has no history term,
    // this genuinely means the algebraic operating-point equations are solved
    // to `newtonTolerance` -- not merely that a time-domain probe stopped
    // moving within some window.
    bool converged = false;
    bool singular = false;
    int sourceStepSolves = 0;
    MnaCircuitEngine::SolveStats lastSolve{};
};

// Establishes a circuit's DC operating point analytically (SPICE .OP style:
// capacitors open, inductors shorted, no trapezoidal history) and leaves the
// engine's capacitor/inductor state variables initialized at that equilibrium,
// ready for real-time trapezoidal processing to continue from rather than from
// a cold, all-zero start.
//
// Source stepping is reused here as a homotopy continuation exactly as
// establishOperatingPoint() uses it: each solved step becomes the Newton
// initial guess for the next, slightly higher supply voltage, so the solver
// never has to jump from an all-zero guess directly to a fully biased
// nonlinear circuit. Unlike establishOperatingPoint(), there is no settling
// loop afterward -- the DC system has no memory, so the homotopy's own
// convergence at the final (full-voltage) step *is* the operating point.
//
// `sources` should cover every independent supply/reference rail the circuit
// needs primed (e.g. a 9 V supply and a 4.5 V virtual-ground reference); any
// audio input source should already be pinned to 0 V by the caller (true by
// construction immediately after MnaCircuitEngine::prepare(), since a freshly
// added voltage source defaults to 0 V and prepare()'s reset() does not touch
// source voltages).
//
// Control-thread only, like establishOperatingPoint() -- never call this from
// the audio callback path.
inline DcOperatingPointResult establishDcOperatingPoint(
        MnaCircuitEngine& engine,
        std::span<const OperatingPointSourceTarget> sources,
        DcOperatingPointOptions options = {}) {
    DcOperatingPointResult result{};
    if (!engine.prepared()) {
        result.singular = true;
        return result;
    }

    options.sourceSteps = std::clamp(options.sourceSteps, 1, 4096);
    options.solvesPerStep = std::clamp(options.solvesPerStep, 1, 32);
    options.maximumNewtonIterations = std::clamp(options.maximumNewtonIterations, 1, 40);
    options.newtonTolerance = std::max(1.0e-9f, options.newtonTolerance);

    const auto previousMode = engine.nonlinearSolverMode();
    engine.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
    engine.setOperatingPointMode(true);

    for (const auto& source : sources)
        if (!engine.setVoltageSource(source.source, 0.0f)) {
            result.singular = true;
            engine.setOperatingPointMode(false);
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
                break;
            }
        }
        if (result.singular) break;
    }

    result.converged = !result.singular && result.lastSolve.converged;

    // Leave the engine in its normal trapezoidal mode: the next static-cache
    // rebuild (triggered lazily by the dirty flag setOperatingPointMode(false)
    // sets) restores real dt-based companion coefficients while the
    // capacitor/inductor previousVoltage_/previousCurrent_ this DC solve just
    // wrote via updateDynamicState() carry over unchanged -- exactly the
    // equilibrium initial condition real-time processing should continue from.
    engine.setOperatingPointMode(false);
    engine.setNonlinearSolverMode(previousMode);
    return result;
}

} // namespace guitardsp::circuit
