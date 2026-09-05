#pragma once

#include "FixedPatternSparseSolver.h"
#include "guitardsp/hq/ComponentCatalog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace guitardsp::circuit {

using Node = std::uint16_t;
using SourceHandle = std::uint16_t;
using ResistorHandle = std::uint16_t;
using CapacitorHandle = std::uint16_t;
using InductorHandle = std::uint16_t;
using PotHandle = std::uint16_t;
using DiodeHandle = std::uint16_t;
using BjtHandle = std::uint16_t;
using JfetHandle = std::uint16_t;
using MosfetHandle = std::uint16_t;
using OpAmpHandle = std::uint16_t;
using TriodeHandle = std::uint16_t;
using PentodeHandle = std::uint16_t;
using ControlledSourceHandle = std::uint16_t;
inline constexpr Node ground = 0;

// MNA reference engine with cached linear execution and a prepared fixed-pattern
// sparse backend for nonlinear Newton systems. Dense partial-pivot solving remains
// the numerical oracle/fallback whenever the sparse no-pivot factorization meets
// an unsafe numerical pivot.
class MnaCircuitEngine {
public:
    enum class NonlinearSolverMode : std::uint8_t {
        automatic,
        denseReference,
        sparseFixedPattern
    };

    struct SolveStats {
        int iterations = 0;
        bool converged = true;
        bool singular = false;
    };

    struct PerformanceStats {
        std::uint64_t samples = 0;
        std::uint64_t staticCacheRebuilds = 0;
        std::uint64_t sampleRhsAssemblies = 0;
        std::uint64_t nonlinearAssemblies = 0;
        std::uint64_t fullFactorizations = 0;
        std::uint64_t cachedLinearSolves = 0;
        std::uint64_t generalLinearSolves = 0;
        std::uint64_t sparseNewtonSolves = 0;
        std::uint64_t sparseFallbackSolves = 0;
        std::uint64_t nonlinearResidualConvergences = 0;
    };

    Node addNode() {
        prepared_ = false;
        return static_cast<Node>(++nodeCount_);
    }

    ResistorHandle addResistor(Node a, Node b, hq::ResistorSpec spec) {
        prepared_ = false;
        resistors_.push_back({a, b, spec});
        return static_cast<ResistorHandle>(resistors_.size() - 1U);
    }

    CapacitorHandle addCapacitor(Node a, Node b, hq::CapacitorSpec spec) {
        prepared_ = false;
        capacitors_.push_back({a, b, spec, 0.0f, 0.0f, 0.0f});
        return static_cast<CapacitorHandle>(capacitors_.size() - 1U);
    }

    InductorHandle addInductor(Node a, Node b, hq::InductorSpec spec) {
        prepared_ = false;
        inductors_.push_back({a, b, spec, 0.0f, 0.0f, 0, 0.0f});
        return static_cast<InductorHandle>(inductors_.size() - 1U);
    }

    PotHandle addPotentiometer(Node high, Node wiper, Node low, hq::PotentiometerSpec spec) {
        prepared_ = false;
        potentiometers_.push_back({high, wiper, low, spec});
        return static_cast<PotHandle>(potentiometers_.size() - 1U);
    }

    void addCurrentSource(Node positive, Node negative, float amps) {
        prepared_ = false;
        currentSources_.push_back({positive, negative, amps});
    }

    SourceHandle addVoltageSource(Node positive, Node negative, float volts = 0.0f) {
        prepared_ = false;
        voltageSources_.push_back({positive, negative, volts, 0});
        return static_cast<SourceHandle>(voltageSources_.size() - 1U);
    }

    ControlledSourceHandle addVccs(Node outputPositive, Node outputNegative,
                                   Node controlPositive, Node controlNegative,
                                   float transconductanceSiemens) {
        prepared_ = false;
        vccs_.push_back({outputPositive, outputNegative, controlPositive, controlNegative,
                         transconductanceSiemens});
        return static_cast<ControlledSourceHandle>(vccs_.size() - 1U);
    }

    ControlledSourceHandle addVcvs(Node outputPositive, Node outputNegative,
                                   Node controlPositive, Node controlNegative,
                                   float voltageGain) {
        prepared_ = false;
        vcvs_.push_back({outputPositive, outputNegative, controlPositive, controlNegative,
                         voltageGain, 0});
        return static_cast<ControlledSourceHandle>(vcvs_.size() - 1U);
    }

    ControlledSourceHandle addCccs(Node outputPositive, Node outputNegative,
                                   SourceHandle controlVoltageSource, float currentGain) {
        prepared_ = false;
        cccs_.push_back({outputPositive, outputNegative, controlVoltageSource, currentGain});
        return static_cast<ControlledSourceHandle>(cccs_.size() - 1U);
    }

    ControlledSourceHandle addCcvs(Node outputPositive, Node outputNegative,
                                   SourceHandle controlVoltageSource, float transresistanceOhms) {
        prepared_ = false;
        ccvs_.push_back({outputPositive, outputNegative, controlVoltageSource,
                         transresistanceOhms, 0});
        return static_cast<ControlledSourceHandle>(ccvs_.size() - 1U);
    }

    DiodeHandle addDiode(Node anode, Node cathode, hq::DiodeSpec spec) {
        prepared_ = false;
        diodes_.push_back({anode, cathode, spec});
        return static_cast<DiodeHandle>(diodes_.size() - 1U);
    }

    BjtHandle addBjt(Node collector, Node base, Node emitter, hq::BJTSpec spec) {
        prepared_ = false;
        bjts_.push_back({collector, base, emitter, spec});
        return static_cast<BjtHandle>(bjts_.size() - 1U);
    }

    JfetHandle addJfet(Node drain, Node gate, Node source, hq::JFETSpec spec) {
        prepared_ = false;
        jfets_.push_back({drain, gate, source, spec});
        return static_cast<JfetHandle>(jfets_.size() - 1U);
    }

    MosfetHandle addMosfet(Node drain, Node gate, Node source, hq::MOSFETSpec spec) {
        prepared_ = false;
        mosfets_.push_back({drain, gate, source, spec});
        return static_cast<MosfetHandle>(mosfets_.size() - 1U);
    }

    OpAmpHandle addOpAmp(Node output, Node nonInverting, Node inverting,
                         Node reference, hq::OpAmpSpec spec) {
        prepared_ = false;
        opAmps_.push_back({output, nonInverting, inverting, reference, spec, 0});
        return static_cast<OpAmpHandle>(opAmps_.size() - 1U);
    }

    TriodeHandle addTriode(Node plate, Node grid, Node cathode, hq::TriodeSpec spec) {
        prepared_ = false;
        triodes_.push_back({plate, grid, cathode, spec});
        return static_cast<TriodeHandle>(triodes_.size() - 1U);
    }

    PentodeHandle addPentode(Node plate, Node grid, Node screen, Node cathode, hq::PentodeSpec spec) {
        prepared_ = false;
        pentodes_.push_back({plate, grid, screen, cathode, spec});
        return static_cast<PentodeHandle>(pentodes_.size() - 1U);
    }

    bool setVoltageSource(SourceHandle handle, float volts) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= voltageSources_.size()) return false;
        voltageSources_[i].volts = volts;
        return true;
    }

    float currentThroughVoltageSource(SourceHandle handle) const noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= voltageSources_.size()) return 0.0f;
        const auto branch = voltageSources_[i].branchIndex;
        return branch < solution_.size() ? solution_[branch] : 0.0f;
    }

    bool setResistorSpec(ResistorHandle handle, hq::ResistorSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= resistors_.size()) return false;
        resistors_[i].spec = spec;
        staticCacheDirty_ = true;
        return true;
    }

    bool setResistance(ResistorHandle handle, float ohms) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= resistors_.size()) return false;
        resistors_[i].spec.resistanceOhms = std::max(1.0e-6f, ohms);
        staticCacheDirty_ = true;
        return true;
    }

    bool setCapacitorSpec(CapacitorHandle handle, hq::CapacitorSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= capacitors_.size()) return false;
        capacitors_[i].spec = spec;
        staticCacheDirty_ = true;
        return true;
    }

    bool setCapacitance(CapacitorHandle handle, float farads) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= capacitors_.size()) return false;
        capacitors_[i].spec.capacitanceFarads = std::max(0.0f, farads);
        staticCacheDirty_ = true;
        return true;
    }

    bool setInductorSpec(InductorHandle handle, hq::InductorSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= inductors_.size()) return false;
        inductors_[i].spec = spec;
        staticCacheDirty_ = true;
        return true;
    }

    bool setInductance(InductorHandle handle, float henries) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= inductors_.size()) return false;
        inductors_[i].spec.inductanceHenries = std::max(0.0f, henries);
        staticCacheDirty_ = true;
        return true;
    }

    bool setPotentiometerPosition(PotHandle handle, float position) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= potentiometers_.size()) return false;
        potentiometers_[i].spec.position = std::clamp(position, 0.0f, 1.0f);
        staticCacheDirty_ = true;
        return true;
    }

    bool setPotentiometerSpec(PotHandle handle, hq::PotentiometerSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= potentiometers_.size()) return false;
        potentiometers_[i].spec = spec;
        staticCacheDirty_ = true;
        return true;
    }

    bool setVccsTransconductance(ControlledSourceHandle handle, float siemens) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= vccs_.size()) return false;
        vccs_[i].transconductance = siemens;
        staticCacheDirty_ = true;
        return true;
    }

    bool setVcvsGain(ControlledSourceHandle handle, float gain) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= vcvs_.size()) return false;
        vcvs_[i].gain = gain;
        staticCacheDirty_ = true;
        return true;
    }

    bool setCccsGain(ControlledSourceHandle handle, float gain) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= cccs_.size()) return false;
        cccs_[i].gain = gain;
        staticCacheDirty_ = true;
        return true;
    }

    bool setCcvsTransresistance(ControlledSourceHandle handle, float ohms) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= ccvs_.size()) return false;
        ccvs_[i].transresistanceOhms = ohms;
        staticCacheDirty_ = true;
        return true;
    }

    bool setDiodeSpec(DiodeHandle handle, hq::DiodeSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= diodes_.size()) return false;
        diodes_[i].spec = spec;
        diodes_[i].inverseThermalVoltage = 1.0f / std::max(1.0e-6f,
            spec.emissionCoefficient * spec.thermalVoltage);
        return true;
    }

    bool setBjtSpec(BjtHandle handle, hq::BJTSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= bjts_.size()) return false;
        bjts_[i].spec = spec;
        return true;
    }

    bool setJfetSpec(JfetHandle handle, hq::JFETSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= jfets_.size()) return false;
        jfets_[i].spec = spec;
        return true;
    }

    bool setMosfetSpec(MosfetHandle handle, hq::MOSFETSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= mosfets_.size()) return false;
        mosfets_[i].spec = spec;
        return true;
    }

    bool setOpAmpSpec(OpAmpHandle handle, hq::OpAmpSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= opAmps_.size()) return false;
        opAmps_[i].spec = spec;
        staticCacheDirty_ = true;
        return true;
    }

    bool setTriodeSpec(TriodeHandle handle, hq::TriodeSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= triodes_.size()) return false;
        triodes_[i].spec = spec;
        return true;
    }

    bool setPentodeSpec(PentodeHandle handle, hq::PentodeSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= pentodes_.size()) return false;
        pentodes_[i].spec = spec;
        return true;
    }

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        const std::size_t nodeUnknowns = nodeCount_;
        std::size_t branch = nodeUnknowns;
        for (auto& source : voltageSources_) source.branchIndex = branch++;
        for (auto& source : vcvs_) source.branchIndex = branch++;
        for (auto& source : ccvs_) source.branchIndex = branch++;
        for (auto& opAmp : opAmps_) opAmp.branchIndex = branch++;
        for (auto& inductor : inductors_) inductor.branchIndex = branch++;
        for (auto& diode : diodes_)
            diode.inverseThermalVoltage = 1.0f / std::max(1.0e-6f,
                diode.spec.emissionCoefficient * diode.spec.thermalVoltage);
        dimension_ = branch;
        if (dimension_ == 0U) return false;

        // Node topology is fixed from here on for this prepare() call, so every
        // nonlinear device's stamp write destinations can be resolved once
        // instead of on every Newton iteration (see CompiledCurrentStamp above).
        for (auto& d : diodes_)
            d.conductanceLayout = compileConductanceStamp(dimension_, d.anode, d.cathode);
        for (auto& bjt : bjts_) {
            bjt.collectorLayout = compileCurrentStamp<3>(dimension_, bjt.collector, bjt.emitter,
                std::array<Node, 3>{bjt.collector, bjt.base, bjt.emitter});
            bjt.baseLayout = compileCurrentStamp<3>(dimension_, bjt.base, bjt.emitter,
                std::array<Node, 3>{bjt.collector, bjt.base, bjt.emitter});
        }
        for (auto& jfet : jfets_)
            jfet.layout = compileCurrentStamp<3>(dimension_, jfet.drain, jfet.source,
                std::array<Node, 3>{jfet.drain, jfet.gate, jfet.source});
        for (auto& mosfet : mosfets_)
            mosfet.layout = compileCurrentStamp<3>(dimension_, mosfet.drain, mosfet.source,
                std::array<Node, 3>{mosfet.drain, mosfet.gate, mosfet.source});
        for (auto& triode : triodes_)
            triode.layout = compileCurrentStamp<3>(dimension_, triode.plate, triode.cathode,
                std::array<Node, 3>{triode.plate, triode.grid, triode.cathode});
        for (auto& pentode : pentodes_) {
            pentode.plateLayout = compileCurrentStamp<4>(dimension_, pentode.plate, pentode.cathode,
                std::array<Node, 4>{pentode.plate, pentode.grid, pentode.screen, pentode.cathode});
            pentode.screenLayout = compileCurrentStamp<3>(dimension_, pentode.screen, pentode.cathode,
                std::array<Node, 3>{pentode.grid, pentode.screen, pentode.cathode});
        }

        const std::size_t matrixSize = dimension_ * dimension_;
        staticMatrix_.assign(matrixSize, 0.0f);
        staticRhs_.assign(dimension_, 0.0f);
        sampleRhs_.assign(dimension_, 0.0f);
        matrix_.assign(matrixSize, 0.0f);
        rhs_.assign(dimension_, 0.0f);
        solution_.assign(dimension_, 0.0f);
        candidate_.assign(dimension_, 0.0f);
        lineSearchCandidate_.assign(dimension_, 0.0f);
        previousSampleSolution_.assign(dimension_, 0.0f);
        workMatrix_.assign(matrixSize, 0.0f);
        workRhs_.assign(dimension_, 0.0f);
        linearLu_.assign(matrixSize, 0.0f);
        linearPivots_.assign(dimension_, 0U);

        // Nodes directly constrained by an independent source to ground do not need
        // Newton voltage limiting: their exact value is known from the linear MNA
        // equations. Exempting them allows a 9 V pedal supply or 250 V tube supply
        // to land immediately while nonlinear internal nodes remain trust-limited.
        fixedVoltageNodes_.assign(nodeCount_, 0U);
        for (const auto& source : voltageSources_) {
            if (source.negative == ground && source.positive != ground)
                fixedVoltageNodes_[nodeIndex(source.positive)] = 1U;
            else if (source.positive == ground && source.negative != ground)
                fixedVoltageNodes_[nodeIndex(source.negative)] = 1U;
        }

        performanceStats_ = {};
        staticCacheDirty_ = true;
        prepared_ = true;
        rebuildStaticCache();
        if (hasNonlinearDevices()) prepareSparseNonlinearSolver();
        reset();
        return true;
    }

    void reset() noexcept {
        std::fill(solution_.begin(), solution_.end(), 0.0f);
        std::fill(candidate_.begin(), candidate_.end(), 0.0f);
        std::fill(previousSampleSolution_.begin(), previousSampleSolution_.end(), 0.0f);
        previousSampleSolutionValid_ = false;
        for (auto& c : capacitors_) {
            c.previousVoltage = 0.0f;
            c.previousCurrent = 0.0f;
        }
        for (auto& l : inductors_) {
            l.previousVoltage = 0.0f;
            l.previousCurrent = 0.0f;
        }
        for (auto& diode : diodes_) {
            diode.junctionVoltage = 0.0f;
            diode.junctionVoltageValid = false;
        }
        lastStats_ = {};
    }

    SolveStats processSample(int maximumNewtonIterations = 12, float tolerance = 1.0e-6f) noexcept {
        SolveStats stats{};
        if (!prepared_) {
            stats.converged = false;
            stats.singular = true;
            lastStats_ = stats;
            return stats;
        }

        ++performanceStats_.samples;
        if (staticCacheDirty_) rebuildStaticCache();
        assembleSampleRhs();

        maximumNewtonIterations = std::clamp(maximumNewtonIterations, 1, 40);
        tolerance = std::max(1.0e-9f, tolerance);
        const bool nonlinear = hasNonlinearDevices();

        if (!nonlinear) {
            rhs_ = sampleRhs_;
            if (!linearFactorValid_ || !solveCachedLinearSystem()) {
                stats.singular = true;
                stats.converged = false;
                stats.iterations = 1;
                lastStats_ = stats;
                return stats;
            }
            stats.iterations = 1;
            stats.converged = true;
            updateDynamicState();
            lastStats_ = stats;
            return stats;
        }

        const bool forceSparse = nonlinearSolverMode_ == NonlinearSolverMode::sparseFixedPattern;
        const bool usesPreparedSparse = sparseSolver_.available()
            && (forceSparse
                || (nonlinearSolverMode_ == NonlinearSolverMode::automatic
                    && dimension_ >= 12U && sparseSolver_.factorDensity() <= 0.70f));
        if (usesPreparedSparse) {
            sparseSolver_.prepareSampleRhs(sampleRhs_);
            if (previousSampleSolutionValid_) {
                // Consecutive oversampled audio states are strongly correlated.
                // A bounded first-order extrapolation is only an initial guess:
                // Newton still solves the unchanged component-level equations.
                for (std::size_t i = 0; i < dimension_; ++i) {
                    const float current = solution_[i];
                    const float slope = current - previousSampleSolution_[i];
                    const float scale = std::max(1.0f, std::abs(current));
                    candidate_[i] = current
                        + std::clamp(slope, -0.10f * scale, 0.10f * scale);
                    previousSampleSolution_[i] = current;
                }
            } else {
                candidate_ = solution_;
                previousSampleSolution_ = solution_;
            }
            previousSampleSolutionValid_ = true;
        } else {
            candidate_ = solution_;
        }
        rhs_ = sampleRhs_;
        bool candidateFromFullSparseSolve = false;
        stats.converged = false;
        for (int iteration = 0; iteration < maximumNewtonIterations; ++iteration) {
            for (const auto index : nonlinearMatrixIndices_)
                matrix_[index] = staticMatrix_[index];
            if (iteration > 0) {
                for (const auto row : nonlinearRhsIndices_)
                    rhs_[row] = sampleRhs_[row];
            }
            stampNonlinear(candidate_, usesPreparedSparse);
            ++performanceStats_.nonlinearAssemblies;

            if (iteration > 0
                && (usesPreparedSparse
                        ? (candidateFromFullSparseSolve
                            ? sparseSolver_.validateNonlinear(matrix_, rhs_, candidate_,
                                nonlinearResidualTolerance_)
                            : sparseSolver_.validate(matrix_, rhs_, candidate_,
                                nonlinearResidualTolerance_))
                        // The dense/general path has no equivalent cheap check, so it
                        // is only worth the extra pass once few iterations remain:
                        // a candidate can sit on a device's pinch-off/threshold kink
                        // or at a floating-point noise floor where the Jacobian is
                        // near-singular and every remaining Newton step overshoots
                        // or oscillates despite already satisfying the nonlinear
                        // equations well. Restricting it to the last few iterations
                        // keeps ordinary dense solves (already converging well
                        // before the budget runs out, e.g. the accelerated-nonlinear
                        // performance-counter fixtures) taking exactly one general
                        // solve per assembly, unchanged from before line search.
                        : (iteration + 3 >= maximumNewtonIterations
                            && scaledResidualNorm(candidate_) <= nonlinearResidualTolerance_))) {
                // Evaluate the actual nonlinear circuit equations at the current
                // candidate before factoring their Jacobian again. Near an
                // op-amp operating point, float-sized voltage jitter can keep
                // successive Newton iterates moving while KCL is already solved
                // to a fraction of one part per million. Residual convergence is
                // the physical criterion and avoids pointless 40-step cycles.
                solution_ = candidate_;
                stats.converged = true;
                stats.iterations = iteration;
                ++performanceStats_.nonlinearResidualConvergences;
                break;
            }

            // Merit value for the backtracking line search below: the residual of
            // the current candidate under this iteration's freshly stamped
            // Jacobian/companion sources, before any step is taken.
            const float candidateResidual = residualNorm(candidate_);

            bool solved = false;
            bool usedSparseSolve = false;
            if (usesPreparedSparse) {
                // Double-precision fixed-pattern LU checks every pivot here. The
                // more expensive backward-error residual is deferred until Newton
                // is ready to accept the sample, rather than repeated for every
                // intermediate iterate.
                solved = sparseSolver_.solve(matrix_, rhs_, solution_, false);
                if (solved) {
                    usedSparseSolve = true;
                    ++performanceStats_.sparseNewtonSolves;
                } else {
                    ++performanceStats_.sparseFallbackSolves;
                    solved = solveGeneralLinearSystem();
                }
            } else {
                if (forceSparse) ++performanceStats_.sparseFallbackSolves;
                solved = solveGeneralLinearSystem();
            }

            if (!solved) {
                stats.singular = true;
                stats.converged = false;
                stats.iterations = iteration + 1;
                lastStats_ = stats;
                return stats;
            }

            float maxDelta = 0.0f;
            float maxScaledDelta = 0.0f;
            for (std::size_t i = 0; i < dimension_; ++i) {
                const float delta = std::abs(solution_[i] - candidate_[i]);
                maxDelta = std::max(maxDelta, delta);
                const float scale = std::max(1.0f,
                    std::max(std::abs(solution_[i]), std::abs(candidate_[i])));
                maxScaledDelta = std::max(maxScaledDelta, delta / scale);
            }
            if (usedSparseSolve
                && (maxScaledDelta <= tolerance
                    || iteration + 1 == maximumNewtonIterations)
                && !sparseSolver_.validate(matrix_, rhs_, solution_)) {
                ++performanceStats_.sparseFallbackSolves;
                usedSparseSolve = false;
                if (!solveGeneralLinearSystem()) {
                    stats.singular = true;
                    stats.converged = false;
                    stats.iterations = iteration + 1;
                    lastStats_ = stats;
                    return stats;
                }
                maxDelta = 0.0f;
                maxScaledDelta = 0.0f;
                for (std::size_t i = 0; i < dimension_; ++i) {
                    const float delta = std::abs(solution_[i] - candidate_[i]);
                    maxDelta = std::max(maxDelta, delta);
                    const float scale = std::max(1.0f,
                        std::max(std::abs(solution_[i]), std::abs(candidate_[i])));
                    maxScaledDelta = std::max(maxScaledDelta, delta / scale);
                }
            }

            // Apply a lightweight trust region instead of taking an unrestricted
            // Newton jump after the first few iterations. Audio circuits routinely
            // combine exponential junctions, large op-amp gains and capacitor
            // companion conductances; a single full step can otherwise jump from a
            // valid several-volt operating point to hundreds of volts and poison the
            // next sample's dynamic history. The step limit scales with the current
            // node magnitude, so low-voltage semiconductor junctions stay tightly
            // controlled while tube/transformer nodes can still move by tens of volts.
            // Once the candidate is already inside the local Newton basin, a full
            // step restores quadratic convergence. The trust-region clamp remains
            // active for larger moves and startup/rail transitions.
            if (usedSparseSolve && maxDelta <= 0.25f) {
                // No trust-region bound can activate below the 0.25 V minimum,
                // so a full Newton step is exactly the solved vector. Copy it
                // directly instead of branching and clamping all 48 unknowns.
                candidate_ = solution_;
                candidateFromFullSparseSolve = true;
            } else {
                candidateFromFullSparseSolve = false;
                // Globalize the Newton step with backtracking line search instead of
                // unconditionally applying a fixed damping schedule. A fixed factor
                // (previously 0.65 for the first 3 iterations, then 0.85) makes the
                // damped update a deterministic, time-invariant map; around a
                // high-gain feedback stage that map can have a locally repelling
                // fixed point, so the iterate can settle into an exact repeating
                // cycle instead of converging, no matter how many iterations are
                // allowed (issue #14, reported as DS-1 silent-input hiss, later
                // found to be a structural property of the fixed-damping scheme
                // itself and not specific to any one circuit -- issue #16). Halving
                // the step until the trial state strictly lowers the residual below
                // the current candidate's is a standard globalization of Newton's
                // method: because every accepted step strictly decreases the merit
                // value, no state can ever recur, which rules a stable cycle out
                // structurally rather than by detecting and reacting to one after
                // the fact.
                constexpr int kMaxLineSearchBacktracks = 6;

                // Trust-region bound as a single shared scale rather than an
                // independent per-node clamp. Two node unknowns can be tied
                // together by an exact linear identity the rest of the Newton
                // solve depends on -- e.g. a floating voltage source's own row,
                // V_pos - V_neg = V_source, or the Ebers-Moll subcircuit's
                // zero-volt current-sense sources -- and clamping each side's step
                // to its own magnitude-scaled bound independently can shift the
                // two sides by different fractions of their raw step, breaking
                // that identity for every candidate until the solve fully
                // converges. Finding one scale that satisfies every node's bound
                // and applying it uniformly keeps any such identity intact at
                // every trial, exactly as an unclamped full step would.
                float trustRegionScale = 1.0f;
                for (std::size_t i = 0; i < nodeCount_; ++i) {
                    if (i < fixedVoltageNodes_.size() && fixedVoltageNodes_[i] != 0U) continue;
                    const float rawDelta = solution_[i] - candidate_[i];
                    const float scale = std::max(1.0f, std::abs(candidate_[i]));
                    // Raised from 25 V to 60 V (issue #27) so pentode/beam-tetrode
                    // power stages, whose plate/screen rails sit at 400-800 V, still
                    // reach their operating point within a handful of cold-start
                    // samples. The clamp only engages once |candidate| exceeds ~100 V
                    // (0.25 * scale > 25), so low-voltage diode/BJT/JFET junctions see
                    // no change from this and keep their original tight step limit.
                    const float maximumStep = std::clamp(0.25f * scale, 0.25f, 60.0f);
                    const float absRawDelta = std::abs(rawDelta);
                    if (absRawDelta > maximumStep)
                        trustRegionScale = std::min(trustRegionScale, maximumStep / absRawDelta);
                }

                const auto buildTrial = [&](float alpha) noexcept {
                    const float effectiveAlpha = alpha * trustRegionScale;
                    for (std::size_t i = 0; i < dimension_; ++i) {
                        if (i < nodeCount_ && i < fixedVoltageNodes_.size()
                            && fixedVoltageNodes_[i] != 0U) {
                            lineSearchCandidate_[i] = solution_[i];
                            continue;
                        }
                        lineSearchCandidate_[i] = candidate_[i]
                            + effectiveAlpha * (solution_[i] - candidate_[i]);
                    }
                };

                float alpha = 1.0f;
                bool lineSearchAccepted = false;
                for (int backtrack = 0; backtrack < kMaxLineSearchBacktracks; ++backtrack) {
                    buildTrial(alpha);
                    for (const auto index : nonlinearMatrixIndices_)
                        matrix_[index] = staticMatrix_[index];
                    for (const auto row : nonlinearRhsIndices_)
                        rhs_[row] = sampleRhs_[row];
                    stampNonlinear(lineSearchCandidate_, usesPreparedSparse);
                    const float trialResidual = residualNorm(lineSearchCandidate_);
                    // Accept a non-increasing residual, not only a strictly
                    // decreasing one. Once a circuit is within float precision of
                    // its true operating point, the residual has nothing left to
                    // shrink and a strict "<" can reject every alpha on noise
                    // alone, forcing the fixed-damping fallback below on every
                    // remaining iteration and burning the whole iteration budget
                    // without ever setting stats.converged. Rejecting only an
                    // actual increase still blocks a genuine repeating cycle
                    // (issue #14/#16), since a cycle's states are not exactly
                    // equal in residual by coincidence every step.
                    if (trialResidual <= candidateResidual) {
                        candidate_ = lineSearchCandidate_;
                        lineSearchAccepted = true;
                        break;
                    }
                    alpha *= 0.5f;
                }

                if (!lineSearchAccepted) {
                    // The Newton direction was not a descent direction of the
                    // residual at any of the alphas tried -- e.g. a JFET/MOSFET
                    // candidate sitting exactly on its pinch-off/threshold kink,
                    // where the channel model's derivative collapses to near zero
                    // and the raw Newton step overshoots wildly without actually
                    // being wrong. Falling back to the original fixed-damping step
                    // keeps this case exactly as robust as before line search
                    // existed, rather than forcing acceptance of a trial already
                    // known to make the residual worse.
                    const float damping = iteration < 3 ? 0.65f : 0.85f;
                    buildTrial(damping);
                    candidate_ = lineSearchCandidate_;
                }
            }

            stats.iterations = iteration + 1;
            if (maxScaledDelta <= tolerance) {
                stats.converged = true;
                candidate_ = solution_;
                break;
            }
        }

        if (!stats.converged) solution_ = candidate_;
        updateDynamicState();
        lastStats_ = stats;
        return stats;
    }

    // A single DC operating-point Newton solve at the circuit's *current*
    // independent-source values: capacitors are treated as open (i = C dv/dt
    // = 0 at DC) and inductors collapse to a plain resistor equal to their
    // series resistance (0 ohm = an ideal short), by driving the trapezoidal
    // companion model's coefficients to zero for the duration of this call --
    // see rebuildStaticCache()'s `dcAnalysisActive_` branch and
    // addDynamicRhs()'s early return below. The nonlinear solve itself is
    // exactly processSample()'s Newton loop (line search, trust region,
    // sparse/dense dispatch, diode junction-voltage reuse, everything) so a
    // caller doing source-stepping continuation across many small voltage
    // steps -- see TS808Circuit::primeOperatingPoint() for the pattern this
    // is meant to replace -- gets the identical globalization guarantees a
    // transient step gets. See the "Nonlinear solver design" note in
    // CLAUDE.md for why that shared machinery, not a bespoke DC-only Newton
    // loop, is the required approach for any new solve path.
    //
    // Does not touch capacitor/inductor companion history; call
    // commitOperatingPointAsSteadyState() once a homotopy has finished
    // ramping to its final target to seed that history from the converged
    // solution before resuming normal transient processSample() calls.
    SolveStats solveDcOperatingPoint(int maximumNewtonIterations = 40,
                                     float tolerance = 1.0e-6f) noexcept {
        const bool wasActive = dcAnalysisActive_;
        dcAnalysisActive_ = true;
        staticCacheDirty_ = true;
        const SolveStats stats = processSample(maximumNewtonIterations, tolerance);
        dcAnalysisActive_ = wasActive;
        staticCacheDirty_ = true;
        return stats;
    }

    // Seeds every capacitor/inductor's trapezoidal companion history from the
    // most recent solveDcOperatingPoint() solution, so the next transient
    // processSample() starts already at that equilibrium instead of cold
    // (all-zero) state. A capacitor carries zero current at DC by
    // definition; an inductor carries whatever DC current its solved branch
    // equation implies (nonzero only if it has series resistance).
    void commitOperatingPointAsSteadyState() noexcept {
        for (auto& c : capacitors_) {
            c.previousVoltage = voltage(c.a, c.b);
            c.previousCurrent = 0.0f;
        }
        for (auto& l : inductors_) {
            l.previousVoltage = voltage(l.a, l.b);
            l.previousCurrent = l.branchIndex < solution_.size() ? solution_[l.branchIndex] : 0.0f;
        }
        previousSampleSolutionValid_ = false;
        // solveDcOperatingPoint() always leaves staticCacheDirty_ set (the
        // real-dt companion coefficients still need recomputing after the
        // dt->infinity DC stamps). Rebuild it here, eagerly, rather than
        // leaving it for the caller's first transient processSample(): a
        // deferred rebuild would otherwise be charged to that first
        // post-prepare() sample, which several tests (e.g.
        // PreampCircuitTests' "prolonged silence preserves the cached
        // physical operating point") assert never happens once
        // performance counters are reset after prepare().
        rebuildStaticCache();
    }

    float voltage(Node node) const noexcept {
        if (node == ground) return 0.0f;
        const auto index = nodeIndex(node);
        return index < solution_.size() ? solution_[index] : 0.0f;
    }

    float voltage(Node positive, Node negative) const noexcept {
        return voltage(positive) - voltage(negative);
    }

    float inductorCurrent(std::size_t index) const noexcept {
        if (index >= inductors_.size()) return 0.0f;
        const auto branch = inductors_[index].branchIndex;
        return branch < solution_.size() ? solution_[branch] : 0.0f;
    }

    std::size_t nodeCount() const noexcept { return nodeCount_; }
    std::size_t dimension() const noexcept { return dimension_; }
    bool prepared() const noexcept { return prepared_; }
    SolveStats lastStats() const noexcept { return lastStats_; }
    PerformanceStats performanceStats() const noexcept { return performanceStats_; }
    void resetPerformanceStats() noexcept { performanceStats_ = {}; }
    void setNonlinearSolverMode(NonlinearSolverMode mode) noexcept { nonlinearSolverMode_ = mode; }
    NonlinearSolverMode nonlinearSolverMode() const noexcept { return nonlinearSolverMode_; }
    void setNonlinearResidualTolerance(float tolerance) noexcept {
        nonlinearResidualTolerance_ = std::clamp(static_cast<double>(tolerance),
                                                  1.0e-9, 2.0e-4);
    }
    float nonlinearResidualTolerance() const noexcept {
        return static_cast<float>(nonlinearResidualTolerance_);
    }
    bool sparseNonlinearSolverAvailable() const noexcept { return sparseSolver_.available(); }
    std::size_t sparseNonlinearOriginalNonZeros() const noexcept { return sparseSolver_.originalNonZeros(); }
    std::size_t sparseNonlinearFactorNonZeros() const noexcept { return sparseSolver_.factorNonZeros(); }
    std::size_t sparseNonlinearCachedLinearUnknowns() const noexcept {
        return sparseSolver_.cachedLinearUnknowns();
    }
    float sparseNonlinearFactorDensity() const noexcept { return sparseSolver_.factorDensity(); }

private:
    // Compiled nonlinear stamp write destinations. Node topology (and hence
    // every row/column a nonlinear device's Jacobian can ever touch) is fixed
    // once prepare() runs; only the derivative/current values Newton computes
    // each iteration actually change. Resolving "which flat matrix_/rhs_ cell
    // does this Jacobian term write to" once here -- instead of re-deriving it
    // from Node identities via nodeIndex()/ground comparisons on every Newton
    // iteration of every device -- is docs/MNA_ACCELERATION.md's "next
    // acceleration stage" #1. npos marks a ground-connected terminal, which
    // never has a matrix column/row of its own.
    static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();

    template <std::size_t N>
    struct CompiledCurrentStamp {
        std::size_t positiveRow = npos;
        std::size_t negativeRow = npos;
        std::array<std::size_t, N> positiveSlot{};
        std::array<std::size_t, N> negativeSlot{};
    };

    struct CompiledConductanceStamp {
        std::size_t aRow = npos;
        std::size_t bRow = npos;
        std::size_t aaSlot = npos;
        std::size_t bbSlot = npos;
        std::size_t abSlot = npos;
        std::size_t baSlot = npos;
    };

    struct Resistor { Node a{}, b{}; hq::ResistorSpec spec{}; };
    struct Capacitor {
        Node a{}, b{};
        hq::CapacitorSpec spec{};
        float previousVoltage = 0.0f;
        float previousCurrent = 0.0f;
        float companionConductance = 0.0f;
    };
    struct Inductor {
        Node a{}, b{};
        hq::InductorSpec spec{};
        float previousVoltage = 0.0f;
        float previousCurrent = 0.0f;
        std::size_t branchIndex = 0;
        float companionAlpha = 0.0f;
    };
    struct Potentiometer { Node high{}, wiper{}, low{}; hq::PotentiometerSpec spec{}; };
    struct CurrentSource { Node positive{}, negative{}; float amps = 0.0f; };
    struct VoltageSource { Node positive{}, negative{}; float volts = 0.0f; std::size_t branchIndex = 0; };
    struct Vccs {
        Node outputPositive{}, outputNegative{}, controlPositive{}, controlNegative{};
        float transconductance = 0.0f;
    };
    struct Vcvs {
        Node outputPositive{}, outputNegative{}, controlPositive{}, controlNegative{};
        float gain = 1.0f;
        std::size_t branchIndex = 0;
    };
    struct Cccs {
        Node outputPositive{}, outputNegative{};
        SourceHandle controlSource{};
        float gain = 1.0f;
    };
    struct Ccvs {
        Node outputPositive{}, outputNegative{};
        SourceHandle controlSource{};
        float transresistanceOhms = 1.0f;
        std::size_t branchIndex = 0;
    };
    struct Diode {
        Node anode{}, cathode{};
        hq::DiodeSpec spec{};
        float junctionVoltage = 0.0f;
        float inverseThermalVoltage = 0.0f;
        bool junctionVoltageValid = false;
        CompiledConductanceStamp conductanceLayout{};
    };
    struct Bjt {
        Node collector{}, base{}, emitter{};
        hq::BJTSpec spec{};
        CompiledCurrentStamp<3> collectorLayout{};
        CompiledCurrentStamp<3> baseLayout{};
    };
    struct Jfet {
        Node drain{}, gate{}, source{};
        hq::JFETSpec spec{};
        CompiledCurrentStamp<3> layout{};
    };
    struct Mosfet {
        Node drain{}, gate{}, source{};
        hq::MOSFETSpec spec{};
        CompiledCurrentStamp<3> layout{};
    };
    struct OpAmp {
        Node output{}, nonInverting{}, inverting{}, reference{};
        hq::OpAmpSpec spec{};
        std::size_t branchIndex = 0;
    };
    struct Triode {
        Node plate{}, grid{}, cathode{};
        hq::TriodeSpec spec{};
        CompiledCurrentStamp<3> layout{};
    };
    struct Pentode {
        Node plate{}, grid{}, screen{}, cathode{};
        hq::PentodeSpec spec{};
        CompiledCurrentStamp<4> plateLayout{};
        CompiledCurrentStamp<3> screenLayout{};
    };

    struct JacobianTerm { Node node{}; float derivative = 0.0f; };
    struct DiodeLinearization { float current = 0.0f; float conductance = 0.0f; };

    static std::size_t nodeIndex(Node node) noexcept {
        return static_cast<std::size_t>(node - 1U);
    }

    // Resolve every flat matrix_ cell a device's linearized-current stamp can
    // ever write once, from its fixed terminal topology. `nodes` gives the
    // Jacobian's column terminals in the same order the caller will later pass
    // derivative values, so the two line up positionally at stamp time.
    template <std::size_t N>
    static CompiledCurrentStamp<N> compileCurrentStamp(std::size_t dimension, Node positive,
                                                        Node negative,
                                                        const std::array<Node, N>& nodes) noexcept {
        CompiledCurrentStamp<N> layout;
        layout.positiveRow = positive != ground ? nodeIndex(positive) : npos;
        layout.negativeRow = negative != ground ? nodeIndex(negative) : npos;
        for (std::size_t i = 0; i < N; ++i) {
            const auto column = nodes[i] != ground ? nodeIndex(nodes[i]) : npos;
            layout.positiveSlot[i] = (layout.positiveRow != npos && column != npos)
                ? layout.positiveRow * dimension + column : npos;
            layout.negativeSlot[i] = (layout.negativeRow != npos && column != npos)
                ? layout.negativeRow * dimension + column : npos;
        }
        return layout;
    }

    // Same idea for a two-terminal conductance stamp (the diode's own
    // anode/cathode companion conductance): compile the up-to-four matrix
    // cells it can touch once from its fixed terminals.
    static CompiledConductanceStamp compileConductanceStamp(std::size_t dimension, Node a,
                                                             Node b) noexcept {
        CompiledConductanceStamp layout;
        layout.aRow = a != ground ? nodeIndex(a) : npos;
        layout.bRow = b != ground ? nodeIndex(b) : npos;
        layout.aaSlot = layout.aRow != npos ? layout.aRow * dimension + layout.aRow : npos;
        layout.bbSlot = layout.bRow != npos ? layout.bRow * dimension + layout.bRow : npos;
        layout.abSlot = (layout.aRow != npos && layout.bRow != npos)
            ? layout.aRow * dimension + layout.bRow : npos;
        layout.baSlot = (layout.aRow != npos && layout.bRow != npos)
            ? layout.bRow * dimension + layout.aRow : npos;
        return layout;
    }

    void stampConductanceCompiled(const CompiledConductanceStamp& layout,
                                  float conductance) noexcept {
        if (layout.aaSlot != npos) matrix_[layout.aaSlot] += conductance;
        if (layout.bbSlot != npos) matrix_[layout.bbSlot] += conductance;
        if (layout.abSlot != npos) matrix_[layout.abSlot] -= conductance;
        if (layout.baSlot != npos) matrix_[layout.baSlot] -= conductance;
    }

    void stampCurrentSourceCompiled(const CompiledConductanceStamp& layout,
                                    float current) noexcept {
        if (layout.aRow != npos) rhs_[layout.aRow] -= current;
        if (layout.bRow != npos) rhs_[layout.bRow] += current;
    }

    float nodeVoltage(const std::vector<float>& x, Node node) const noexcept {
        if (node == ground) return 0.0f;
        const auto i = nodeIndex(node);
        return i < x.size() ? x[i] : 0.0f;
    }

    bool hasNonlinearDevices() const noexcept {
        return !diodes_.empty() || !bjts_.empty() || !jfets_.empty() ||
               !mosfets_.empty() || !triodes_.empty() || !pentodes_.empty();
    }

    std::size_t voltageSourceBranch(SourceHandle handle) const noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= voltageSources_.size()) return dimension_;
        return voltageSources_[i].branchIndex;
    }

    void addMatrix(std::vector<float>& matrix, std::size_t row,
                   std::size_t col, float value) const noexcept {
        matrix[row * dimension_ + col] += value;
    }

    void stampConductance(std::vector<float>& matrix, Node a, Node b,
                          float conductance) const noexcept {
        if (a != ground) addMatrix(matrix, nodeIndex(a), nodeIndex(a), conductance);
        if (b != ground) addMatrix(matrix, nodeIndex(b), nodeIndex(b), conductance);
        if (a != ground && b != ground) {
            addMatrix(matrix, nodeIndex(a), nodeIndex(b), -conductance);
            addMatrix(matrix, nodeIndex(b), nodeIndex(a), -conductance);
        }
    }

    void stampCurrentSource(std::vector<float>& rhs, Node positive,
                            Node negative, float current) const noexcept {
        if (positive != ground) rhs[nodeIndex(positive)] -= current;
        if (negative != ground) rhs[nodeIndex(negative)] += current;
    }

    void stampBranchStructure(std::vector<float>& matrix, Node positive,
                              Node negative, std::size_t branchIndex) const noexcept {
        if (positive != ground) {
            addMatrix(matrix, nodeIndex(positive), branchIndex, 1.0f);
            addMatrix(matrix, branchIndex, nodeIndex(positive), 1.0f);
        }
        if (negative != ground) {
            addMatrix(matrix, nodeIndex(negative), branchIndex, -1.0f);
            addMatrix(matrix, branchIndex, nodeIndex(negative), -1.0f);
        }
    }

    void stampVccs(std::vector<float>& matrix, Node outputPositive,
                   Node outputNegative, Node controlPositive,
                   Node controlNegative, float transconductance) const noexcept {
        if (outputPositive != ground) {
            if (controlPositive != ground)
                addMatrix(matrix, nodeIndex(outputPositive), nodeIndex(controlPositive), transconductance);
            if (controlNegative != ground)
                addMatrix(matrix, nodeIndex(outputPositive), nodeIndex(controlNegative), -transconductance);
        }
        if (outputNegative != ground) {
            if (controlPositive != ground)
                addMatrix(matrix, nodeIndex(outputNegative), nodeIndex(controlPositive), -transconductance);
            if (controlNegative != ground)
                addMatrix(matrix, nodeIndex(outputNegative), nodeIndex(controlNegative), transconductance);
        }
    }

    void stampVcvs(std::vector<float>& matrix, const Vcvs& source) const noexcept {
        stampBranchStructure(matrix, source.outputPositive, source.outputNegative, source.branchIndex);
        if (source.controlPositive != ground)
            addMatrix(matrix, source.branchIndex, nodeIndex(source.controlPositive), -source.gain);
        if (source.controlNegative != ground)
            addMatrix(matrix, source.branchIndex, nodeIndex(source.controlNegative), source.gain);
    }

    void stampCccs(std::vector<float>& matrix, const Cccs& source) const noexcept {
        const auto controlBranch = voltageSourceBranch(source.controlSource);
        if (controlBranch >= dimension_) return;
        if (source.outputPositive != ground)
            addMatrix(matrix, nodeIndex(source.outputPositive), controlBranch, source.gain);
        if (source.outputNegative != ground)
            addMatrix(matrix, nodeIndex(source.outputNegative), controlBranch, -source.gain);
    }

    void stampCcvs(std::vector<float>& matrix, const Ccvs& source) const noexcept {
        const auto controlBranch = voltageSourceBranch(source.controlSource);
        stampBranchStructure(matrix, source.outputPositive, source.outputNegative, source.branchIndex);
        if (controlBranch < dimension_)
            addMatrix(matrix, source.branchIndex, controlBranch, -source.transresistanceOhms);
    }

    void stampOpAmp(std::vector<float>& matrix, std::vector<float>& rhs,
                    const OpAmp& device) const noexcept {
        const float gainDb = std::clamp(device.spec.openLoopGainDb, 0.0f, 120.0f);
        const float gain = std::pow(10.0f, gainDb / 20.0f);
        stampBranchStructure(matrix, device.output, device.reference, device.branchIndex);
        if (device.nonInverting != ground)
            addMatrix(matrix, device.branchIndex, nodeIndex(device.nonInverting), -gain);
        if (device.inverting != ground)
            addMatrix(matrix, device.branchIndex, nodeIndex(device.inverting), gain);
        rhs[device.branchIndex] += gain * device.spec.inputOffsetVoltage;
    }

    void rebuildStaticCache() noexcept {
        std::fill(staticMatrix_.begin(), staticMatrix_.end(), 0.0f);
        std::fill(staticRhs_.begin(), staticRhs_.end(), 0.0f);
        // A DC operating-point solve (dcAnalysisActive_) reuses this exact
        // stamping code with dt -> infinity: a capacitor's companion
        // conductance 2C/dt and an inductor's companion alpha 2L/dt both
        // collapse to exactly 0.0f (finite / infinite is well-defined in
        // IEEE 754), which is precisely "open circuit" for a capacitor and
        // "resistor equal to series resistance" (0 ohm = ideal short) for an
        // inductor -- see solveDcOperatingPoint() above.
        const float dt = dcAnalysisActive_
            ? std::numeric_limits<float>::infinity()
            : 1.0f / static_cast<float>(sampleRate_);

        for (const auto& r : resistors_) {
            const float resistance = std::max(1.0e-6f, r.spec.resistanceOhms);
            stampConductance(staticMatrix_, r.a, r.b, 1.0f / resistance);
        }

        for (const auto& p : potentiometers_) {
            constexpr float contactFloorOhms = 1.0e-3f;
            const float total = std::max(2.0f * contactFloorOhms, p.spec.totalResistanceOhms);
            const float position = std::clamp(p.spec.normalizedElectricalPosition(), 0.0f, 1.0f);
            const float lowResistance = std::max(contactFloorOhms, total * position);
            const float highResistance = std::max(contactFloorOhms, total * (1.0f - position));
            stampConductance(staticMatrix_, p.high, p.wiper, 1.0f / highResistance);
            stampConductance(staticMatrix_, p.wiper, p.low, 1.0f / lowResistance);
        }

        for (auto& c : capacitors_) {
            const float capacitance = std::max(0.0f, c.spec.capacitanceFarads);
            c.companionConductance = 2.0f * capacitance / dt;
            if (c.companionConductance > 0.0f)
                stampConductance(staticMatrix_, c.a, c.b, c.companionConductance);
            if (c.spec.leakageResistanceOhms > 0.0f && std::isfinite(c.spec.leakageResistanceOhms))
                stampConductance(staticMatrix_, c.a, c.b,
                                 1.0f / std::max(1.0f, c.spec.leakageResistanceOhms));
        }

        for (const auto& source : currentSources_)
            stampCurrentSource(staticRhs_, source.positive, source.negative, source.amps);

        for (const auto& source : voltageSources_)
            stampBranchStructure(staticMatrix_, source.positive, source.negative, source.branchIndex);

        for (const auto& source : vccs_)
            stampVccs(staticMatrix_, source.outputPositive, source.outputNegative,
                      source.controlPositive, source.controlNegative, source.transconductance);
        for (const auto& source : vcvs_) stampVcvs(staticMatrix_, source);
        for (const auto& source : cccs_) stampCccs(staticMatrix_, source);
        for (const auto& source : ccvs_) stampCcvs(staticMatrix_, source);
        for (const auto& opAmp : opAmps_) stampOpAmp(staticMatrix_, staticRhs_, opAmp);

        for (auto& l : inductors_) {
            const float inductance = std::max(1.0e-12f, l.spec.inductanceHenries);
            l.companionAlpha = 2.0f * inductance / dt;
            const float rEq = l.companionAlpha + std::max(0.0f, l.spec.seriesResistanceOhms);
            stampBranchStructure(staticMatrix_, l.a, l.b, l.branchIndex);
            addMatrix(staticMatrix_, l.branchIndex, l.branchIndex, -rEq);
        }

        staticCacheDirty_ = false;
        matrix_ = staticMatrix_;
        if (sparseSolver_.available()) sparseSolver_.refreshStaticFactor(staticMatrix_);
        previousSampleSolutionValid_ = false;
        ++performanceStats_.staticCacheRebuilds;
        linearFactorValid_ = !hasNonlinearDevices() && factorizeStaticLinearSystem();
    }

    void assembleSampleRhs() noexcept {
        sampleRhs_ = staticRhs_;
        addDynamicRhs(sampleRhs_);
        ++performanceStats_.sampleRhsAssemblies;
    }

    void addDynamicRhs(std::vector<float>& rhs) const noexcept {
        for (const auto& source : voltageSources_)
            rhs[source.branchIndex] += source.volts;

        // A capacitor's history term is already zero in DC mode because
        // dt = infinity drives companionConductance to exactly 0.0f (see the
        // `continue` below). An inductor's is not: historyVoltage still
        // carries a `-previousVoltage` term even with companionAlpha == 0,
        // which would bias the DC solve away from the plain-resistor model
        // solveDcOperatingPoint() relies on. Skip both loops outright so a
        // DC solve sees an untouched open/short system with no leftover
        // transient history.
        if (dcAnalysisActive_) return;

        for (const auto& c : capacitors_) {
            if (c.companionConductance <= 0.0f) continue;
            const float historyCurrent = -c.companionConductance * c.previousVoltage - c.previousCurrent;
            stampCurrentSource(rhs, c.a, c.b, historyCurrent);
        }

        for (const auto& l : inductors_) {
            const float historyVoltage = -l.companionAlpha * l.previousCurrent - l.previousVoltage;
            rhs[l.branchIndex] += historyVoltage;
        }
    }

    // `layout` resolves this call's positive/negative rows and every
    // Jacobian-term column to a flat matrix_ slot once, in prepare() (see
    // compileCurrentStamp above), rather than re-deriving row/column addresses
    // from Node identities on every Newton iteration. jacobian[i] and
    // layout.positiveSlot[i]/negativeSlot[i] correspond positionally; the
    // equivalent-current calculation below is untouched and still reads
    // nodeVoltage()/ground exactly as before.
    template <std::size_t N>
    void stampLinearizedCurrent(const CompiledCurrentStamp<N>& layout, float currentAtGuess,
                                const std::vector<float>& guess,
                                const std::array<JacobianTerm, N>& jacobian) noexcept {
        float equivalentCurrent = currentAtGuess;
        for (const auto& term : jacobian)
            equivalentCurrent -= term.derivative * nodeVoltage(guess, term.node);

        for (std::size_t i = 0; i < N; ++i)
            if (layout.positiveSlot[i] != npos) matrix_[layout.positiveSlot[i]] += jacobian[i].derivative;
        for (std::size_t i = 0; i < N; ++i)
            if (layout.negativeSlot[i] != npos) matrix_[layout.negativeSlot[i]] -= jacobian[i].derivative;

        if (layout.positiveRow != npos) rhs_[layout.positiveRow] -= equivalentCurrent;
        if (layout.negativeRow != npos) rhs_[layout.negativeRow] += equivalentCurrent;
    }

    static DiodeLinearization linearizeDiode(Diode& diode,
                                             float terminalVoltage,
                                             bool reuseJunctionVoltage) noexcept {
        const auto& spec = diode.spec;
        const float rs = std::max(0.0f, spec.seriesResistanceOhms);
        const float denominator = std::max(1.0e-6f,
            spec.emissionCoefficient * spec.thermalVoltage);
        const auto evaluate = [&](float voltage) noexcept {
            const float exponent = reuseJunctionVoltage
                ? voltage * diode.inverseThermalVoltage
                : voltage / denominator;
            const float exponential = exponent <= -40.0f
                ? 4.248354255291589e-18f
                : exponent >= 40.0f
                    ? 2.3538526683702e17f
                    : std::exp(exponent);
            return DiodeLinearization{
                spec.saturationCurrent * (exponential - 1.0f),
                reuseJunctionVoltage
                    ? spec.saturationCurrent * exponential * diode.inverseThermalVoltage
                    : spec.saturationCurrent * exponential / denominator
            };
        };
        // Consecutive Newton iterates and oversampled audio samples start close
        // to the same physical junction voltage. Reusing that previous implicit
        // root avoids repeatedly solving a forward-biased BJT diode from its
        // terminal voltage. The identical diode equation is still solved to the
        // original tolerance; only its initial guess changes.
        float vj = reuseJunctionVoltage && diode.junctionVoltageValid
                && std::abs(diode.junctionVoltage - terminalVoltage) <= 0.75f
            ? diode.junctionVoltage : terminalVoltage;
        for (int i = 0; i < 8; ++i) {
            const auto junction = evaluate(vj);
            const float residual = vj + rs * junction.current - terminalVoltage;
            const float derivative = 1.0f + rs * junction.conductance;
            const float step = residual / std::max(1.0e-12f, derivative);
            const float nextJunctionVoltage = vj - std::clamp(step, -0.5f, 0.5f);
            if (nextJunctionVoltage == vj) {
                // The last Newton correction is below float resolution. Its
                // already-computed exponential is exactly the final junction
                // evaluation; recomputing exp at the identical bit pattern only
                // wastes time on quiet, biased semiconductor operating points.
                diode.junctionVoltage = vj;
                diode.junctionVoltageValid = std::isfinite(vj);
                return {junction.current,
                        junction.conductance /
                            std::max(1.0e-12f, 1.0f + rs * junction.conductance)};
            }
            vj = nextJunctionVoltage;
            if (std::abs(step) < 1.0e-7f) break;
        }
        diode.junctionVoltage = vj;
        diode.junctionVoltageValid = std::isfinite(vj);
        const auto junction = evaluate(vj);
        return {junction.current,
                junction.conductance /
                    std::max(1.0e-12f, 1.0f + rs * junction.conductance)};
    }

    void stampBjt(const Bjt& device, const std::vector<float>& guess) noexcept {
        const float polarity = device.spec.polarity == hq::TransistorPolarity::pnp ? -1.0f : 1.0f;
        const float vc = nodeVoltage(guess, device.collector);
        const float vb = nodeVoltage(guess, device.base);
        const float ve = nodeVoltage(guess, device.emitter);
        const float vbe = polarity * (vb - ve);
        const float vce = polarity * (vc - ve);

        const float vt = std::max(1.0e-4f, device.spec.thermalVoltage);
        constexpr float saturationCurrent = 1.0e-14f;
        const float exponent = std::clamp(vbe / vt, -40.0f, 28.0f);
        const float exponential = std::exp(exponent);
        float idealCollector = std::max(0.0f, saturationCurrent * (exponential - 1.0f));
        float dIdealDvbe = idealCollector > 0.0f ? saturationCurrent * exponential / vt : 0.0f;

        const float vSat = std::max(0.02f, device.spec.saturationVoltage);
        const float positiveVce = std::max(0.0f, vce);
        const float saturationFactor = 1.0f - std::exp(-positiveVce / vSat);
        const float dSaturationDvce = vce > 0.0f ? std::exp(-positiveVce / vSat) / vSat : 0.0f;

        const float currentLimit = std::max(1.0e-6f, device.spec.maxCollectorCurrentAmps);
        float collectorMagnitude = idealCollector * saturationFactor;
        float dCollectorDvbe = dIdealDvbe * saturationFactor;
        float dCollectorDvce = idealCollector * dSaturationDvce;
        if (collectorMagnitude > currentLimit) {
            collectorMagnitude = currentLimit;
            dCollectorDvbe = 0.0f;
            dCollectorDvce = 0.0f;
        }

        const float collectorCurrent = polarity * collectorMagnitude;
        const std::array<JacobianTerm, 3> collectorJac{{
            {device.collector, dCollectorDvce},
            {device.base, dCollectorDvbe},
            {device.emitter, -(dCollectorDvce + dCollectorDvbe)}
        }};
        stampLinearizedCurrent(device.collectorLayout, collectorCurrent, guess, collectorJac);

        const float beta = std::max(1.0f, device.spec.beta);
        const float baseCurrent = collectorCurrent / beta;
        const std::array<JacobianTerm, 3> baseJac{{
            {device.collector, dCollectorDvce / beta},
            {device.base, dCollectorDvbe / beta},
            {device.emitter, -(dCollectorDvce + dCollectorDvbe) / beta}
        }};
        stampLinearizedCurrent(device.baseLayout, baseCurrent, guess, baseJac);
    }

    void stampJfet(const Jfet& device, const std::vector<float>& guess) noexcept {
        const float polarity = device.spec.polarity == hq::TransistorPolarity::pChannel ? -1.0f : 1.0f;
        const float vd = nodeVoltage(guess, device.drain);
        const float vg = nodeVoltage(guess, device.gate);
        const float vs = nodeVoltage(guess, device.source);
        const float vgs = polarity * (vg - vs);
        const float vds = polarity * (vd - vs);

        const float vp = std::min(-1.0e-3f, device.spec.pinchOffVoltage);
        float channel = 0.0f;
        float dChannelDvgs = 0.0f;
        if (vgs > vp) {
            const float ratio = 1.0f - vgs / vp;
            channel = std::max(0.0f, device.spec.idssAmps) * ratio * ratio;
            dChannelDvgs = -2.0f * std::max(0.0f, device.spec.idssAmps) * ratio / vp;
        }

        constexpr float kneeVolts = 0.20f;
        const float normalizedVds = vds / kneeVolts;
        const float direction = std::tanh(normalizedVds);
        const float dDirectionDvds = (1.0f - direction * direction) / kneeVolts;
        const float lambda = std::max(0.0f, device.spec.lambda);
        const float modulation = 1.0f + lambda * std::abs(vds);
        const float dModulationDvds = vds > 0.0f ? lambda : vds < 0.0f ? -lambda : 0.0f;

        const float effectiveCurrent = channel * direction * modulation;
        const float dIdDvgs = dChannelDvgs * direction * modulation;
        const float dIdDvds = channel * (dDirectionDvds * modulation + direction * dModulationDvds);
        const float current = polarity * effectiveCurrent;
        const std::array<JacobianTerm, 3> jac{{
            {device.drain, dIdDvds},
            {device.gate, dIdDvgs},
            {device.source, -(dIdDvds + dIdDvgs)}
        }};
        stampLinearizedCurrent(device.layout, current, guess, jac);
    }

    void stampMosfet(const Mosfet& device, const std::vector<float>& guess) noexcept {
        const float polarity = device.spec.polarity == hq::TransistorPolarity::pChannel ? -1.0f : 1.0f;
        const float vd = nodeVoltage(guess, device.drain);
        const float vg = nodeVoltage(guess, device.gate);
        const float vs = nodeVoltage(guess, device.source);
        const float vgs = polarity * (vg - vs);
        const float vds = polarity * (vd - vs);
        const float threshold = std::max(0.0f, device.spec.thresholdVoltage);
        const float overdrive = vgs - threshold;
        // An off rail shunt contributes exactly zero current and zero Jacobian.
        // Skip its stamp entirely instead of rewriting nine zero matrix entries.
        if (overdrive <= 0.0f || vds <= 0.0f) return;
        const float k = std::max(0.0f, device.spec.transconductance);
        const float lambda = std::max(0.0f, device.spec.lambda);

        float effectiveCurrent = 0.0f;
        float dIdDvgs = 0.0f;
        float dIdDvds = 0.0f;
        const float modulation = 1.0f + lambda * vds;
        if (vds < overdrive) {
            const float base = k * (overdrive * vds - 0.5f * vds * vds);
            effectiveCurrent = base * modulation;
            dIdDvgs = k * vds * modulation;
            dIdDvds = k * (overdrive - vds) * modulation + base * lambda;
        } else {
            const float base = 0.5f * k * overdrive * overdrive;
            effectiveCurrent = base * modulation;
            dIdDvgs = k * overdrive * modulation;
            dIdDvds = base * lambda;
        }

        const float current = polarity * effectiveCurrent;
        const std::array<JacobianTerm, 3> jac{{
            {device.drain, dIdDvds},
            {device.gate, dIdDvgs},
            {device.source, -(dIdDvds + dIdDvgs)}
        }};
        stampLinearizedCurrent(device.layout, current, guess, jac);
    }

    void stampTriode(const Triode& device, const std::vector<float>& guess) noexcept {
        const float vp = nodeVoltage(guess, device.plate);
        const float vg = nodeVoltage(guess, device.grid);
        const float vk = nodeVoltage(guess, device.cathode);
        const float vpk = vp - vk;
        const float vgk = vg - vk;

        // Analytic Jacobian (see TriodeModel::plateCurrentJacobian) replaces
        // the previous two-sided central-difference gm/gp: same clamp
        // envelope, exact partials instead of a finite-difference estimate.
        // Zeroing gm/gp outside [0, 0.20] matches what a central difference
        // straddling that clamp would settle to (a constant clamped current
        // has zero slope), since the clamp is a stamp-level current limit,
        // not part of the device model itself.
        const auto raw = device.spec.model.plateCurrentJacobian(vgk, vpk);
        const bool inRange = raw.current >= 0.0f && raw.current <= 0.20f;
        const float current = std::clamp(raw.current, 0.0f, 0.20f);
        const float gm = inRange ? raw.gm : 0.0f;
        const float gp = inRange ? raw.gp : 0.0f;

        const float safeGm = std::clamp(gm, -1.0f, 1.0f);
        const float safeGp = std::clamp(gp, -1.0f, 1.0f);
        const std::array<JacobianTerm, 3> jac{{
            {device.plate, safeGp},
            {device.grid, safeGm},
            {device.cathode, -(safeGp + safeGm)}
        }};
        stampLinearizedCurrent(device.layout, current, guess, jac);
    }

    void stampPentode(const Pentode& device, const std::vector<float>& guess) noexcept {
        const float vp = nodeVoltage(guess, device.plate);
        const float vg = nodeVoltage(guess, device.grid);
        const float vs = nodeVoltage(guess, device.screen);
        const float vk = nodeVoltage(guess, device.cathode);
        const float vpk = vp - vk;
        const float vgk = vg - vk;
        const float vsk = vs - vk;

        // Analytic Jacobian (see PentodeModel::currentsAndJacobian) replaces
        // the previous 12-call central-difference construction below with a
        // single closed-form evaluation. As in stampTriode, each current's
        // partials are zeroed outside its stamp clamp envelope to match what
        // a straddling central difference would have settled to.
        const auto raw = device.spec.model.currentsAndJacobian(vgk, vpk, vsk);

        const bool plateInRange = raw.plateCurrent >= 0.0f && raw.plateCurrent <= 0.40f;
        const float plateCurrent = std::clamp(raw.plateCurrent, 0.0f, 0.40f);
        const float gm = plateInRange ? raw.dPlateDVg : 0.0f;
        const float gp = plateInRange ? raw.dPlateDVp : 0.0f;
        const float gscreen = plateInRange ? raw.dPlateDVs : 0.0f;

        const bool screenInRange = raw.screenCurrent >= 0.0f && raw.screenCurrent <= 0.15f;
        const float screenCurrent = std::clamp(raw.screenCurrent, 0.0f, 0.15f);
        const float gsGrid = screenInRange ? raw.dScreenDVg : 0.0f;
        const float gsScreen = screenInRange ? raw.dScreenDVs : 0.0f;

        const float safeGm = std::clamp(gm, -1.0f, 1.0f);
        const float safeGp = std::clamp(gp, -1.0f, 1.0f);
        const float safeGscreen = std::clamp(gscreen, -1.0f, 1.0f);
        const std::array<JacobianTerm, 4> plateJac{{
            {device.plate, safeGp},
            {device.grid, safeGm},
            {device.screen, safeGscreen},
            {device.cathode, -(safeGp + safeGm + safeGscreen)}
        }};
        stampLinearizedCurrent(device.plateLayout, plateCurrent, guess, plateJac);

        const float safeGsGrid = std::clamp(gsGrid, -1.0f, 1.0f);
        const float safeGsScreen = std::clamp(gsScreen, -1.0f, 1.0f);
        const std::array<JacobianTerm, 3> screenJac{{
            {device.grid, safeGsGrid},
            {device.screen, safeGsScreen},
            {device.cathode, -(safeGsGrid + safeGsScreen)}
        }};
        stampLinearizedCurrent(device.screenLayout, screenCurrent, guess, screenJac);
    }

    // Plain (unweighted) Euclidean norm of the KCL residual F(x) = A x - b for the
    // system currently stamped into matrix_/rhs_. Because nonlinear device stamps
    // are Newton-Raphson companion models linearized at their own guess, this is
    // the true residual of the nonlinear circuit equations at x, not an artifact
    // of the linearization. Deliberately left unscaled: the backtracking line
    // search further down relies on this being exactly ||F(x)|| under a *fixed*
    // norm, since that is what guarantees a Newton step is a descent direction of
    // the merit as alpha -> 0. An earlier version divided each row by a per-row
    // scale that itself depended on x (a backward-error style residual, like
    // FixedPatternSparseSolver's own validate()); that additional x-dependence
    // broke the descent guarantee and line search would chase a residual that
    // kept climbing as alpha shrank instead of settling back to the untouched
    // candidate's value.
    float residualNorm(const std::vector<float>& x) const noexcept {
        double sumSquares = 0.0;
        for (std::size_t row = 0; row < dimension_; ++row) {
            double sum = -static_cast<double>(rhs_[row]);
            const std::size_t base = row * dimension_;
            for (std::size_t slot = residualRowOffsets_[row];
                 slot < residualRowOffsets_[row + 1U]; ++slot) {
                const auto col = residualColumns_[slot];
                sum += static_cast<double>(matrix_[base + col]) * x[col];
            }
            sumSquares += sum * sum;
        }
        return static_cast<float>(std::sqrt(sumSquares));
    }

    // Scaled backward-error residual, matching the style FixedPatternSparseSolver
    // uses to validate a solve (and the same nonlinearResidualTolerance_ scale).
    // Used only as a single-point threshold test -- is this one candidate already
    // physically converged? -- never to compare two different candidates against
    // each other, so unlike residualNorm's line-search use there is no
    // descent-direction guarantee to preserve, and the fact that its per-row scale
    // depends on x is harmless.
    float scaledResidualNorm(const std::vector<float>& x) const noexcept {
        float worst = 0.0f;
        for (std::size_t row = 0; row < dimension_; ++row) {
            double sum = -static_cast<double>(rhs_[row]);
            double scale = std::abs(static_cast<double>(rhs_[row]));
            const std::size_t base = row * dimension_;
            for (std::size_t slot = residualRowOffsets_[row];
                 slot < residualRowOffsets_[row + 1U]; ++slot) {
                const auto col = residualColumns_[slot];
                const double term = static_cast<double>(matrix_[base + col]) * x[col];
                sum += term;
                scale += std::abs(term);
            }
            const double backwardError = std::abs(sum) / std::max(1.0e-12, scale);
            worst = std::max(worst, static_cast<float>(backwardError));
        }
        return worst;
    }

    void stampNonlinear(const std::vector<float>& guess,
                        bool reuseJunctionVoltage) noexcept {
        for (auto& d : diodes_) {
            const float v = nodeVoltage(guess, d.anode) - nodeVoltage(guess, d.cathode);
            const auto linear = linearizeDiode(d, v, reuseJunctionVoltage);
            const float g = std::max(1.0e-12f, linear.conductance);
            const float iEq = linear.current - g * v;
            stampConductanceCompiled(d.conductanceLayout, g);
            stampCurrentSourceCompiled(d.conductanceLayout, iEq);
        }
        for (const auto& bjt : bjts_) stampBjt(bjt, guess);
        for (const auto& jfet : jfets_) stampJfet(jfet, guess);
        for (const auto& mosfet : mosfets_) stampMosfet(mosfet, guess);
        for (const auto& triode : triodes_) stampTriode(triode, guess);
        for (const auto& pentode : pentodes_) stampPentode(pentode, guess);
    }

    void markPattern(std::vector<std::uint8_t>& pattern, std::size_t row,
                     std::size_t column) const noexcept {
        if (row < dimension_ && column < dimension_)
            pattern[row * dimension_ + column] = 1U;
    }

    void markConductancePattern(std::vector<std::uint8_t>& pattern, Node a, Node b) const noexcept {
        if (a != ground) markPattern(pattern, nodeIndex(a), nodeIndex(a));
        if (b != ground) markPattern(pattern, nodeIndex(b), nodeIndex(b));
        if (a != ground && b != ground) {
            markPattern(pattern, nodeIndex(a), nodeIndex(b));
            markPattern(pattern, nodeIndex(b), nodeIndex(a));
        }
    }

    void markBranchPattern(std::vector<std::uint8_t>& pattern, Node positive,
                           Node negative, std::size_t branchIndex) const noexcept {
        if (positive != ground) {
            markPattern(pattern, nodeIndex(positive), branchIndex);
            markPattern(pattern, branchIndex, nodeIndex(positive));
        }
        if (negative != ground) {
            markPattern(pattern, nodeIndex(negative), branchIndex);
            markPattern(pattern, branchIndex, nodeIndex(negative));
        }
    }

    void markVccsPattern(std::vector<std::uint8_t>& pattern, Node outputPositive,
                         Node outputNegative, Node controlPositive, Node controlNegative) const noexcept {
        if (outputPositive != ground) {
            if (controlPositive != ground) markPattern(pattern, nodeIndex(outputPositive), nodeIndex(controlPositive));
            if (controlNegative != ground) markPattern(pattern, nodeIndex(outputPositive), nodeIndex(controlNegative));
        }
        if (outputNegative != ground) {
            if (controlPositive != ground) markPattern(pattern, nodeIndex(outputNegative), nodeIndex(controlPositive));
            if (controlNegative != ground) markPattern(pattern, nodeIndex(outputNegative), nodeIndex(controlNegative));
        }
    }

    void markThreeTerminalPattern(std::vector<std::uint8_t>& pattern,
                                  Node a, Node b, Node c) const noexcept {
        const std::array<Node, 3> nodes{{a, b, c}};
        for (const auto rowNode : nodes) {
            if (rowNode == ground) continue;
            for (const auto columnNode : nodes) {
                if (columnNode != ground)
                    markPattern(pattern, nodeIndex(rowNode), nodeIndex(columnNode));
            }
        }
    }

    void markFourTerminalPattern(std::vector<std::uint8_t>& pattern,
                                 Node a, Node b, Node c, Node d) const noexcept {
        const std::array<Node, 4> nodes{{a, b, c, d}};
        for (const auto rowNode : nodes) {
            if (rowNode == ground) continue;
            for (const auto columnNode : nodes) {
                if (columnNode != ground)
                    markPattern(pattern, nodeIndex(rowNode), nodeIndex(columnNode));
            }
        }
    }

    bool prepareSparseNonlinearSolver() {
        std::vector<std::uint8_t> pattern(dimension_ * dimension_, 0U);
        std::vector<std::uint8_t> nonlinearPattern(dimension_ * dimension_, 0U);

        for (const auto& r : resistors_) markConductancePattern(pattern, r.a, r.b);
        for (const auto& p : potentiometers_) {
            markConductancePattern(pattern, p.high, p.wiper);
            markConductancePattern(pattern, p.wiper, p.low);
        }
        for (const auto& c : capacitors_) markConductancePattern(pattern, c.a, c.b);
        for (const auto& source : voltageSources_)
            markBranchPattern(pattern, source.positive, source.negative, source.branchIndex);
        for (const auto& source : vccs_)
            markVccsPattern(pattern, source.outputPositive, source.outputNegative,
                            source.controlPositive, source.controlNegative);
        for (const auto& source : vcvs_) {
            markBranchPattern(pattern, source.outputPositive, source.outputNegative, source.branchIndex);
            if (source.controlPositive != ground)
                markPattern(pattern, source.branchIndex, nodeIndex(source.controlPositive));
            if (source.controlNegative != ground)
                markPattern(pattern, source.branchIndex, nodeIndex(source.controlNegative));
        }
        for (const auto& source : cccs_) {
            const auto controlBranch = voltageSourceBranch(source.controlSource);
            if (controlBranch >= dimension_) continue;
            if (source.outputPositive != ground)
                markPattern(pattern, nodeIndex(source.outputPositive), controlBranch);
            if (source.outputNegative != ground)
                markPattern(pattern, nodeIndex(source.outputNegative), controlBranch);
        }
        for (const auto& source : ccvs_) {
            const auto controlBranch = voltageSourceBranch(source.controlSource);
            markBranchPattern(pattern, source.outputPositive, source.outputNegative, source.branchIndex);
            if (controlBranch < dimension_)
                markPattern(pattern, source.branchIndex, controlBranch);
        }
        for (const auto& opAmp : opAmps_) {
            markBranchPattern(pattern, opAmp.output, opAmp.reference, opAmp.branchIndex);
            if (opAmp.nonInverting != ground)
                markPattern(pattern, opAmp.branchIndex, nodeIndex(opAmp.nonInverting));
            if (opAmp.inverting != ground)
                markPattern(pattern, opAmp.branchIndex, nodeIndex(opAmp.inverting));
        }
        for (const auto& l : inductors_) {
            markBranchPattern(pattern, l.a, l.b, l.branchIndex);
            markPattern(pattern, l.branchIndex, l.branchIndex);
        }

        for (const auto& d : diodes_) {
            markConductancePattern(pattern, d.anode, d.cathode);
            markConductancePattern(nonlinearPattern, d.anode, d.cathode);
        }
        for (const auto& bjt : bjts_) {
            markThreeTerminalPattern(pattern, bjt.collector, bjt.base, bjt.emitter);
            markThreeTerminalPattern(nonlinearPattern, bjt.collector, bjt.base, bjt.emitter);
        }
        for (const auto& jfet : jfets_) {
            markThreeTerminalPattern(pattern, jfet.drain, jfet.gate, jfet.source);
            markThreeTerminalPattern(nonlinearPattern, jfet.drain, jfet.gate, jfet.source);
        }
        for (const auto& mosfet : mosfets_) {
            markThreeTerminalPattern(pattern, mosfet.drain, mosfet.gate, mosfet.source);
            markThreeTerminalPattern(nonlinearPattern, mosfet.drain, mosfet.gate, mosfet.source);
        }
        for (const auto& triode : triodes_) {
            markThreeTerminalPattern(pattern, triode.plate, triode.grid, triode.cathode);
            markThreeTerminalPattern(nonlinearPattern, triode.plate, triode.grid, triode.cathode);
        }
        for (const auto& pentode : pentodes_) {
            markFourTerminalPattern(pattern, pentode.plate, pentode.grid, pentode.screen, pentode.cathode);
            markFourTerminalPattern(nonlinearPattern, pentode.plate, pentode.grid, pentode.screen, pentode.cathode);
        }

        // residualNorm()/scaledResidualNorm() are called up to several dozen
        // times per audio sample (once per Newton iteration and once per
        // backtracking line-search trial). The circuit's structural sparsity
        // pattern above is exactly the set of matrix_ cells any stamp can ever
        // touch and does not depend on Newton's current guess, so it is
        // compiled once here into a per-row nonzero-column list instead of
        // scanning and zero-testing all dimension_^2 cells on every call.
        // Cells outside this pattern are always exactly 0.0f and contributed
        // nothing to the previous dense scan either, so this is bit-identical.
        residualRowOffsets_.assign(dimension_ + 1U, 0U);
        residualColumns_.clear();
        residualColumns_.reserve(pattern.size());
        for (std::size_t row = 0; row < dimension_; ++row) {
            residualRowOffsets_[row] = residualColumns_.size();
            for (std::size_t column = 0; column < dimension_; ++column) {
                if (pattern[row * dimension_ + column] != 0U)
                    residualColumns_.push_back(column);
            }
        }
        residualRowOffsets_[dimension_] = residualColumns_.size();

        nonlinearMatrixIndices_.clear();
        nonlinearMatrixIndices_.reserve(pattern.size());
        nonlinearRhsIndices_.clear();
        nonlinearRhsIndices_.reserve(dimension_);
        std::size_t previousNonlinearRow = dimension_;
        for (std::size_t index = 0; index < nonlinearPattern.size(); ++index) {
            if (nonlinearPattern[index] != 0U) {
                nonlinearMatrixIndices_.push_back(index);
                const auto row = index / dimension_;
                if (row != previousNonlinearRow) {
                    nonlinearRhsIndices_.push_back(row);
                    previousNonlinearRow = row;
                }
            }
        }

        return sparseSolver_.prepare(dimension_, pattern, staticMatrix_, nonlinearPattern);
    }

    bool factorizeStaticLinearSystem() noexcept {
        linearLu_ = staticMatrix_;
        constexpr float pivotFloor = 1.0e-14f;
        for (std::size_t col = 0; col < dimension_; ++col) {
            std::size_t pivot = col;
            float largest = std::abs(linearLu_[col * dimension_ + col]);
            for (std::size_t row = col + 1U; row < dimension_; ++row) {
                const float value = std::abs(linearLu_[row * dimension_ + col]);
                if (value > largest) { largest = value; pivot = row; }
            }
            if (largest < pivotFloor || !std::isfinite(largest)) return false;
            linearPivots_[col] = pivot;
            if (pivot != col) {
                for (std::size_t c = 0; c < dimension_; ++c)
                    std::swap(linearLu_[col * dimension_ + c], linearLu_[pivot * dimension_ + c]);
            }
            const float diagonal = linearLu_[col * dimension_ + col];
            for (std::size_t row = col + 1U; row < dimension_; ++row) {
                const float factor = linearLu_[row * dimension_ + col] / diagonal;
                linearLu_[row * dimension_ + col] = factor;
                for (std::size_t c = col + 1U; c < dimension_; ++c)
                    linearLu_[row * dimension_ + c] -= factor * linearLu_[col * dimension_ + c];
            }
        }
        ++performanceStats_.fullFactorizations;
        return true;
    }

    bool solveCachedLinearSystem() noexcept {
        workRhs_ = rhs_;
        constexpr float pivotFloor = 1.0e-14f;

        for (std::size_t col = 0; col < dimension_; ++col) {
            const auto pivot = linearPivots_[col];
            if (pivot != col) std::swap(workRhs_[col], workRhs_[pivot]);
        }
        for (std::size_t row = 0; row < dimension_; ++row) {
            float sum = workRhs_[row];
            for (std::size_t c = 0; c < row; ++c)
                sum -= linearLu_[row * dimension_ + c] * workRhs_[c];
            workRhs_[row] = sum;
        }
        for (std::size_t ri = dimension_; ri-- > 0U;) {
            float sum = workRhs_[ri];
            for (std::size_t c = ri + 1U; c < dimension_; ++c)
                sum -= linearLu_[ri * dimension_ + c] * solution_[c];
            const float diagonal = linearLu_[ri * dimension_ + ri];
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return false;
            solution_[ri] = sum / diagonal;
            if (!std::isfinite(solution_[ri])) return false;
        }
        ++performanceStats_.cachedLinearSolves;
        return true;
    }

    bool solveGeneralLinearSystem() noexcept {
        workMatrix_ = matrix_;
        workRhs_ = rhs_;
        constexpr float pivotFloor = 1.0e-14f;

        for (std::size_t col = 0; col < dimension_; ++col) {
            std::size_t pivot = col;
            float largest = std::abs(workMatrix_[col * dimension_ + col]);
            for (std::size_t row = col + 1U; row < dimension_; ++row) {
                const float value = std::abs(workMatrix_[row * dimension_ + col]);
                if (value > largest) { largest = value; pivot = row; }
            }
            if (largest < pivotFloor || !std::isfinite(largest)) return false;
            if (pivot != col) {
                for (std::size_t c = col; c < dimension_; ++c)
                    std::swap(workMatrix_[col * dimension_ + c], workMatrix_[pivot * dimension_ + c]);
                std::swap(workRhs_[col], workRhs_[pivot]);
            }

            const float diagonal = workMatrix_[col * dimension_ + col];
            for (std::size_t row = col + 1U; row < dimension_; ++row) {
                const float factor = workMatrix_[row * dimension_ + col] / diagonal;
                if (factor == 0.0f) continue;
                workMatrix_[row * dimension_ + col] = 0.0f;
                for (std::size_t c = col + 1U; c < dimension_; ++c)
                    workMatrix_[row * dimension_ + c] -= factor * workMatrix_[col * dimension_ + c];
                workRhs_[row] -= factor * workRhs_[col];
            }
        }

        for (std::size_t ri = dimension_; ri-- > 0U;) {
            float sum = workRhs_[ri];
            for (std::size_t c = ri + 1U; c < dimension_; ++c)
                sum -= workMatrix_[ri * dimension_ + c] * solution_[c];
            const float diagonal = workMatrix_[ri * dimension_ + ri];
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return false;
            solution_[ri] = sum / diagonal;
            if (!std::isfinite(solution_[ri])) return false;
        }
        ++performanceStats_.generalLinearSolves;
        return true;
    }

    void updateDynamicState() noexcept {
        for (auto& c : capacitors_) {
            const float v = voltage(c.a, c.b);
            const float i = c.companionConductance * (v - c.previousVoltage) - c.previousCurrent;
            c.previousVoltage = v;
            c.previousCurrent = i;
        }
        for (auto& l : inductors_) {
            const float v = voltage(l.a, l.b);
            const float i = l.branchIndex < solution_.size() ? solution_[l.branchIndex] : 0.0f;
            l.previousVoltage = v;
            l.previousCurrent = i;
        }
    }

    std::size_t nodeCount_ = 0;
    std::size_t dimension_ = 0;
    double sampleRate_ = 48000.0;
    bool prepared_ = false;
    bool staticCacheDirty_ = true;
    bool dcAnalysisActive_ = false;
    bool linearFactorValid_ = false;
    NonlinearSolverMode nonlinearSolverMode_ = NonlinearSolverMode::automatic;
    double nonlinearResidualTolerance_ = 3.0e-7;
    SolveStats lastStats_{};
    PerformanceStats performanceStats_{};

    std::vector<Resistor> resistors_;
    std::vector<Capacitor> capacitors_;
    std::vector<Inductor> inductors_;
    std::vector<Potentiometer> potentiometers_;
    std::vector<CurrentSource> currentSources_;
    std::vector<VoltageSource> voltageSources_;
    std::vector<Vccs> vccs_;
    std::vector<Vcvs> vcvs_;
    std::vector<Cccs> cccs_;
    std::vector<Ccvs> ccvs_;
    std::vector<Diode> diodes_;
    std::vector<Bjt> bjts_;
    std::vector<Jfet> jfets_;
    std::vector<Mosfet> mosfets_;
    std::vector<OpAmp> opAmps_;
    std::vector<Triode> triodes_;
    std::vector<Pentode> pentodes_;

    std::vector<float> staticMatrix_;
    std::vector<float> staticRhs_;
    std::vector<float> sampleRhs_;
    std::vector<float> matrix_;
    std::vector<float> rhs_;
    std::vector<float> solution_;
    std::vector<float> candidate_;
    std::vector<float> lineSearchCandidate_;
    std::vector<float> previousSampleSolution_;
    std::vector<float> workMatrix_;
    std::vector<float> workRhs_;
    std::vector<float> linearLu_;
    std::vector<std::size_t> linearPivots_;
    std::vector<std::size_t> nonlinearMatrixIndices_;
    std::vector<std::size_t> nonlinearRhsIndices_;
    std::vector<std::size_t> residualRowOffsets_;
    std::vector<std::size_t> residualColumns_;
    std::vector<std::uint8_t> fixedVoltageNodes_;
    bool previousSampleSolutionValid_ = false;
    FixedPatternSparseSolver sparseSolver_;
};

} // namespace guitardsp::circuit
