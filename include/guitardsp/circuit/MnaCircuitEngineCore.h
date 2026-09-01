#pragma once

#include "FixedPatternSparseSolver.h"
#include "guitardsp/hq/ComponentCatalog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

        const std::size_t matrixSize = dimension_ * dimension_;
        staticMatrix_.assign(matrixSize, 0.0f);
        staticRhs_.assign(dimension_, 0.0f);
        sampleRhs_.assign(dimension_, 0.0f);
        matrix_.assign(matrixSize, 0.0f);
        rhs_.assign(dimension_, 0.0f);
        solution_.assign(dimension_, 0.0f);
        candidate_.assign(dimension_, 0.0f);
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

            if (iteration > 0 && usesPreparedSparse
                && (candidateFromFullSparseSolve
                    ? sparseSolver_.validateNonlinear(matrix_, rhs_, candidate_,
                        nonlinearResidualTolerance_)
                    : sparseSolver_.validate(matrix_, rhs_, candidate_,
                        nonlinearResidualTolerance_))) {
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
                const float damping = iteration < 3 ? 0.65f : 0.85f;
                for (std::size_t i = 0; i < dimension_; ++i) {
                    const float rawDelta = solution_[i] - candidate_[i];
                    if (i < nodeCount_) {
                        if (i < fixedVoltageNodes_.size() && fixedVoltageNodes_[i] != 0U) {
                            candidate_[i] = solution_[i];
                            continue;
                        }
                        const float scale = std::max(1.0f, std::abs(candidate_[i]));
                        const float maximumStep = std::clamp(0.25f * scale, 0.25f, 25.0f);
                        candidate_[i] += damping * std::clamp(rawDelta, -maximumStep, maximumStep);
                    } else {
                        candidate_[i] += damping * rawDelta;
                    }
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
    };
    struct Bjt { Node collector{}, base{}, emitter{}; hq::BJTSpec spec{}; };
    struct Jfet { Node drain{}, gate{}, source{}; hq::JFETSpec spec{}; };
    struct Mosfet { Node drain{}, gate{}, source{}; hq::MOSFETSpec spec{}; };
    struct OpAmp {
        Node output{}, nonInverting{}, inverting{}, reference{};
        hq::OpAmpSpec spec{};
        std::size_t branchIndex = 0;
    };
    struct Triode { Node plate{}, grid{}, cathode{}; hq::TriodeSpec spec{}; };

    struct JacobianTerm { Node node{}; float derivative = 0.0f; };
    struct DiodeLinearization { float current = 0.0f; float conductance = 0.0f; };

    static std::size_t nodeIndex(Node node) noexcept {
        return static_cast<std::size_t>(node - 1U);
    }

    float nodeVoltage(const std::vector<float>& x, Node node) const noexcept {
        if (node == ground) return 0.0f;
        const auto i = nodeIndex(node);
        return i < x.size() ? x[i] : 0.0f;
    }

    bool hasNonlinearDevices() const noexcept {
        return !diodes_.empty() || !bjts_.empty() || !jfets_.empty() ||
               !mosfets_.empty() || !triodes_.empty();
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
        const float dt = 1.0f / static_cast<float>(sampleRate_);

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

    template <std::size_t N>
    void stampLinearizedCurrent(Node positive, Node negative, float currentAtGuess,
                                const std::vector<float>& guess,
                                const std::array<JacobianTerm, N>& jacobian) noexcept {
        float equivalentCurrent = currentAtGuess;
        for (const auto& term : jacobian)
            equivalentCurrent -= term.derivative * nodeVoltage(guess, term.node);

        if (positive != ground) {
            const auto row = nodeIndex(positive);
            for (const auto& term : jacobian)
                if (term.node != ground) addMatrix(matrix_, row, nodeIndex(term.node), term.derivative);
        }
        if (negative != ground) {
            const auto row = nodeIndex(negative);
            for (const auto& term : jacobian)
                if (term.node != ground) addMatrix(matrix_, row, nodeIndex(term.node), -term.derivative);
        }
        stampCurrentSource(rhs_, positive, negative, equivalentCurrent);
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
        stampLinearizedCurrent(device.collector, device.emitter, collectorCurrent, guess, collectorJac);

        const float beta = std::max(1.0f, device.spec.beta);
        const float baseCurrent = collectorCurrent / beta;
        const std::array<JacobianTerm, 3> baseJac{{
            {device.collector, dCollectorDvce / beta},
            {device.base, dCollectorDvbe / beta},
            {device.emitter, -(dCollectorDvce + dCollectorDvbe) / beta}
        }};
        stampLinearizedCurrent(device.base, device.emitter, baseCurrent, guess, baseJac);
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
        stampLinearizedCurrent(device.drain, device.source, current, guess, jac);
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
        stampLinearizedCurrent(device.drain, device.source, current, guess, jac);
    }

    void stampTriode(const Triode& device, const std::vector<float>& guess) noexcept {
        const float vp = nodeVoltage(guess, device.plate);
        const float vg = nodeVoltage(guess, device.grid);
        const float vk = nodeVoltage(guess, device.cathode);
        const float vpk = vp - vk;
        const float vgk = vg - vk;

        const auto currentFor = [&](float gridToCathode, float plateToCathode) noexcept {
            return std::clamp(device.spec.model.plateCurrent(gridToCathode, plateToCathode), 0.0f, 0.20f);
        };

        const float current = currentFor(vgk, vpk);
        constexpr float gridStep = 1.0e-3f;
        const float plateStep = std::max(0.05f, std::abs(vpk) * 1.0e-4f);
        const float gm = (currentFor(vgk + gridStep, vpk) - currentFor(vgk - gridStep, vpk)) /
                         (2.0f * gridStep);
        const float gp = (currentFor(vgk, vpk + plateStep) - currentFor(vgk, vpk - plateStep)) /
                         (2.0f * plateStep);

        const float safeGm = std::clamp(gm, -1.0f, 1.0f);
        const float safeGp = std::clamp(gp, -1.0f, 1.0f);
        const std::array<JacobianTerm, 3> jac{{
            {device.plate, safeGp},
            {device.grid, safeGm},
            {device.cathode, -(safeGp + safeGm)}
        }};
        stampLinearizedCurrent(device.plate, device.cathode, current, guess, jac);
    }

    void stampNonlinear(const std::vector<float>& guess,
                        bool reuseJunctionVoltage) noexcept {
        for (auto& d : diodes_) {
            const float v = nodeVoltage(guess, d.anode) - nodeVoltage(guess, d.cathode);
            const auto linear = linearizeDiode(d, v, reuseJunctionVoltage);
            const float g = std::max(1.0e-12f, linear.conductance);
            const float iEq = linear.current - g * v;
            stampConductance(matrix_, d.anode, d.cathode, g);
            stampCurrentSource(rhs_, d.anode, d.cathode, iEq);
        }
        for (const auto& bjt : bjts_) stampBjt(bjt, guess);
        for (const auto& jfet : jfets_) stampJfet(jfet, guess);
        for (const auto& mosfet : mosfets_) stampMosfet(mosfet, guess);
        for (const auto& triode : triodes_) stampTriode(triode, guess);
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

    std::vector<float> staticMatrix_;
    std::vector<float> staticRhs_;
    std::vector<float> sampleRhs_;
    std::vector<float> matrix_;
    std::vector<float> rhs_;
    std::vector<float> solution_;
    std::vector<float> candidate_;
    std::vector<float> previousSampleSolution_;
    std::vector<float> workMatrix_;
    std::vector<float> workRhs_;
    std::vector<float> linearLu_;
    std::vector<std::size_t> linearPivots_;
    std::vector<std::size_t> nonlinearMatrixIndices_;
    std::vector<std::size_t> nonlinearRhsIndices_;
    std::vector<std::uint8_t> fixedVoltageNodes_;
    bool previousSampleSolutionValid_ = false;
    FixedPatternSparseSolver sparseSolver_;
};

} // namespace guitardsp::circuit
