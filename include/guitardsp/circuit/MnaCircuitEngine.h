#pragma once

#include "guitardsp/hq/ComponentCatalog.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace guitardsp::circuit {

using Node = std::uint16_t;
using SourceHandle = std::uint16_t;
inline constexpr Node ground = 0;

// Realtime-oriented Modified Nodal Analysis core.
// Topology is built/prepared on the control thread. processSample() performs no
// allocation and supports R/C/L, ideal voltage/current sources and Shockley diodes.
// Capacitors and inductors use trapezoidal companion models. Diodes are solved by
// Newton iteration on the complete MNA system.
class MnaCircuitEngine {
public:
    struct SolveStats {
        int iterations = 0;
        bool converged = true;
        bool singular = false;
    };

    Node addNode() {
        prepared_ = false;
        return static_cast<Node>(++nodeCount_);
    }

    void addResistor(Node a, Node b, hq::ResistorSpec spec) {
        prepared_ = false;
        resistors_.push_back({a, b, spec});
    }

    void addCapacitor(Node a, Node b, hq::CapacitorSpec spec) {
        prepared_ = false;
        capacitors_.push_back({a, b, spec, 0.0f, 0.0f});
    }

    void addInductor(Node a, Node b, hq::InductorSpec spec) {
        prepared_ = false;
        inductors_.push_back({a, b, spec, 0.0f, 0.0f, 0});
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

    void addDiode(Node anode, Node cathode, hq::DiodeSpec spec) {
        prepared_ = false;
        diodes_.push_back({anode, cathode, spec});
    }

    bool setVoltageSource(SourceHandle handle, float volts) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= voltageSources_.size()) return false;
        voltageSources_[i].volts = volts;
        return true;
    }

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        const std::size_t nodeUnknowns = nodeCount_;
        std::size_t branch = nodeUnknowns;
        for (auto& source : voltageSources_) source.branchIndex = branch++;
        for (auto& inductor : inductors_) inductor.branchIndex = branch++;
        dimension_ = branch;
        if (dimension_ == 0U) return false;

        matrix_.assign(dimension_ * dimension_, 0.0f);
        rhs_.assign(dimension_, 0.0f);
        solution_.assign(dimension_, 0.0f);
        candidate_.assign(dimension_, 0.0f);
        workMatrix_.assign(dimension_ * dimension_, 0.0f);
        workRhs_.assign(dimension_, 0.0f);
        reset();
        prepared_ = true;
        return true;
    }

    void reset() noexcept {
        std::fill(solution_.begin(), solution_.end(), 0.0f);
        std::fill(candidate_.begin(), candidate_.end(), 0.0f);
        for (auto& c : capacitors_) {
            c.previousVoltage = 0.0f;
            c.previousCurrent = 0.0f;
        }
        for (auto& l : inductors_) {
            l.previousVoltage = 0.0f;
            l.previousCurrent = 0.0f;
        }
        lastStats_ = {};
    }

    SolveStats processSample(int maximumNewtonIterations = 10, float tolerance = 1.0e-6f) noexcept {
        SolveStats stats{};
        if (!prepared_) {
            stats.converged = false;
            stats.singular = true;
            lastStats_ = stats;
            return stats;
        }

        maximumNewtonIterations = std::clamp(maximumNewtonIterations, 1, 32);
        tolerance = std::max(1.0e-9f, tolerance);
        candidate_ = solution_;
        stats.converged = diodes_.empty();

        for (int iteration = 0; iteration < maximumNewtonIterations; ++iteration) {
            assemble(candidate_);
            if (!solveLinearSystem()) {
                stats.singular = true;
                stats.converged = false;
                stats.iterations = iteration + 1;
                lastStats_ = stats;
                return stats;
            }

            float maxDelta = 0.0f;
            for (std::size_t i = 0; i < dimension_; ++i)
                maxDelta = std::max(maxDelta, std::abs(solution_[i] - candidate_[i]));
            candidate_ = solution_;
            stats.iterations = iteration + 1;
            if (diodes_.empty() || maxDelta <= tolerance) {
                stats.converged = true;
                break;
            }
        }

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

private:
    struct Resistor { Node a{}, b{}; hq::ResistorSpec spec{}; };
    struct Capacitor {
        Node a{}, b{};
        hq::CapacitorSpec spec{};
        float previousVoltage = 0.0f;
        float previousCurrent = 0.0f;
    };
    struct Inductor {
        Node a{}, b{};
        hq::InductorSpec spec{};
        float previousVoltage = 0.0f;
        float previousCurrent = 0.0f;
        std::size_t branchIndex = 0;
    };
    struct CurrentSource { Node positive{}, negative{}; float amps = 0.0f; };
    struct VoltageSource { Node positive{}, negative{}; float volts = 0.0f; std::size_t branchIndex = 0; };
    struct Diode { Node anode{}, cathode{}; hq::DiodeSpec spec{}; };

    static std::size_t nodeIndex(Node node) noexcept {
        return static_cast<std::size_t>(node - 1U);
    }

    float nodeVoltage(const std::vector<float>& x, Node node) const noexcept {
        if (node == ground) return 0.0f;
        const auto i = nodeIndex(node);
        return i < x.size() ? x[i] : 0.0f;
    }

    void addMatrix(std::size_t row, std::size_t col, float value) noexcept {
        matrix_[row * dimension_ + col] += value;
    }

    void stampConductance(Node a, Node b, float conductance) noexcept {
        if (a != ground) addMatrix(nodeIndex(a), nodeIndex(a), conductance);
        if (b != ground) addMatrix(nodeIndex(b), nodeIndex(b), conductance);
        if (a != ground && b != ground) {
            addMatrix(nodeIndex(a), nodeIndex(b), -conductance);
            addMatrix(nodeIndex(b), nodeIndex(a), -conductance);
        }
    }

    void stampCurrentSource(Node positive, Node negative, float current) noexcept {
        if (positive != ground) rhs_[nodeIndex(positive)] -= current;
        if (negative != ground) rhs_[nodeIndex(negative)] += current;
    }

    void stampBranch(Node positive, Node negative, std::size_t branchIndex, float diagonal, float value) noexcept {
        if (positive != ground) {
            addMatrix(nodeIndex(positive), branchIndex, 1.0f);
            addMatrix(branchIndex, nodeIndex(positive), 1.0f);
        }
        if (negative != ground) {
            addMatrix(nodeIndex(negative), branchIndex, -1.0f);
            addMatrix(branchIndex, nodeIndex(negative), -1.0f);
        }
        addMatrix(branchIndex, branchIndex, diagonal);
        rhs_[branchIndex] += value;
    }

    struct DiodeLinearization { float current = 0.0f; float conductance = 0.0f; };

    static DiodeLinearization linearizeDiode(const hq::DiodeSpec& spec, float terminalVoltage) noexcept {
        const auto junction = spec.toModel();
        const float rs = std::max(0.0f, spec.seriesResistanceOhms);
        float vj = terminalVoltage;
        for (int i = 0; i < 8; ++i) {
            const float current = junction.current(vj);
            const float gj = junction.conductance(vj);
            const float residual = vj + rs * current - terminalVoltage;
            const float derivative = 1.0f + rs * gj;
            const float step = residual / std::max(1.0e-12f, derivative);
            vj -= std::clamp(step, -0.5f, 0.5f);
            if (std::abs(step) < 1.0e-7f) break;
        }
        const float current = junction.current(vj);
        const float gj = junction.conductance(vj);
        return {current, gj / std::max(1.0e-12f, 1.0f + rs * gj)};
    }

    void assemble(const std::vector<float>& guess) noexcept {
        std::fill(matrix_.begin(), matrix_.end(), 0.0f);
        std::fill(rhs_.begin(), rhs_.end(), 0.0f);

        for (const auto& r : resistors_) {
            const float resistance = std::max(1.0e-6f, r.spec.resistanceOhms);
            stampConductance(r.a, r.b, 1.0f / resistance);
        }

        const float dt = 1.0f / static_cast<float>(sampleRate_);
        for (const auto& c : capacitors_) {
            const float capacitance = std::max(0.0f, c.spec.capacitanceFarads);
            if (capacitance <= 0.0f) continue;
            const float g = 2.0f * capacitance / dt;
            stampConductance(c.a, c.b, g);
            const float historyCurrent = -g * c.previousVoltage - c.previousCurrent;
            stampCurrentSource(c.a, c.b, historyCurrent);
            if (c.spec.leakageResistanceOhms > 0.0f && std::isfinite(c.spec.leakageResistanceOhms))
                stampConductance(c.a, c.b, 1.0f / std::max(1.0f, c.spec.leakageResistanceOhms));
            if (c.spec.esrOhms > 0.0f) {
                // ESR is deliberately not folded into this two-terminal companion yet;
                // model it as an explicit resistor when topology accuracy matters.
            }
        }

        for (const auto& source : currentSources_)
            stampCurrentSource(source.positive, source.negative, source.amps);

        for (const auto& source : voltageSources_)
            stampBranch(source.positive, source.negative, source.branchIndex, 0.0f, source.volts);

        for (const auto& l : inductors_) {
            const float inductance = std::max(1.0e-12f, l.spec.inductanceHenries);
            const float rEq = 2.0f * inductance / dt + std::max(0.0f, l.spec.seriesResistanceOhms);
            const float historyVoltage = -(2.0f * inductance / dt) * l.previousCurrent - l.previousVoltage;
            stampBranch(l.a, l.b, l.branchIndex, -rEq, historyVoltage);
        }

        for (const auto& d : diodes_) {
            const float v = nodeVoltage(guess, d.anode) - nodeVoltage(guess, d.cathode);
            const auto linear = linearizeDiode(d.spec, v);
            const float g = std::max(1.0e-12f, linear.conductance);
            const float iEq = linear.current - g * v;
            stampConductance(d.anode, d.cathode, g);
            stampCurrentSource(d.anode, d.cathode, iEq);
        }
    }

    bool solveLinearSystem() noexcept {
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
        return true;
    }

    void updateDynamicState() noexcept {
        const float dt = 1.0f / static_cast<float>(sampleRate_);
        for (auto& c : capacitors_) {
            const float v = voltage(c.a, c.b);
            const float g = 2.0f * std::max(0.0f, c.spec.capacitanceFarads) / dt;
            const float i = g * (v - c.previousVoltage) - c.previousCurrent;
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
    SolveStats lastStats_{};

    std::vector<Resistor> resistors_;
    std::vector<Capacitor> capacitors_;
    std::vector<Inductor> inductors_;
    std::vector<CurrentSource> currentSources_;
    std::vector<VoltageSource> voltageSources_;
    std::vector<Diode> diodes_;

    std::vector<float> matrix_;
    std::vector<float> rhs_;
    std::vector<float> solution_;
    std::vector<float> candidate_;
    std::vector<float> workMatrix_;
    std::vector<float> workRhs_;
};

} // namespace guitardsp::circuit
