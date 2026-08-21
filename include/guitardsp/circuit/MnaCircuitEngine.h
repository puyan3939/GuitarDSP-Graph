#pragma once

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

// Realtime-oriented Modified Nodal Analysis core.
//
// Topology is built/prepared on the control thread. processSample() performs no
// allocation and reuses fixed matrix/vector storage. The engine is deliberately a
// dense correctness/reference solver for small circuit islands; a later compiled
// fixed-pattern/sparse backend can preserve the same netlist contract.
//
// Supported stamps:
// - editable R / C / L and ideal current/voltage sources
// - three-terminal potentiometers with editable taper/position
// - VCCS / VCVS / CCCS / CCVS controlled sources
// - Shockley diodes with series resistance
// - engineering BJT, JFET and MOSFET nonlinear three-terminal stamps
// - finite-open-loop-gain op-amp macro stamps
// - nonlinear plate/grid/cathode triode stamps
//
// Capacitors and inductors use trapezoidal companion models. Nonlinear devices
// are linearized into the complete MNA system and solved by Newton iteration.
// Component setters do not rebuild topology, but callers must serialize writes
// against processSample(); a lock-free block-boundary command queue is a separate
// graph/runtime responsibility.
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

    ResistorHandle addResistor(Node a, Node b, hq::ResistorSpec spec) {
        prepared_ = false;
        resistors_.push_back({a, b, spec});
        return static_cast<ResistorHandle>(resistors_.size() - 1U);
    }

    CapacitorHandle addCapacitor(Node a, Node b, hq::CapacitorSpec spec) {
        prepared_ = false;
        capacitors_.push_back({a, b, spec, 0.0f, 0.0f});
        return static_cast<CapacitorHandle>(capacitors_.size() - 1U);
    }

    InductorHandle addInductor(Node a, Node b, hq::InductorSpec spec) {
        prepared_ = false;
        inductors_.push_back({a, b, spec, 0.0f, 0.0f, 0});
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

    ControlledSourceHandle addVccs(Node outputPositive,
                                   Node outputNegative,
                                   Node controlPositive,
                                   Node controlNegative,
                                   float transconductanceSiemens) {
        prepared_ = false;
        vccs_.push_back({outputPositive, outputNegative, controlPositive, controlNegative,
                         transconductanceSiemens});
        return static_cast<ControlledSourceHandle>(vccs_.size() - 1U);
    }

    ControlledSourceHandle addVcvs(Node outputPositive,
                                   Node outputNegative,
                                   Node controlPositive,
                                   Node controlNegative,
                                   float voltageGain) {
        prepared_ = false;
        vcvs_.push_back({outputPositive, outputNegative, controlPositive, controlNegative,
                         voltageGain, 0});
        return static_cast<ControlledSourceHandle>(vcvs_.size() - 1U);
    }

    ControlledSourceHandle addCccs(Node outputPositive,
                                   Node outputNegative,
                                   SourceHandle controlVoltageSource,
                                   float currentGain) {
        prepared_ = false;
        cccs_.push_back({outputPositive, outputNegative, controlVoltageSource, currentGain});
        return static_cast<ControlledSourceHandle>(cccs_.size() - 1U);
    }

    ControlledSourceHandle addCcvs(Node outputPositive,
                                   Node outputNegative,
                                   SourceHandle controlVoltageSource,
                                   float transresistanceOhms) {
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

    OpAmpHandle addOpAmp(Node output,
                         Node nonInverting,
                         Node inverting,
                         Node reference,
                         hq::OpAmpSpec spec) {
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
        return true;
    }

    bool setResistance(ResistorHandle handle, float ohms) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= resistors_.size()) return false;
        resistors_[i].spec.resistanceOhms = std::max(1.0e-6f, ohms);
        return true;
    }

    bool setCapacitorSpec(CapacitorHandle handle, hq::CapacitorSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= capacitors_.size()) return false;
        capacitors_[i].spec = spec;
        return true;
    }

    bool setCapacitance(CapacitorHandle handle, float farads) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= capacitors_.size()) return false;
        capacitors_[i].spec.capacitanceFarads = std::max(0.0f, farads);
        return true;
    }

    bool setInductorSpec(InductorHandle handle, hq::InductorSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= inductors_.size()) return false;
        inductors_[i].spec = spec;
        return true;
    }

    bool setInductance(InductorHandle handle, float henries) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= inductors_.size()) return false;
        inductors_[i].spec.inductanceHenries = std::max(0.0f, henries);
        return true;
    }

    bool setPotentiometerPosition(PotHandle handle, float position) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= potentiometers_.size()) return false;
        potentiometers_[i].spec.position = std::clamp(position, 0.0f, 1.0f);
        return true;
    }

    bool setPotentiometerSpec(PotHandle handle, hq::PotentiometerSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= potentiometers_.size()) return false;
        potentiometers_[i].spec = spec;
        return true;
    }

    bool setVccsTransconductance(ControlledSourceHandle handle, float siemens) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= vccs_.size()) return false;
        vccs_[i].transconductance = siemens;
        return true;
    }

    bool setVcvsGain(ControlledSourceHandle handle, float gain) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= vcvs_.size()) return false;
        vcvs_[i].gain = gain;
        return true;
    }

    bool setCccsGain(ControlledSourceHandle handle, float gain) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= cccs_.size()) return false;
        cccs_[i].gain = gain;
        return true;
    }

    bool setCcvsTransresistance(ControlledSourceHandle handle, float ohms) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= ccvs_.size()) return false;
        ccvs_[i].transresistanceOhms = ohms;
        return true;
    }

    bool setDiodeSpec(DiodeHandle handle, hq::DiodeSpec spec) noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= diodes_.size()) return false;
        diodes_[i].spec = spec;
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

    SolveStats processSample(int maximumNewtonIterations = 12, float tolerance = 1.0e-6f) noexcept {
        SolveStats stats{};
        if (!prepared_) {
            stats.converged = false;
            stats.singular = true;
            lastStats_ = stats;
            return stats;
        }

        maximumNewtonIterations = std::clamp(maximumNewtonIterations, 1, 40);
        tolerance = std::max(1.0e-9f, tolerance);
        candidate_ = solution_;
        const bool nonlinear = hasNonlinearDevices();
        stats.converged = !nonlinear;

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

            if (nonlinear && iteration < 3) {
                constexpr float damping = 0.65f;
                for (std::size_t i = 0; i < dimension_; ++i)
                    candidate_[i] += damping * (solution_[i] - candidate_[i]);
            } else {
                candidate_ = solution_;
            }

            stats.iterations = iteration + 1;
            if (!nonlinear || maxDelta <= tolerance) {
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
    struct Diode { Node anode{}, cathode{}; hq::DiodeSpec spec{}; };
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
        return !diodes_.empty() || !bjts_.empty() || !jfets_.empty() || !mosfets_.empty() || !triodes_.empty();
    }

    std::size_t voltageSourceBranch(SourceHandle handle) const noexcept {
        const auto i = static_cast<std::size_t>(handle);
        if (i >= voltageSources_.size()) return dimension_;
        return voltageSources_[i].branchIndex;
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

    void stampVccs(Node outputPositive,
                   Node outputNegative,
                   Node controlPositive,
                   Node controlNegative,
                   float transconductance) noexcept {
        if (outputPositive != ground) {
            if (controlPositive != ground)
                addMatrix(nodeIndex(outputPositive), nodeIndex(controlPositive), transconductance);
            if (controlNegative != ground)
                addMatrix(nodeIndex(outputPositive), nodeIndex(controlNegative), -transconductance);
        }
        if (outputNegative != ground) {
            if (controlPositive != ground)
                addMatrix(nodeIndex(outputNegative), nodeIndex(controlPositive), -transconductance);
            if (controlNegative != ground)
                addMatrix(nodeIndex(outputNegative), nodeIndex(controlNegative), transconductance);
        }
    }

    void stampVcvs(const Vcvs& source) noexcept {
        const auto branch = source.branchIndex;
        if (source.outputPositive != ground) {
            addMatrix(nodeIndex(source.outputPositive), branch, 1.0f);
            addMatrix(branch, nodeIndex(source.outputPositive), 1.0f);
        }
        if (source.outputNegative != ground) {
            addMatrix(nodeIndex(source.outputNegative), branch, -1.0f);
            addMatrix(branch, nodeIndex(source.outputNegative), -1.0f);
        }
        if (source.controlPositive != ground)
            addMatrix(branch, nodeIndex(source.controlPositive), -source.gain);
        if (source.controlNegative != ground)
            addMatrix(branch, nodeIndex(source.controlNegative), source.gain);
    }

    void stampCccs(const Cccs& source) noexcept {
        const auto controlBranch = voltageSourceBranch(source.controlSource);
        if (controlBranch >= dimension_) return;
        if (source.outputPositive != ground)
            addMatrix(nodeIndex(source.outputPositive), controlBranch, source.gain);
        if (source.outputNegative != ground)
            addMatrix(nodeIndex(source.outputNegative), controlBranch, -source.gain);
    }

    void stampCcvs(const Ccvs& source) noexcept {
        const auto controlBranch = voltageSourceBranch(source.controlSource);
        const auto branch = source.branchIndex;
        if (source.outputPositive != ground) {
            addMatrix(nodeIndex(source.outputPositive), branch, 1.0f);
            addMatrix(branch, nodeIndex(source.outputPositive), 1.0f);
        }
        if (source.outputNegative != ground) {
            addMatrix(nodeIndex(source.outputNegative), branch, -1.0f);
            addMatrix(branch, nodeIndex(source.outputNegative), -1.0f);
        }
        if (controlBranch < dimension_)
            addMatrix(branch, controlBranch, -source.transresistanceOhms);
    }

    void stampOpAmp(const OpAmp& device) noexcept {
        const auto branch = device.branchIndex;
        const float gainDb = std::clamp(device.spec.openLoopGainDb, 0.0f, 120.0f);
        const float gain = std::pow(10.0f, gainDb / 20.0f);

        if (device.output != ground) {
            addMatrix(nodeIndex(device.output), branch, 1.0f);
            addMatrix(branch, nodeIndex(device.output), 1.0f);
        }
        if (device.reference != ground) {
            addMatrix(nodeIndex(device.reference), branch, -1.0f);
            addMatrix(branch, nodeIndex(device.reference), -1.0f);
        }
        if (device.nonInverting != ground)
            addMatrix(branch, nodeIndex(device.nonInverting), -gain);
        if (device.inverting != ground)
            addMatrix(branch, nodeIndex(device.inverting), gain);

        rhs_[branch] += gain * device.spec.inputOffsetVoltage;
    }

    template <std::size_t N>
    void stampLinearizedCurrent(Node positive,
                                Node negative,
                                float currentAtGuess,
                                const std::vector<float>& guess,
                                const std::array<JacobianTerm, N>& jacobian) noexcept {
        float equivalentCurrent = currentAtGuess;
        for (const auto& term : jacobian)
            equivalentCurrent -= term.derivative * nodeVoltage(guess, term.node);

        if (positive != ground) {
            const auto row = nodeIndex(positive);
            for (const auto& term : jacobian)
                if (term.node != ground) addMatrix(row, nodeIndex(term.node), term.derivative);
        }
        if (negative != ground) {
            const auto row = nodeIndex(negative);
            for (const auto& term : jacobian)
                if (term.node != ground) addMatrix(row, nodeIndex(term.node), -term.derivative);
        }
        stampCurrentSource(positive, negative, equivalentCurrent);
    }

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
        stampLinearizedCurrent(device.collector, device.emitter,
                               collectorCurrent, guess, collectorJac);

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
        const float modulation = 1.0f + std::max(0.0f, device.spec.lambda) * std::abs(vds);
        const float dModulationDvds = vds > 0.0f ? std::max(0.0f, device.spec.lambda)
                                     : vds < 0.0f ? -std::max(0.0f, device.spec.lambda)
                                                  : 0.0f;

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
        const float k = std::max(0.0f, device.spec.transconductance);
        const float lambda = std::max(0.0f, device.spec.lambda);

        float effectiveCurrent = 0.0f;
        float dIdDvgs = 0.0f;
        float dIdDvds = 0.0f;
        if (overdrive > 0.0f && vds > 0.0f) {
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
            return std::clamp(device.spec.model.plateCurrent(gridToCathode, plateToCathode),
                              0.0f, 0.20f);
        };

        const float current = currentFor(vgk, vpk);
        constexpr float gridStep = 1.0e-3f;
        const float plateStep = std::max(0.05f, std::abs(vpk) * 1.0e-4f);
        const float gm = (currentFor(vgk + gridStep, vpk) -
                          currentFor(vgk - gridStep, vpk)) / (2.0f * gridStep);
        const float gp = (currentFor(vgk, vpk + plateStep) -
                          currentFor(vgk, vpk - plateStep)) / (2.0f * plateStep);

        const float safeGm = std::clamp(gm, -1.0f, 1.0f);
        const float safeGp = std::clamp(gp, -1.0f, 1.0f);
        const std::array<JacobianTerm, 3> jac{{
            {device.plate, safeGp},
            {device.grid, safeGm},
            {device.cathode, -(safeGp + safeGm)}
        }};
        stampLinearizedCurrent(device.plate, device.cathode, current, guess, jac);
    }

    void assemble(const std::vector<float>& guess) noexcept {
        std::fill(matrix_.begin(), matrix_.end(), 0.0f);
        std::fill(rhs_.begin(), rhs_.end(), 0.0f);

        for (const auto& r : resistors_) {
            const float resistance = std::max(1.0e-6f, r.spec.resistanceOhms);
            stampConductance(r.a, r.b, 1.0f / resistance);
        }

        for (const auto& p : potentiometers_) {
            constexpr float contactFloorOhms = 1.0e-3f;
            const float total = std::max(2.0f * contactFloorOhms, p.spec.totalResistanceOhms);
            const float position = std::clamp(p.spec.normalizedElectricalPosition(), 0.0f, 1.0f);
            const float lowResistance = std::max(contactFloorOhms, total * position);
            const float highResistance = std::max(contactFloorOhms, total * (1.0f - position));
            stampConductance(p.high, p.wiper, 1.0f / highResistance);
            stampConductance(p.wiper, p.low, 1.0f / lowResistance);
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
        }

        for (const auto& source : currentSources_)
            stampCurrentSource(source.positive, source.negative, source.amps);

        for (const auto& source : voltageSources_)
            stampBranch(source.positive, source.negative, source.branchIndex, 0.0f, source.volts);

        for (const auto& source : vccs_)
            stampVccs(source.outputPositive, source.outputNegative,
                      source.controlPositive, source.controlNegative,
                      source.transconductance);

        for (const auto& source : vcvs_)
            stampVcvs(source);

        for (const auto& source : cccs_)
            stampCccs(source);

        for (const auto& source : ccvs_)
            stampCcvs(source);

        for (const auto& opAmp : opAmps_)
            stampOpAmp(opAmp);

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

        for (const auto& bjt : bjts_) stampBjt(bjt, guess);
        for (const auto& jfet : jfets_) stampJfet(jfet, guess);
        for (const auto& mosfet : mosfets_) stampMosfet(mosfet, guess);
        for (const auto& triode : triodes_) stampTriode(triode, guess);
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

    std::vector<float> matrix_;
    std::vector<float> rhs_;
    std::vector<float> solution_;
    std::vector<float> candidate_;
    std::vector<float> workMatrix_;
    std::vector<float> workRhs_;
};

} // namespace guitardsp::circuit
