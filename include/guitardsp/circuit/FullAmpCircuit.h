#pragma once

#include "PowerAmpCircuit.h"
#include "PreampCircuit.h"

#include <algorithm>

namespace guitardsp::circuit {

// Full guitar amplifier signal path: PreampCircuit's 12AX7 common-cathode
// gain stage + Bass/Treble tone stack cascaded into PowerAmpCircuit's EL34
// single-ended power stage.
//
// Design choice (see issue #41): the two stages stay as independent
// MnaCircuitEngine instances -- each keeps its own prepare()/
// primeOperatingPoint()/processSample() unchanged -- rather than being
// merged into a single combined netlist. Every sample, the preamp's output
// voltage is fed directly in as the power amp's input voltage
// (processSample() cascade), the same way a real preamp stage drives a power
// stage through a coupling cap.
//
// This mirrors how each stage is already built: PreampCircuit's output
// already terminates into a fixed 100k ohm load resistor (a stand-in for
// "whatever comes next"), and PowerAmpCircuit's grid-leak input network is a
// high-impedance load from the driving stage's point of view, so cascading
// processSample() calls is a physically reasonable approximation of a real
// two-stage amp chain. What it does *not* capture is true interstage
// loading -- e.g. the power amp's grid drawing current during hard positive
// grid conduction and sagging the preamp's plate voltage -- which only a
// single combined netlist could reproduce exactly.
//
// A unified single-engine netlist was considered and rejected for now: it
// would roughly double the Newton problem's unknown count (12AX7 + tone
// stack + EL34 pentode + output transformer in one system) and require
// co-priming two very different B+ rails (300 V / 420 V) from a cold start,
// which is new solver-convergence risk beyond what PreampCircuit/
// PowerAmpCircuit already have validated individually. The cascade keeps
// both stages' proven DC-priming schedules, solver tolerances and Newton
// behaviour exactly as-is.
class FullAmpCircuit {
public:
    struct StageVoltages {
        PreampCircuit::StageVoltages preamp;
        PowerAmpCircuit::StageVoltages powerAmp;
    };

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        if (!preamp_.prepare(sampleRate_)) return false;
        if (!powerAmp_.prepare(sampleRate_)) return false;
        return true;
    }

    void reset() noexcept {
        preamp_.reset();
        powerAmp_.reset();
    }

    bool setBass(float normalized) noexcept { return preamp_.setBass(normalized); }
    bool setTreble(float normalized) noexcept { return preamp_.setTreble(normalized); }
    bool setControls(float bass, float treble) noexcept { return preamp_.setControls(bass, treble); }

    float processSample(float input) noexcept {
        const float preampOut = preamp_.processSample(input);
        return powerAmp_.processSample(preampOut);
    }

    StageVoltages stageVoltages() const noexcept {
        return {preamp_.stageVoltages(), powerAmp_.stageVoltages()};
    }

    float bass() const noexcept { return preamp_.bass(); }
    float treble() const noexcept { return preamp_.treble(); }
    float appliedBass() const noexcept { return preamp_.appliedBass(); }
    float appliedTreble() const noexcept { return preamp_.appliedTreble(); }

    MnaCircuitEngine::SolveStats preampSolveStats() const noexcept { return preamp_.lastSolveStats(); }
    MnaCircuitEngine::SolveStats powerAmpSolveStats() const noexcept { return powerAmp_.lastSolveStats(); }

    const PreampCircuit& preamp() const noexcept { return preamp_; }
    PreampCircuit& preamp() noexcept { return preamp_; }
    const PowerAmpCircuit& powerAmp() const noexcept { return powerAmp_; }
    PowerAmpCircuit& powerAmp() noexcept { return powerAmp_; }

private:
    double sampleRate_ = 48000.0;
    PreampCircuit preamp_;
    PowerAmpCircuit powerAmp_;
};

} // namespace guitardsp::circuit
