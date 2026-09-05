#pragma once

#include "DiodeParasiticSubcircuit.h"
#include "DynamicOpAmpSubcircuit.h"
#include "guitardsp/hq/AdditionalDeviceStages.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace guitardsp::circuit {

// Component-level optical (LA-2A-style) compressor, built from ordinary MNA
// parts exactly like TS808Circuit/DS1Circuit/PreampCircuit/PowerAmpCircuit.
//
// Signal path:
//
//   input coupling cap -> 1M bias to vref -> 4k7 series resistor -> gain
//   cell node. An LDR (guitardsp::hq::OptocouplerLDR), modelled as an
//   ordinary variable resistor from the gain cell to vref, shunts the gain
//   cell to the mid-supply AC reference. With the LDR dark (~5 Mohm) the
//   4k7/5M divider is essentially transparent (~-0.01 dB); with the LDR
//   fully lit (~500 ohm) the 4k7/500 divider attenuates by roughly -20 dB.
//   This divider is the actual gain-reduction element -- everything else in
//   the signal path is unity-gain buffering.
//   gain cell -> coupling cap -> unity-gain output buffer
//   (DynamicOpAmpSubcircuit) -> output coupling cap -> output node.
//
// Sidechain (feedback, tapped from the output like a real LA-2A rather than
// from the input, so the loop compresses based on what actually left the
// gain cell):
//
//   output node -> coupling cap -> 1M bias to vref -> non-inverting input of
//   a DynamicOpAmpSubcircuit wired as a precision half-wave peak detector
//   (inverting input = envelope node, feedback through a single diode from
//   the op-amp's output to the envelope node). The diode only ever lets the
//   op-amp *source* current into the envelope node, so positive excursions
//   of the sidechain signal above the current envelope charge the envelope
//   node quickly (attack, limited mainly by the op-amp's slew rate); a
//   100k/1uF bleed network alone discharges the envelope node back toward
//   vref between peaks (release, RC = 100 ms).
//
// LDR update: every processSample(), after the Newton solve, the envelope
// node's voltage above vref is fed to OptocouplerLDR::processLedDrive(),
// and the resistance it returns is pushed into the gain-cell shunt resistor
// via MnaCircuitEngine::setResistance() -- the same "only push a spec
// change through when it actually moved more than float noise" pattern
// PowerAmpCircuit::updateOutputTransformerSaturation() uses for its output
// transformer's saturating magnetizing inductance, and for the same reason:
// setResistance() unconditionally dirties the engine's static matrix cache,
// so committing every sample even at idle would force a full rebuild every
// single sample for no audible benefit. Because the resistor is only
// updated *after* this sample's solve, the gain cell always sees the
// previous sample's envelope: a one-sample delay (~21 us at 48 kHz) that is
// negligible next to the LDR's own multi-millisecond attack/release time
// constants.
class CompressorCircuit {
public:
    struct StageVoltages {
        float gainCell = 0.0f;
        float envelope = 0.0f;
        float output = 0.0f;
    };

    // Scales the envelope voltage above vref (volts) into the LDR's
    // normalized [0,1] LED drive. Chosen so a guitar-level sidechain signal
    // (envelope on the order of a few hundred mV to ~1 V above vref) sweeps
    // the LDR through most of its dark-to-light range.
    static constexpr float sensitivityPerVolt = 2.5f;

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        engine_ = MnaCircuitEngine{};
        ldr_.prepare(sampleRate_, hq::OptocouplerSpec{});

        supply_ = engine_.addNode();
        vref_ = engine_.addNode();
        inputJack_ = engine_.addNode();
        const Node inputCoupled = engine_.addNode();
        gainCell_ = engine_.addNode();
        const Node bufferNonInv = engine_.addNode();
        const Node bufferOut = engine_.addNode();
        outputNode_ = engine_.addNode();
        const Node scCoupled = engine_.addNode();
        const Node envDrive = engine_.addNode();
        envNode_ = engine_.addNode();

        // Supply sources are initially zero; source-stepping continuation
        // establishes the DC operating point the same way TS808Circuit does.
        supplySource_ = engine_.addVoltageSource(supply_, ground, 0.0f);
        vrefSource_ = engine_.addVoltageSource(vref_, ground, 0.0f);
        inputSource_ = engine_.addVoltageSource(inputJack_, ground, 0.0f);

        // Main signal path: coupling -> bias -> series resistor -> LDR shunt
        // (gain cell) -> coupling -> unity buffer -> coupling -> output.
        // Coupling cap/bias resistor pairs are sized the same way TS808Circuit's
        // are (tens of ohm-farads giving a ~10-20 ms RC, not hundreds of ms), so
        // the DC operating point actually settles within prepare()'s warm-up
        // window instead of still visibly charging toward vref/ground.
        engine_.addCapacitor(inputJack_, inputCoupled,
                             capacitor(22.0e-9f, 25.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(inputCoupled, vref_, resistor(510000.0f));
        engine_.addResistor(inputCoupled, gainCell_, resistor(4700.0f));
        ldrResistor_ = engine_.addResistor(gainCell_, vref_, resistor(ldr_.resistanceOhms()));
        engine_.addCapacitor(gainCell_, bufferNonInv,
                             capacitor(0.1e-6f, 25.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(bufferNonInv, vref_, resistor(100000.0f));

        const auto opAmp = compressorOpAmp();
        bufferOpAmp_ = addDynamicOpAmpSubcircuit(
            engine_, bufferOut, bufferNonInv, bufferOut, supply_, ground, ground, opAmp);
        engine_.addCapacitor(bufferOut, outputNode_,
                             capacitor(10.0e-6f, 16.0f, hq::CapacitorTechnology::electrolytic));
        engine_.addResistor(outputNode_, ground, resistor(10000.0f));

        // Sidechain: tap the output, precision half-wave peak detector.
        engine_.addCapacitor(outputNode_, scCoupled,
                             capacitor(0.1e-6f, 25.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(scCoupled, vref_, resistor(100000.0f));
        peakDetector_ = addDynamicOpAmpSubcircuit(
            engine_, envDrive, scCoupled, envNode_, supply_, ground, ground, opAmp);

        auto envelopeDiode = hq::component_presets::oneN4148();
        envelopeDiode.name = "Compressor sidechain rectifier 1N4148-style";
        envelopeDiode_ = addDiodeParasiticSubcircuit(engine_, envDrive, envNode_, envelopeDiode);

        engine_.addCapacitor(envNode_, vref_,
                             capacitor(1.0e-6f, 25.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(envNode_, vref_, resistor(100000.0f));

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return false;

        lastSolve_ = {};
        lastLdrResistanceOhms_ = ldr_.resistanceOhms();

        if (!primeOperatingPoint()) return false;

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        engine_.setNonlinearResidualTolerance(2.0e-5f);
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        ldr_.reset();
        lastLdrResistanceOhms_ = ldr_.resistanceOhms();
        engine_.setResistance(ldrResistor_, lastLdrResistanceOhms_);
        lastSolve_ = {};
    }

    float processSample(float input) noexcept {
        engine_.setVoltageSource(inputSource_, input);
        lastSolve_ = engine_.processSample(40, 2.0e-5f);
        updateLdrResistance();
        const float out = engine_.voltage(outputNode_);
        if (lastSolve_.singular || !std::isfinite(out)) return 0.0f;
        return out;
    }

    StageVoltages stageVoltages() const noexcept {
        return {engine_.voltage(gainCell_) - engine_.voltage(vref_),
                engine_.voltage(envNode_) - engine_.voltage(vref_),
                engine_.voltage(outputNode_)};
    }

    float ldrResistanceOhms() const noexcept { return lastLdrResistanceOhms_; }
    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
    // Mirrors PowerAmpCircuit::updateOutputTransformerSaturation(): only push
    // an updated resistance through when it has moved by more than float
    // noise, since MnaCircuitEngine::setResistance() unconditionally marks
    // the static matrix cache dirty and forces a full rebuild on the next
    // solve.
    void updateLdrResistance() noexcept {
        const float envelopeVolts = engine_.voltage(envNode_) - engine_.voltage(vref_);
        const float normalizedDrive = std::clamp(envelopeVolts * sensitivityPerVolt, 0.0f, 1.0f);
        const float resistanceOhms = ldr_.processLedDrive(normalizedDrive);
        const float threshold = std::max(1.0f, std::abs(lastLdrResistanceOhms_) * 1.0e-4f);
        if (std::abs(resistanceOhms - lastLdrResistanceOhms_) > threshold) {
            engine_.setResistance(ldrResistor_, resistanceOhms);
            lastLdrResistanceOhms_ = resistanceOhms;
        }
    }

    bool primeOperatingPoint() noexcept {
        // Source stepping is a standard nonlinear-circuit continuation
        // technique: each solution becomes the initial guess for the next
        // slightly higher supply voltage. It runs only during prepare(),
        // never on the audio thread. Each step is a DC operating-point solve
        // (capacitors open, inductors shorted -- see
        // MnaCircuitEngine::solveDcOperatingPoint()), not a transient step,
        // so the homotopy converges to the circuit's true DC equilibrium --
        // including the sidechain envelope's ~100 ms release network -- in a
        // single pass rather than needing a multi-second silent warm-up to
        // wait that transient out.
        constexpr int sourceSteps = 128;
        constexpr int solvesPerStep = 2;
        for (int step = 1; step <= sourceSteps; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(sourceSteps);
            engine_.setVoltageSource(supplySource_, 9.0f * t);
            engine_.setVoltageSource(vrefSource_, 4.5f * t);
            engine_.setVoltageSource(inputSource_, 0.0f);
            for (int settle = 0; settle < solvesPerStep; ++settle) {
                lastSolve_ = engine_.solveDcOperatingPoint(40, 1.0e-6f);
                if (lastSolve_.singular || !finiteStages()) return false;
            }
        }
        // Must run before commitOperatingPointAsSteadyState(): it can call
        // setResistance() (if the LDR's drive-dependent resistance moved
        // from its construction-time default), which dirties the static
        // cache again. Doing that before the commit's own eager rebuild
        // folds both into the same rebuild, instead of leaving a second one
        // pending for the first post-prepare() transient sample.
        updateLdrResistance();
        engine_.commitOperatingPointAsSteadyState();
        return true;
    }

    bool finiteStages() const noexcept {
        const auto s = stageVoltages();
        return std::isfinite(s.gainCell) && std::isfinite(s.envelope) && std::isfinite(s.output);
    }

    static hq::ResistorSpec resistor(float ohms) noexcept {
        hq::ResistorSpec r{};
        r.resistanceOhms = std::max(1.0e-3f, ohms);
        r.tolerancePercent = 5.0f;
        r.powerRatingWatts = 0.25f;
        return r;
    }

    static hq::CapacitorSpec capacitor(float farads, float volts,
                                       hq::CapacitorTechnology technology) noexcept {
        hq::CapacitorSpec c{};
        c.capacitanceFarads = std::max(0.0f, farads);
        c.tolerancePercent = technology == hq::CapacitorTechnology::electrolytic ? 20.0f : 10.0f;
        c.voltageRatingVolts = std::max(1.0f, volts);
        c.esrOhms = technology == hq::CapacitorTechnology::electrolytic ? 0.5f : 0.03f;
        c.leakageResistanceOhms = technology == hq::CapacitorTechnology::electrolytic ? 5.0e6f : 1.0e9f;
        c.dielectricAbsorption = 0.0f;
        c.technology = technology;
        return c;
    }

    static hq::OpAmpSpec compressorOpAmp() noexcept {
        auto op = hq::component_presets::jrc4558();
        op.name = "Compressor JRC4558-style";
        return op;
    }

    MnaCircuitEngine engine_;
    double sampleRate_ = 48000.0;
    SourceHandle supplySource_{};
    SourceHandle vrefSource_{};
    SourceHandle inputSource_{};
    Node supply_ = ground;
    Node vref_ = ground;
    Node inputJack_ = ground;
    Node gainCell_ = ground;
    Node outputNode_ = ground;
    Node envNode_ = ground;
    ResistorHandle ldrResistor_{};
    DynamicOpAmpSubcircuit bufferOpAmp_{};
    DynamicOpAmpSubcircuit peakDetector_{};
    DiodeParasiticSubcircuit envelopeDiode_{};
    hq::OptocouplerLDR ldr_{};
    float lastLdrResistanceOhms_ = 5.0e6f;
    MnaCircuitEngine::SolveStats lastSolve_{};
};

} // namespace guitardsp::circuit
