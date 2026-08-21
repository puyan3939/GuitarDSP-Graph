#pragma once

#include "ActiveDeviceParasiticSubcircuits.h"
#include "DiodeParasiticSubcircuit.h"
#include "DynamicOpAmpSubcircuit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace guitardsp::circuit {

// Engaged-audio-path TS808 circuit assembled from MNA components.
//
// Topology follows the classic block order:
//   9 V / 4.5 V bias -> 2SC1815-style input emitter follower
//   -> JRC4558 clipping amplifier with 4k7/47n high-pass feedback,
//      51k + 500k drive resistance, anti-parallel silicon diodes and 51 pF
//   -> 1k/220n passive low-pass -> active 20k tone network
//   -> 100k level control -> 2SC1815-style output emitter follower.
//
// The JFET bypass/toggle logic and reverse-polarity supply protection are not in
// this engaged signal-path model yet. The 4.5 V node is represented as an ideal
// bias source: this avoids simulating the pedal's long power-on decoupling transient
// in every audio instance while preserving the AC operating point seen by the audio
// circuit. A later DC-operating-point/power-network pass can remove that shortcut.
class TS808Circuit {
public:
    static constexpr float defaultDrive = 0.45f;
    static constexpr float defaultTone = 0.50f;
    static constexpr float defaultLevel = 0.55f;

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        engine_ = MnaCircuitEngine{};

        const Node supply = engine_.addNode();
        const Node vref = engine_.addNode();
        const Node inputJack = engine_.addNode();
        const Node inputCoupled = engine_.addNode();
        const Node q1Base = engine_.addNode();
        const Node q1Emitter = engine_.addNode();
        const Node clipNonInv = engine_.addNode();
        const Node clipInv = engine_.addNode();
        const Node clipHpNode = engine_.addNode();
        const Node clipFeedbackNode = engine_.addNode();
        const Node clipOut = engine_.addNode();
        const Node toneNonInv = engine_.addNode();
        const Node toneInv = engine_.addNode();
        const Node toneWiper = engine_.addNode();
        const Node toneRcNode = engine_.addNode();
        const Node toneOut = engine_.addNode();
        const Node levelFeed = engine_.addNode();
        const Node levelTop = engine_.addNode();
        const Node levelWiper = engine_.addNode();
        const Node q3Base = engine_.addNode();
        const Node q3Emitter = engine_.addNode();
        const Node outputCouplingInput = engine_.addNode();
        outputNode_ = engine_.addNode();

        engine_.addVoltageSource(supply, ground, 9.0f);
        engine_.addVoltageSource(vref, ground, 4.5f);
        inputSource_ = engine_.addVoltageSource(inputJack, ground, 0.0f);

        // Input buffer: C1 22 nF, R1 1 k, R2 510 k, Q1 emitter follower,
        // R3 10 k and C2 1 uF.
        engine_.addCapacitor(inputJack, inputCoupled, capacitor(22.0e-9f, 50.0f,
                                                               hq::CapacitorTechnology::film));
        engine_.addResistor(inputCoupled, q1Base, resistor(1000.0f));
        engine_.addResistor(vref, q1Base, resistor(510000.0f));
        addBjtParasiticSubcircuit(engine_, supply, q1Base, q1Emitter, twoSC1815Style());
        engine_.addResistor(q1Emitter, ground, resistor(10000.0f));
        engine_.addCapacitor(q1Emitter, clipNonInv, capacitor(1.0e-6f, 50.0f,
                                                             hq::CapacitorTechnology::film));
        engine_.addResistor(clipNonInv, vref, resistor(10000.0f));

        // Clipping amplifier: the 4k7/47n branch is AC-ground referenced exactly
        // as in the classic schematic. The drive control is used as a rheostat.
        const auto opAmp = ts808OpAmp();
        clipOpAmp_ = addDynamicOpAmpSubcircuit(engine_, clipOut, clipNonInv, clipInv,
                                                supply, ground, ground, opAmp);
        engine_.addCapacitor(clipInv, clipHpNode, capacitor(47.0e-9f, 50.0f,
                                                           hq::CapacitorTechnology::film));
        engine_.addResistor(clipHpNode, ground, resistor(4700.0f));
        engine_.addResistor(clipInv, clipFeedbackNode, resistor(51000.0f));

        auto drive = potentiometer(500000.0f, hq::PotTaper::audio, 1.0f - defaultDrive);
        drivePot_ = engine_.addPotentiometer(clipFeedbackNode, clipOut, clipOut, drive);

        auto clipDiode = hq::component_presets::oneN4148();
        clipDiode.name = "TS808 1N914/1N4148-style";
        clippingDiodePositive_ = addDiodeParasiticSubcircuit(engine_, clipInv, clipOut, clipDiode);
        clippingDiodeNegative_ = addDiodeParasiticSubcircuit(engine_, clipOut, clipInv, clipDiode);
        engine_.addCapacitor(clipInv, clipOut, capacitor(51.0e-12f, 50.0f,
                                                        hq::CapacitorTechnology::ceramic));

        // Tone / volume section from the classic second half of the JRC4558.
        // R7/C5 create the ~723 Hz passive low-pass. The 20 k tone pot spans the
        // op-amp inputs, with its wiper returned to ground through 220 nF + 220 R.
        engine_.addResistor(clipOut, toneNonInv, resistor(1000.0f));
        engine_.addCapacitor(toneNonInv, ground, capacitor(220.0e-9f, 35.0f,
                                                           hq::CapacitorTechnology::film));
        engine_.addResistor(toneNonInv, vref, resistor(10000.0f));

        auto tone = potentiometer(20000.0f, hq::PotTaper::linear, 1.0f - defaultTone);
        tonePot_ = engine_.addPotentiometer(toneNonInv, toneWiper, toneInv, tone);
        engine_.addCapacitor(toneWiper, toneRcNode, capacitor(220.0e-9f, 35.0f,
                                                             hq::CapacitorTechnology::film));
        engine_.addResistor(toneRcNode, ground, resistor(220.0f));
        engine_.addResistor(toneOut, toneInv, resistor(1000.0f));

        // The second 4558 half is intentionally kept as the engine's finite-gain
        // op-amp primitive for now. The clipping half needs large-signal rail/slew
        // behavior; the tone half normally remains linear, and avoiding a second
        // artificial rail/slew macro materially improves Newton conditioning while
        // preserving the actual passive tone network around it.
        engine_.addOpAmp(toneOut, toneNonInv, toneInv, ground, opAmp);

        engine_.addCapacitor(toneOut, levelFeed, capacitor(1.0e-6f, 50.0f,
                                                           hq::CapacitorTechnology::film));
        engine_.addResistor(levelFeed, levelTop, resistor(1000.0f));
        auto level = potentiometer(100000.0f, hq::PotTaper::audio, defaultLevel);
        levelPot_ = engine_.addPotentiometer(levelTop, levelWiper, ground, level);

        // Output buffer: C8 100 nF, 510 k bias, 10 k emitter resistor, 100 R
        // output resistor, C9 10 uF and 10 k output load/pulldown.
        engine_.addCapacitor(levelWiper, q3Base, capacitor(100.0e-9f, 50.0f,
                                                           hq::CapacitorTechnology::film));
        engine_.addResistor(vref, q3Base, resistor(510000.0f));
        addBjtParasiticSubcircuit(engine_, supply, q3Base, q3Emitter, twoSC1815Style());
        engine_.addResistor(q3Emitter, ground, resistor(10000.0f));
        engine_.addResistor(q3Emitter, outputCouplingInput, resistor(100.0f));
        engine_.addCapacitor(outputCouplingInput, outputNode_, capacitor(10.0e-6f, 16.0f,
                                                                        hq::CapacitorTechnology::electrolytic));
        engine_.addResistor(outputNode_, ground, resistor(10000.0f));

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        if (!engine_.prepare(sampleRate_)) return false;

        drive_ = defaultDrive;
        tone_ = defaultTone;
        level_ = defaultLevel;
        lastSolve_ = {};

        // Control-thread warm-up only. It suppresses most coupling-cap startup
        // transient without putting any allocation or topology work on the audio path.
        const auto warmSamples = static_cast<std::size_t>(
            std::clamp(sampleRate_ * 0.08, 512.0, 8192.0));
        for (std::size_t i = 0; i < warmSamples; ++i) {
            engine_.setVoltageSource(inputSource_, 0.0f);
            lastSolve_ = engine_.processSample(40, 2.0e-5f);
            if (lastSolve_.singular || !std::isfinite(engine_.voltage(outputNode_))) return false;
        }
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        lastSolve_ = {};
    }

    bool setDrive(float normalized) noexcept {
        normalized = std::clamp(normalized, 0.0f, 1.0f);
        if (std::abs(normalized - drive_) < 1.0e-6f) return true;
        drive_ = normalized;
        // With high=feedback resistor and wiper/low tied to the op-amp output,
        // electrical resistance increases as the mechanical position is reversed.
        return engine_.setPotentiometerPosition(drivePot_, 1.0f - normalized);
    }

    bool setTone(float normalized) noexcept {
        normalized = std::clamp(normalized, 0.0f, 1.0f);
        if (std::abs(normalized - tone_) < 1.0e-6f) return true;
        tone_ = normalized;
        // Parameter convention: 0=bass, 1=treble. The physical wiper reaches
        // the inverting-input end at the treble side, hence the reversal.
        return engine_.setPotentiometerPosition(tonePot_, 1.0f - normalized);
    }

    bool setLevel(float normalized) noexcept {
        normalized = std::clamp(normalized, 0.0f, 1.0f);
        if (std::abs(normalized - level_) < 1.0e-6f) return true;
        level_ = normalized;
        return engine_.setPotentiometerPosition(levelPot_, normalized);
    }

    bool setControls(float drive, float tone, float level) noexcept {
        bool ok = true;
        ok &= setDrive(drive);
        ok &= setTone(tone);
        ok &= setLevel(level);
        return ok;
    }

    float processSample(float input) noexcept {
        engine_.setVoltageSource(inputSource_, input);
        lastSolve_ = engine_.processSample(40, 2.0e-5f);
        const float out = engine_.voltage(outputNode_);
        if (lastSolve_.singular || !std::isfinite(out)) return 0.0f;
        return out;
    }

    float drive() const noexcept { return drive_; }
    float tone() const noexcept { return tone_; }
    float level() const noexcept { return level_; }
    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
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

    static hq::PotentiometerSpec potentiometer(float ohms, hq::PotTaper taper,
                                                float position) noexcept {
        hq::PotentiometerSpec p{};
        p.totalResistanceOhms = ohms;
        p.tolerancePercent = 20.0f;
        p.powerRatingWatts = 0.25f;
        p.taper = taper;
        p.position = std::clamp(position, 0.0f, 1.0f);
        return p;
    }

    static hq::BJTSpec twoSC1815Style() noexcept {
        auto q = hq::component_presets::twoN3904();
        q.name = "2SC1815-style";
        q.beta = 350.0f;
        q.nominalVbe = 0.62f;
        q.saturationVoltage = 0.15f;
        q.maxCollectorVoltage = 50.0f;
        q.maxCollectorCurrentAmps = 0.15f;
        q.inputCapacitanceFarads = 8.0e-12f;
        return q;
    }

    static hq::OpAmpSpec ts808OpAmp() noexcept {
        auto op = hq::component_presets::jrc4558();
        op.name = "TS808 JRC4558-style";
        return op;
    }

    MnaCircuitEngine engine_;
    double sampleRate_ = 48000.0;
    SourceHandle inputSource_{};
    Node outputNode_ = ground;
    PotHandle drivePot_{};
    PotHandle tonePot_{};
    PotHandle levelPot_{};
    DynamicOpAmpSubcircuit clipOpAmp_{};
    DiodeParasiticSubcircuit clippingDiodePositive_{};
    DiodeParasiticSubcircuit clippingDiodeNegative_{};
    MnaCircuitEngine::SolveStats lastSolve_{};
    float drive_ = defaultDrive;
    float tone_ = defaultTone;
    float level_ = defaultLevel;
};

} // namespace guitardsp::circuit
