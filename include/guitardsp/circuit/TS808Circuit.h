#pragma once

#include "BjtEbersMollSubcircuit.h"
#include "DiodeParasiticSubcircuit.h"
#include "DynamicOpAmpSubcircuit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace guitardsp::circuit {

// Component-level engaged TS808 audio path.
//
// Implemented as ordinary MNA parts:
// 9 V / 4.5 V bias, 2SC1815-style input emitter follower, JRC4558 clipping
// amplifier, 4k7/47n feedback high-pass, 51k + 500k drive resistance,
// anti-parallel silicon diodes + junction capacitance, 51 pF compensation,
// passive/active 20k tone network, 100k level pot, and a 2SC1815-style output
// emitter follower. Bypass JFET switching and supply protection are intentionally
// outside this engaged-signal-path model for now.
class TS808Circuit {
public:
    static constexpr float defaultDrive = 0.45f;
    static constexpr float defaultTone = 0.50f;
    static constexpr float defaultLevel = 0.55f;

    struct StageVoltages {
        float inputEmitter = 0.0f;
        float clippingInput = 0.0f;
        float clippingOutput = 0.0f;
        float toneInput = 0.0f;
        float toneOutput = 0.0f;
        float levelWiper = 0.0f;
        float outputEmitter = 0.0f;
        float output = 0.0f;
    };

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        engine_ = MnaCircuitEngine{};

        supply_ = engine_.addNode();
        vref_ = engine_.addNode();
        inputJack_ = engine_.addNode();
        const Node inputCoupled = engine_.addNode();
        const Node q1Base = engine_.addNode();
        q1Emitter_ = engine_.addNode();
        clipNonInv_ = engine_.addNode();
        const Node clipInv = engine_.addNode();
        const Node clipHpNode = engine_.addNode();
        const Node clipFeedbackNode = engine_.addNode();
        clipOut_ = engine_.addNode();
        toneNonInv_ = engine_.addNode();
        const Node toneInv = engine_.addNode();
        const Node toneWiper = engine_.addNode();
        const Node toneRcNode = engine_.addNode();
        toneOut_ = engine_.addNode();
        const Node levelFeed = engine_.addNode();
        const Node levelTop = engine_.addNode();
        levelWiper_ = engine_.addNode();
        const Node q3Base = engine_.addNode();
        q3Emitter_ = engine_.addNode();
        const Node outputCouplingInput = engine_.addNode();
        outputNode_ = engine_.addNode();

        // Supply sources are initially zero. After the topology is prepared we use
        // control-thread source stepping to establish a nearby nonlinear operating
        // point instead of asking Newton to jump from an all-zero guess directly to
        // a 9 V / 4.5 V semiconductor circuit in one sample.
        supplySource_ = engine_.addVoltageSource(supply_, ground, 0.0f);
        vrefSource_ = engine_.addVoltageSource(vref_, ground, 0.0f);
        inputSource_ = engine_.addVoltageSource(inputJack_, ground, 0.0f);

        // Input emitter follower. The two-junction Ebers-Moll macro preserves B-E
        // and B-C conduction explicitly, so this same device model can later be used
        // in transistor gain stages that enter saturation rather than only buffers.
        engine_.addCapacitor(inputJack_, inputCoupled,
                             capacitor(22.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(inputCoupled, q1Base, resistor(1000.0f));
        engine_.addResistor(vref_, q1Base, resistor(510000.0f));
        inputBuffer_ = addBjtEbersMollSubcircuit(
            engine_, supply_, q1Base, q1Emitter_, twoSC1815Style());
        engine_.addResistor(q1Emitter_, ground, resistor(10000.0f));
        engine_.addCapacitor(q1Emitter_, clipNonInv_,
                             capacitor(1.0e-6f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(clipNonInv_, vref_, resistor(10000.0f));

        // First JRC4558 half: clipping amplifier.
        const auto opAmp = ts808OpAmp();
        clipOpAmp_ = addDynamicOpAmpSubcircuit(
            engine_, clipOut_, clipNonInv_, clipInv, supply_, ground, ground, opAmp);
        engine_.addCapacitor(clipInv, clipHpNode,
                             capacitor(47.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(clipHpNode, ground, resistor(4700.0f));
        engine_.addResistor(clipInv, clipFeedbackNode, resistor(51000.0f));
        drivePot_ = engine_.addPotentiometer(
            clipFeedbackNode, clipOut_, clipOut_,
            potentiometer(500000.0f, hq::PotTaper::audio, 1.0f - defaultDrive));

        auto clipDiode = hq::component_presets::oneN4148();
        clipDiode.name = "TS808 1N914/1N4148-style";
        clippingDiodePositive_ = addDiodeParasiticSubcircuit(
            engine_, clipInv, clipOut_, clipDiode);
        clippingDiodeNegative_ = addDiodeParasiticSubcircuit(
            engine_, clipOut_, clipInv, clipDiode);
        engine_.addCapacitor(clipInv, clipOut_,
                             capacitor(51.0e-12f, 50.0f, hq::CapacitorTechnology::ceramic));

        // Second half: R7/C5 passive rolloff plus the 20k active tone network.
        engine_.addResistor(clipOut_, toneNonInv_, resistor(1000.0f));
        engine_.addCapacitor(toneNonInv_, ground,
                             capacitor(220.0e-9f, 35.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(toneNonInv_, vref_, resistor(10000.0f));
        tonePot_ = engine_.addPotentiometer(
            toneNonInv_, toneWiper, toneInv,
            potentiometer(20000.0f, hq::PotTaper::linear, 1.0f - defaultTone));
        engine_.addCapacitor(toneWiper, toneRcNode,
                             capacitor(220.0e-9f, 35.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(toneRcNode, ground, resistor(220.0f));
        engine_.addResistor(toneOut_, toneInv, resistor(1000.0f));

        // The tone half normally stays linear, so the finite-gain MNA op-amp keeps
        // the exact surrounding R/C/pot topology without adding a second stiff
        // large-signal macro to the same Newton system.
        engine_.addOpAmp(toneOut_, toneNonInv_, toneInv, ground, opAmp);

        engine_.addCapacitor(toneOut_, levelFeed,
                             capacitor(1.0e-6f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(levelFeed, levelTop, resistor(1000.0f));
        levelPot_ = engine_.addPotentiometer(
            levelTop, levelWiper_, ground,
            potentiometer(100000.0f, hq::PotTaper::audio, defaultLevel));

        // Output emitter follower.
        engine_.addCapacitor(levelWiper_, q3Base,
                             capacitor(100.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(vref_, q3Base, resistor(510000.0f));
        outputBuffer_ = addBjtEbersMollSubcircuit(
            engine_, supply_, q3Base, q3Emitter_, twoSC1815Style());
        engine_.addResistor(q3Emitter_, ground, resistor(10000.0f));
        engine_.addResistor(q3Emitter_, outputCouplingInput, resistor(100.0f));
        engine_.addCapacitor(outputCouplingInput, outputNode_,
                             capacitor(10.0e-6f, 16.0f, hq::CapacitorTechnology::electrolytic));
        engine_.addResistor(outputNode_, ground, resistor(10000.0f));

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return false;

        targetDrive_ = appliedDrive_ = defaultDrive;
        targetTone_ = appliedTone_ = defaultTone;
        targetLevel_ = appliedLevel_ = defaultLevel;
        controlUpdateCountdown_ = 0;
        lastSolve_ = {};

        if (!primeOperatingPoint()) return false;

        // Once the DC neighborhood is established, return to the automatic solver
        // so normal audio processing can use the prepared sparse nonlinear path.
        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        // The audio-rate Newton voltage criterion is 20 ppm. Requiring the
        // separately row-scaled KCL residual to reach the engine's generic
        // 0.3 ppm default forced a second factorization after the component
        // equations were already within the circuit's accepted precision. Match
        // both criteria exactly, as the DS-1 path does, while retaining the full
        // 48-unknown circuit and its unchanged nonlinear stamps.
        engine_.setNonlinearResidualTolerance(2.0e-5f);
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        targetDrive_ = appliedDrive_ = defaultDrive;
        targetTone_ = appliedTone_ = defaultTone;
        targetLevel_ = appliedLevel_ = defaultLevel;
        controlUpdateCountdown_ = 0;
        engine_.setPotentiometerPosition(drivePot_, 1.0f - appliedDrive_);
        engine_.setPotentiometerPosition(tonePot_, 1.0f - appliedTone_);
        engine_.setPotentiometerPosition(levelPot_, appliedLevel_);
        lastSolve_ = {};
    }

    bool setDrive(float normalized) noexcept {
        targetDrive_ = std::clamp(normalized, 0.0f, 1.0f);
        return true;
    }

    bool setTone(float normalized) noexcept {
        targetTone_ = std::clamp(normalized, 0.0f, 1.0f);
        return true;
    }

    bool setLevel(float normalized) noexcept {
        targetLevel_ = std::clamp(normalized, 0.0f, 1.0f);
        return true;
    }

    bool setControls(float drive, float tone, float level) noexcept {
        setDrive(drive);
        setTone(tone);
        setLevel(level);
        return true;
    }

    float processSample(float input) noexcept {
        applySmoothedControls();
        engine_.setVoltageSource(inputSource_, input);
        lastSolve_ = engine_.processSample(40, 2.0e-5f);
        const float out = engine_.voltage(outputNode_);
        if (lastSolve_.singular || !std::isfinite(out)) return 0.0f;
        return out;
    }

    StageVoltages stageVoltages() const noexcept {
        return {engine_.voltage(q1Emitter_), engine_.voltage(clipNonInv_),
                engine_.voltage(clipOut_), engine_.voltage(toneNonInv_),
                engine_.voltage(toneOut_), engine_.voltage(levelWiper_),
                engine_.voltage(q3Emitter_), engine_.voltage(outputNode_)};
    }

    float drive() const noexcept { return targetDrive_; }
    float tone() const noexcept { return targetTone_; }
    float level() const noexcept { return targetLevel_; }
    float appliedDrive() const noexcept { return appliedDrive_; }
    float appliedTone() const noexcept { return appliedTone_; }
    float appliedLevel() const noexcept { return appliedLevel_; }
    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
    static float approach(float current, float target, float maximumStep) noexcept {
        return current + std::clamp(target - current, -maximumStep, maximumStep);
    }

    void applySmoothedControls() noexcept {
        if (appliedDrive_ == targetDrive_ && appliedTone_ == targetTone_
            && appliedLevel_ == targetLevel_) {
            controlUpdateCountdown_ = 0;
            return;
        }
        if (controlUpdateCountdown_ > 0) {
            --controlUpdateCountdown_;
            return;
        }

        // Potentiometers are control signals, not oversampled audio sources.
        // Update at no less than 24 kHz while retaining the original 5 ms ramp;
        // the complete circuit still runs at its full 2x/4x/8x/16x audio rate.
        // This prevents a GUI gesture from unnecessarily refactorizing the
        // identical physical matrix once per oversampled audio sample.
        const int updateInterval = std::max(1,
            static_cast<int>(sampleRate_ / 24000.0));
        const float maximumStep = static_cast<float>(updateInterval) /
            static_cast<float>(std::max(1.0, sampleRate_ * 0.005));
        controlUpdateCountdown_ = updateInterval - 1;

        const float nextDrive = approach(appliedDrive_, targetDrive_, maximumStep);
        const float nextTone = approach(appliedTone_, targetTone_, maximumStep);
        const float nextLevel = approach(appliedLevel_, targetLevel_, maximumStep);

        if (nextDrive != appliedDrive_) {
            appliedDrive_ = nextDrive;
            engine_.setPotentiometerPosition(drivePot_, 1.0f - appliedDrive_);
        }
        if (nextTone != appliedTone_) {
            appliedTone_ = nextTone;
            engine_.setPotentiometerPosition(tonePot_, 1.0f - appliedTone_);
        }
        if (nextLevel != appliedLevel_) {
            appliedLevel_ = nextLevel;
            engine_.setPotentiometerPosition(levelPot_, appliedLevel_);
        }
    }

    bool primeOperatingPoint() noexcept {
        // Source stepping is a standard nonlinear-circuit continuation technique:
        // each solution becomes the initial guess for the next slightly higher
        // supply voltage. It runs only during prepare(), never on the audio
        // thread. Each step is a DC operating-point solve (capacitors open,
        // inductors shorted -- see MnaCircuitEngine::solveDcOperatingPoint()),
        // not a transient step, so the homotopy converges to the circuit's
        // true DC equilibrium rather than to a point still partway through a
        // capacitor's charge transient.
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
        engine_.commitOperatingPointAsSteadyState();
        return true;
    }

    bool finiteStages() const noexcept {
        const auto s = stageVoltages();
        return std::isfinite(s.inputEmitter) && std::isfinite(s.clippingInput) &&
               std::isfinite(s.clippingOutput) && std::isfinite(s.toneInput) &&
               std::isfinite(s.toneOutput) && std::isfinite(s.levelWiper) &&
               std::isfinite(s.outputEmitter) && std::isfinite(s.output);
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
    SourceHandle supplySource_{};
    SourceHandle vrefSource_{};
    SourceHandle inputSource_{};
    Node supply_ = ground;
    Node vref_ = ground;
    Node inputJack_ = ground;
    Node q1Emitter_ = ground;
    Node clipNonInv_ = ground;
    Node clipOut_ = ground;
    Node toneNonInv_ = ground;
    Node toneOut_ = ground;
    Node levelWiper_ = ground;
    Node q3Emitter_ = ground;
    Node outputNode_ = ground;
    PotHandle drivePot_{};
    PotHandle tonePot_{};
    PotHandle levelPot_{};
    BjtEbersMollSubcircuit inputBuffer_{};
    BjtEbersMollSubcircuit outputBuffer_{};
    DynamicOpAmpSubcircuit clipOpAmp_{};
    DiodeParasiticSubcircuit clippingDiodePositive_{};
    DiodeParasiticSubcircuit clippingDiodeNegative_{};
    MnaCircuitEngine::SolveStats lastSolve_{};
    float targetDrive_ = defaultDrive;
    float targetTone_ = defaultTone;
    float targetLevel_ = defaultLevel;
    float appliedDrive_ = defaultDrive;
    float appliedTone_ = defaultTone;
    float appliedLevel_ = defaultLevel;
    int controlUpdateCountdown_ = 0;
};

} // namespace guitardsp::circuit
