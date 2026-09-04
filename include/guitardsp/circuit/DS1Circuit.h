#pragma once

#include "BjtEbersMollSubcircuit.h"
#include "DiodeParasiticSubcircuit.h"
#include "DynamicOpAmpSubcircuit.h"
#include "OperatingPointContinuation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace guitardsp::circuit {

// Component-level engaged signal path for the post-1994 / dual-op-amp DS-1 family.
//
// The topology follows the published modern DS-1 signal blocks and component
// values rather than collapsing the pedal to a few fitted filters/waveshapers:
//
//   47 nF input coupling + 2SC2240-style emitter follower
//   -> 470 nF / 100 k and 47 nF / 100 k bass-shaping high passes
//   -> 2SC2240-style common-emitter booster with 10 k / 22 ohm gain network,
//      470 k collector-to-base shunt feedback and 250 pF HF feedback
//   -> 68 nF / 100 k op-amp input coupling and unity buffer
//   -> variable non-inverting gain (100 k DIST, 4.7 k / 470 nF feedback leg)
//   -> 2.2 k / 470 nF coupling, antiparallel silicon hard clipping and 10 nF HF cap
//   -> 6.8 k / 100 nF low-pass + 22 nF / 2.2 k / 6.8 k high-pass tone blend
//   -> 100 k LEVEL -> 2SC2240-style output emitter follower -> 1 uF output coupling.
//
// The original Boss JFET switching/flip-flop network and reverse-polarity supply
// protection are intentionally outside this engaged-audio-path model. The active
// device presets are engineering models, not manufacturer SPICE models. This class
// is therefore a component-level circuit implementation of the modern topology,
// not yet a measured hardware-equivalent calibration.
class DS1Circuit {
public:
    static constexpr float defaultDistortion = 0.55f;
    static constexpr float defaultTone = 0.50f;
    static constexpr float defaultLevel = 0.55f;

    struct StageVoltages {
        float inputEmitter = 0.0f;
        float boosterBase = 0.0f;
        float boosterCollector = 0.0f;
        float opAmpBuffer = 0.0f;
        float gainOutput = 0.0f;
        float clippingNode = 0.0f;
        float toneWiper = 0.0f;
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
        const Node inputSeries = engine_.addNode();
        const Node q1Base = engine_.addNode();
        q1Emitter_ = engine_.addNode();
        const Node firstHighPass = engine_.addNode();
        q2Base_ = engine_.addNode();
        const Node q2Emitter = engine_.addNode();
        q2Collector_ = engine_.addNode();
        const Node opAmpInput = engine_.addNode();
        const Node opAmpBufferInv = engine_.addNode();
        opAmpBufferOut_ = engine_.addNode();
        const Node gainInv = engine_.addNode();
        const Node gainFeedbackGround = engine_.addNode();
        gainOut_ = engine_.addNode();
        const Node clipCouplingInput = engine_.addNode();
        clipNode_ = engine_.addNode();
        const Node toneHighPre = engine_.addNode();
        const Node toneHigh = engine_.addNode();
        const Node toneLow = engine_.addNode();
        toneWiper_ = engine_.addNode();
        levelWiper_ = engine_.addNode();
        const Node levelPost = engine_.addNode();
        const Node q3Base = engine_.addNode();
        q3Emitter_ = engine_.addNode();
        const Node outputCouplingInput = engine_.addNode();
        outputNode_ = engine_.addNode();

        // Control-thread source stepping below establishes the semiconductor DC
        // operating point. Starting these sources at zero avoids a hostile 0 -> 9 V
        // Newton jump when prepare() creates the initial all-zero solution vector.
        supplySource_ = engine_.addVoltageSource(supply_, ground, 0.0f);
        vrefSource_ = engine_.addVoltageSource(vref_, ground, 0.0f);
        inputSource_ = engine_.addVoltageSource(inputJack_, ground, 0.0f);

        // Input buffer: R1/C1/R2/Q1/R3.
        engine_.addResistor(inputJack_, inputSeries, resistor(1000.0f));
        engine_.addCapacitor(inputSeries, q1Base,
                             capacitor(47.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(vref_, q1Base, resistor(470000.0f));
        inputBuffer_ = addBjtEbersMollSubcircuit(
            engine_, supply_, q1Base, q1Emitter_, twoSC2240Style());
        engine_.addResistor(q1Emitter_, ground, resistor(10000.0f));

        // The modern engaged path contains two bass-shaping couplings before Q2.
        // The JFET effect switch normally lies between them; in an always-engaged
        // circuit model it is replaced by a direct connection while retaining the
        // published RC time constants.
        engine_.addCapacitor(q1Emitter_, firstHighPass,
                             capacitor(470.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(firstHighPass, ground, resistor(100000.0f));
        engine_.addCapacitor(firstHighPass, q2Base_,
                             capacitor(47.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(q2Base_, ground, resistor(100000.0f));

        // Q2 booster. The very small 22 ohm emitter resistor creates high raw gain;
        // the 470 k collector/base shunt feedback sets the DC neighborhood and
        // reduces closed-loop gain, while C4 rolls off the highest frequencies.
        engine_.addResistor(supply_, q2Collector_, resistor(10000.0f));
        engine_.addResistor(q2Emitter, ground, resistor(22.0f));
        engine_.addResistor(q2Collector_, q2Base_, resistor(470000.0f));
        engine_.addCapacitor(q2Collector_, q2Base_,
                             capacitor(250.0e-12f, 50.0f, hq::CapacitorTechnology::ceramic));
        booster_ = addBjtEbersMollSubcircuit(
            engine_, q2Collector_, q2Base_, q2Emitter, twoSC2240Style());

        // Post-'94 dual op amp. The first half is used as a unity buffer. Keeping it
        // as the finite-gain MNA op-amp is numerically cheaper than adding another
        // large-signal macro to the same Newton system, and this half normally stays
        // away from clipping in the stock circuit.
        engine_.addCapacitor(q2Collector_, opAmpInput,
                             capacitor(68.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(opAmpInput, vref_, resistor(100000.0f));
        auto protectionDiode = hq::component_presets::oneN4148();
        protectionDiode.name = "DS-1 op-amp input protection";
        inputProtection_ = addDiodeParasiticSubcircuit(
            engine_, ground, opAmpInput, protectionDiode);

        const auto opAmp = ds1OpAmp();
        engine_.addOpAmp(opAmpBufferOut_, opAmpInput, opAmpBufferInv, ground, opAmp);
        engine_.addResistor(opAmpBufferOut_, opAmpBufferInv, resistor(100000.0f));

        // Second half: variable non-inverting distortion gain. At audio frequencies
        // the feedback leg is approximately 4.7 k to AC ground, so VR1 spans roughly
        // unity to 22.3x gain. C8 creates the ~72 Hz lower corner and C7 rolls off
        // only very high feedback-loop content.
        gainOpAmp_ = addDynamicOpAmpSubcircuit(
            engine_, gainOut_, opAmpBufferOut_, gainInv, supply_, ground, ground, opAmp);
        drivePot_ = engine_.addPotentiometer(
            gainInv, gainOut_, gainOut_,
            potentiometer(100000.0f, hq::PotTaper::linear, 1.0f - defaultDistortion));
        engine_.addCapacitor(gainInv, gainOut_,
                             capacitor(100.0e-12f, 50.0f, hq::CapacitorTechnology::ceramic));
        engine_.addResistor(gainInv, gainFeedbackGround, resistor(4700.0f));
        engine_.addCapacitor(gainFeedbackGround, ground,
                             capacitor(470.0e-9f, 50.0f, hq::CapacitorTechnology::film));

        // Hard-clipping network. C9 removes the 4.5 V op-amp bias before the shunt
        // diodes; D4/D5 therefore clip symmetrically around true ground. R14/C10 set
        // the familiar ~7.2 kHz small-signal upper corner.
        engine_.addResistor(gainOut_, clipCouplingInput, resistor(2200.0f));
        engine_.addCapacitor(clipCouplingInput, clipNode_,
                             capacitor(470.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        auto clippingDiode = hq::component_presets::oneN4148();
        clippingDiode.name = "DS-1 1S1588/1S2473/1N4148-style";
        clippingPositive_ = addDiodeParasiticSubcircuit(
            engine_, clipNode_, ground, clippingDiode);
        clippingNegative_ = addDiodeParasiticSubcircuit(
            engine_, ground, clipNode_, clippingDiode);
        engine_.addCapacitor(clipNode_, ground,
                             capacitor(10.0e-9f, 50.0f, hq::CapacitorTechnology::film));

        // Passive Big-Muff-family tone blend. The low branch is 6.8 k / 100 nF;
        // the high branch is 22 nF followed by 2.2 k with a 6.8 k shunt. VR3 blends
        // the two rather than implementing a generic one-pole crossfade.
        engine_.addCapacitor(clipNode_, toneHighPre,
                             capacitor(22.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(toneHighPre, toneHigh, resistor(2200.0f));
        engine_.addResistor(toneHigh, ground, resistor(6800.0f));
        engine_.addResistor(clipNode_, toneLow, resistor(6800.0f));
        engine_.addCapacitor(toneLow, ground,
                             capacitor(100.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        tonePot_ = engine_.addPotentiometer(
            toneHigh, toneWiper_, toneLow,
            potentiometer(20000.0f, hq::PotTaper::linear, defaultTone));

        levelPot_ = engine_.addPotentiometer(
            toneWiper_, levelWiper_, ground,
            potentiometer(100000.0f, hq::PotTaper::linear, defaultLevel));

        // Output side of the engaged path. The stock JFET switch is omitted, but the
        // 10 k feed, 47 nF coupling, high-value bias/pulldown resistors and Q3 emitter
        // follower are retained so the buffer loading remains part of the sound.
        engine_.addResistor(levelWiper_, levelPost, resistor(10000.0f));
        engine_.addResistor(levelPost, ground, resistor(1000000.0f));
        engine_.addCapacitor(levelPost, q3Base,
                             capacitor(47.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(vref_, q3Base, resistor(1000000.0f));
        outputBuffer_ = addBjtEbersMollSubcircuit(
            engine_, supply_, q3Base, q3Emitter_, twoSC2240Style());
        engine_.addResistor(q3Emitter_, ground, resistor(10000.0f));
        engine_.addResistor(q3Emitter_, outputCouplingInput, resistor(1000.0f));
        engine_.addCapacitor(outputCouplingInput, outputNode_,
                             capacitor(1.0e-6f, 50.0f, hq::CapacitorTechnology::electrolytic));
        engine_.addResistor(outputNode_, ground, resistor(100000.0f));

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return false;

        targetDistortion_ = appliedDistortion_ = defaultDistortion;
        targetTone_ = appliedTone_ = defaultTone;
        targetLevel_ = appliedLevel_ = defaultLevel;
        controlUpdateCountdown_ = 0;
        lastSolve_ = {};

        // Analytic DC operating-point solve (capacitors open, inductors
        // shorted, source-stepped Newton homotopy) replaces the previous
        // fixed-length silent transient warm-up: every coupling/bypass
        // capacitor's state is initialized at its true equilibrium regardless
        // of its RC time constant, instead of hoping a fixed sample budget
        // happened to be long enough.
        DcOperatingPointOptions dcOptions{};
        dcOptions.sourceSteps = 128;
        dcOptions.solvesPerStep = 2;
        const OperatingPointSourceTarget dcTargets[]{{supplySource_, 9.0f},
                                                       {vrefSource_, 4.5f}};
        const auto dcResult = establishDcOperatingPoint(engine_, dcTargets, dcOptions);
        lastSolve_ = dcResult.lastSolve;
        if (!dcResult.converged || !finiteStages()) return false;

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        // The 57-unknown DS-1 combines three transistor macros with a high-gain
        // op-amp. Its Newton voltage tolerance is already 20 ppm; demanding a
        // second 0.3 ppm row-scaled residual below the circuit's float-resolution
        // floor caused endless 40-step limit cycles even at silence. Match both
        // convergence criteria while retaining every component and exact stamp.
        engine_.setNonlinearResidualTolerance(2.0e-5f);
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        targetDistortion_ = appliedDistortion_ = defaultDistortion;
        targetTone_ = appliedTone_ = defaultTone;
        targetLevel_ = appliedLevel_ = defaultLevel;
        controlUpdateCountdown_ = 0;
        engine_.setPotentiometerPosition(drivePot_, 1.0f - appliedDistortion_);
        engine_.setPotentiometerPosition(tonePot_, appliedTone_);
        engine_.setPotentiometerPosition(levelPot_, appliedLevel_);
        lastSolve_ = {};
    }

    bool setDistortion(float normalized) noexcept {
        targetDistortion_ = std::clamp(normalized, 0.0f, 1.0f);
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

    bool setControls(float distortion, float tone, float level) noexcept {
        setDistortion(distortion);
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
        return {engine_.voltage(q1Emitter_), engine_.voltage(q2Base_),
                engine_.voltage(q2Collector_), engine_.voltage(opAmpBufferOut_),
                engine_.voltage(gainOut_), engine_.voltage(clipNode_),
                engine_.voltage(toneWiper_), engine_.voltage(levelWiper_),
                engine_.voltage(q3Emitter_), engine_.voltage(outputNode_)};
    }

    float distortion() const noexcept { return targetDistortion_; }
    float tone() const noexcept { return targetTone_; }
    float level() const noexcept { return targetLevel_; }
    float appliedDistortion() const noexcept { return appliedDistortion_; }
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
        if (appliedDistortion_ == targetDistortion_ && appliedTone_ == targetTone_
            && appliedLevel_ == targetLevel_) {
            controlUpdateCountdown_ = 0;
            return;
        }
        if (controlUpdateCountdown_ > 0) {
            --controlUpdateCountdown_;
            return;
        }

        // Preserve the five-millisecond physical pot ramp, but run the control
        // trajectory at >=24 kHz instead of the much higher circuit audio rate.
        // All three edited pots still rebuild one exact MNA matrix together.
        const int updateInterval = std::max(1,
            static_cast<int>(sampleRate_ / 24000.0));
        const float maximumStep = static_cast<float>(updateInterval) /
            static_cast<float>(std::max(1.0, sampleRate_ * 0.005));
        controlUpdateCountdown_ = updateInterval - 1;

        const float nextDistortion = approach(appliedDistortion_, targetDistortion_, maximumStep);
        const float nextTone = approach(appliedTone_, targetTone_, maximumStep);
        const float nextLevel = approach(appliedLevel_, targetLevel_, maximumStep);

        if (nextDistortion != appliedDistortion_) {
            appliedDistortion_ = nextDistortion;
            engine_.setPotentiometerPosition(drivePot_, 1.0f - appliedDistortion_);
        }
        if (nextTone != appliedTone_) {
            appliedTone_ = nextTone;
            engine_.setPotentiometerPosition(tonePot_, appliedTone_);
        }
        if (nextLevel != appliedLevel_) {
            appliedLevel_ = nextLevel;
            engine_.setPotentiometerPosition(levelPot_, appliedLevel_);
        }
    }

    bool finiteStages() const noexcept {
        const auto s = stageVoltages();
        return std::isfinite(s.inputEmitter) && std::isfinite(s.boosterBase) &&
               std::isfinite(s.boosterCollector) && std::isfinite(s.opAmpBuffer) &&
               std::isfinite(s.gainOutput) && std::isfinite(s.clippingNode) &&
               std::isfinite(s.toneWiper) && std::isfinite(s.levelWiper) &&
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

    static hq::BJTSpec twoSC2240Style() noexcept {
        auto q = hq::component_presets::twoN5088();
        q.name = "2SC2240-style";
        q.beta = 350.0f;
        q.nominalVbe = 0.62f;
        q.saturationVoltage = 0.15f;
        q.maxCollectorVoltage = 120.0f;
        q.maxCollectorCurrentAmps = 0.10f;
        q.inputCapacitanceFarads = 12.0e-12f;
        return q;
    }

    static hq::OpAmpSpec ds1OpAmp() noexcept {
        hq::OpAmpSpec op{};
        op.name = "DS-1 NJM3404/NJM2904-style";
        op.openLoopGainDb = 100.0f;
        op.gainBandwidthHz = 1.2e6f;
        op.slewRateVoltsPerSecond = 0.5e6f;
        op.inputBiasCurrentAmps = 40.0e-9f;
        op.inputOffsetVoltage = 2.0e-3f;
        op.inputNoiseVoltsPerRootHz = 40.0e-9f;
        op.outputCurrentLimitAmps = 0.020f;
        op.positiveRailHeadroomVolts = 1.5f;
        op.negativeRailHeadroomVolts = 0.05f;
        op.outputResistanceOhms = 50.0f;
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
    Node q2Base_ = ground;
    Node q2Collector_ = ground;
    Node opAmpBufferOut_ = ground;
    Node gainOut_ = ground;
    Node clipNode_ = ground;
    Node toneWiper_ = ground;
    Node levelWiper_ = ground;
    Node q3Emitter_ = ground;
    Node outputNode_ = ground;
    PotHandle drivePot_{};
    PotHandle tonePot_{};
    PotHandle levelPot_{};
    BjtEbersMollSubcircuit inputBuffer_{};
    BjtEbersMollSubcircuit booster_{};
    BjtEbersMollSubcircuit outputBuffer_{};
    DynamicOpAmpSubcircuit gainOpAmp_{};
    DiodeParasiticSubcircuit inputProtection_{};
    DiodeParasiticSubcircuit clippingPositive_{};
    DiodeParasiticSubcircuit clippingNegative_{};
    MnaCircuitEngine::SolveStats lastSolve_{};
    float targetDistortion_ = defaultDistortion;
    float targetTone_ = defaultTone;
    float targetLevel_ = defaultLevel;
    float appliedDistortion_ = defaultDistortion;
    float appliedTone_ = defaultTone;
    float appliedLevel_ = defaultLevel;
    int controlUpdateCountdown_ = 0;
};

} // namespace guitardsp::circuit
