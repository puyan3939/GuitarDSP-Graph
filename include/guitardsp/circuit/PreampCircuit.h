#pragma once

#include "OperatingPointContinuation.h"
#include "TriodeParasiticSubcircuit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace guitardsp::circuit {

// Component-level single-stage 12AX7 common-cathode preamp, plus a simplified
// passive Bass/Treble tone control.
//
// Signal path, built from ordinary MNA parts exactly like TS808Circuit/
// DS1Circuit:
//
//   input coupling cap -> grid stopper -> 1M grid leak (self-bias reference)
//   -> 12AX7 common-cathode gain stage (100k plate resistor off a 300 V B+
//      rail, 1.5k cathode resistor bypassed by a 25 uF cap for near-maximum
//      voltage gain)
//   -> plate coupling cap into a passive two-band tone network (a low-pass
//      RC feeding a Bass pot, a high-pass RC feeding a Treble pot, both
//      wipers summed on one node)
//   -> output coupling cap -> output load.
//
// Unlike TS808Circuit/DS1Circuit this stage has no vref rail: a triode's
// grid/cathode self-bias loop returns to true (0 V) ground rather than a
// mid-supply virtual ground, so only the B+ supply needs DC-priming source
// stepping.
class PreampCircuit {
public:
    static constexpr float defaultBass = 0.50f;
    static constexpr float defaultTreble = 0.50f;
    static constexpr float supplyVolts = 300.0f;

    struct StageVoltages {
        float grid = 0.0f;
        float plate = 0.0f;
        float cathode = 0.0f;
        float toneIn = 0.0f;
        float toneOut = 0.0f;
        float output = 0.0f;
    };

    bool prepare(double sampleRate) {
        sampleRate_ = std::max(1.0, sampleRate);
        engine_ = MnaCircuitEngine{};

        supply_ = engine_.addNode();
        inputJack_ = engine_.addNode();
        const Node inputCoupled = engine_.addNode();
        grid_ = engine_.addNode();
        plate_ = engine_.addNode();
        cathode_ = engine_.addNode();
        toneIn_ = engine_.addNode();
        const Node bassNode = engine_.addNode();
        const Node trebleNode = engine_.addNode();
        toneOut_ = engine_.addNode();
        const Node outputCouplingInput = engine_.addNode();
        outputNode_ = engine_.addNode();

        // The B+ rail is source-stepped up from 0 V during priming, just like
        // TS808Circuit/DS1Circuit step their 9 V supply, so Newton never has to
        // jump from an all-zero guess directly to a fully biased tube circuit.
        supplySource_ = engine_.addVoltageSource(supply_, ground, 0.0f);
        inputSource_ = engine_.addVoltageSource(inputJack_, ground, 0.0f);

        // Input coupling + grid stopper + grid leak. The grid leak is the
        // stage's DC bias reference: with no grid current, the grid sits at
        // 0 V and the cathode resistor alone sets Vgk (classic cathode
        // self-bias).
        engine_.addCapacitor(inputJack_, inputCoupled,
                             capacitor(22.0e-9f, 400.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(inputCoupled, grid_, resistor(1500.0f));
        engine_.addResistor(grid_, ground, resistor(1.0e6f));

        // 12AX7 common-cathode gain stage.
        triode_ = addTriodeParasiticSubcircuit(engine_, plate_, grid_, cathode_,
                                               hq::component_presets::twelveAX7());
        engine_.addResistor(supply_, plate_, resistor(100000.0f));
        engine_.addResistor(cathode_, ground, resistor(1500.0f));
        engine_.addCapacitor(cathode_, ground,
                             capacitor(25.0e-6f, 10.0f, hq::CapacitorTechnology::electrolytic));

        // Plate coupling into the tone stack's input node. R_toneLoad keeps
        // the coupling cap's far side referenced to ground.
        engine_.addCapacitor(plate_, toneIn_,
                             capacitor(22.0e-9f, 400.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(toneIn_, ground, resistor(1.0e6f));

        // Simplified Fender-style two-band tone control: a low-pass RC feeds
        // the Bass pot, a high-pass RC feeds the Treble pot, and both wipers
        // sum directly onto toneOut_ (a standard passive blend topology).
        engine_.addResistor(toneIn_, bassNode, resistor(10000.0f));
        engine_.addCapacitor(bassNode, ground,
                             capacitor(100.0e-9f, 50.0f, hq::CapacitorTechnology::film));
        bassPot_ = engine_.addPotentiometer(
            bassNode, toneOut_, ground,
            potentiometer(100000.0f, hq::PotTaper::audio, defaultBass));

        engine_.addCapacitor(toneIn_, trebleNode,
                             capacitor(500.0e-12f, 50.0f, hq::CapacitorTechnology::ceramic));
        treblePot_ = engine_.addPotentiometer(
            trebleNode, toneOut_, ground,
            potentiometer(100000.0f, hq::PotTaper::audio, defaultTreble));

        // Output coupling into the load.
        engine_.addCapacitor(toneOut_, outputCouplingInput,
                             capacitor(1.0e-6f, 50.0f, hq::CapacitorTechnology::electrolytic));
        engine_.addResistor(outputCouplingInput, outputNode_, resistor(100.0f));
        engine_.addResistor(outputNode_, ground, resistor(100000.0f));

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return false;

        targetBass_ = appliedBass_ = defaultBass;
        targetTreble_ = appliedTreble_ = defaultTreble;
        controlUpdateCountdown_ = 0;
        lastSolve_ = {};

        // Analytic DC operating-point solve (capacitors open, inductors
        // shorted, source-stepped Newton homotopy) replaces the previous
        // fixed-length silent transient warm-up. The passive tone stack's
        // slowest coupling-cap time constant (on the order of 100 ms: 1 uF
        // output coupling into a ~100 k load) is no longer a concern -- the
        // analytic solve reaches the same equilibrium regardless of how slow
        // the RC is. The 300 V B+ swing is much larger than TS808/DS1's 9 V
        // rail, so this uses more homotopy steps and an extra solve per step
        // to keep each Newton jump small.
        DcOperatingPointOptions dcOptions{};
        dcOptions.sourceSteps = 200;
        dcOptions.solvesPerStep = 3;
        const OperatingPointSourceTarget dcTargets[]{{supplySource_, supplyVolts}};
        const auto dcResult = establishDcOperatingPoint(engine_, dcTargets, dcOptions);
        lastSolve_ = dcResult.lastSolve;
        if (!dcResult.converged || !finiteStages()) return false;

        // Once the DC operating point is established, return to the automatic
        // solver for normal audio processing, matching TS808Circuit/DS1Circuit's
        // matched-residual policy so quiet/silent guitar input converges in a
        // single Newton solve instead of limit-cycling near the noise floor.
        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        engine_.setNonlinearResidualTolerance(2.0e-5f);
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        targetBass_ = appliedBass_ = defaultBass;
        targetTreble_ = appliedTreble_ = defaultTreble;
        controlUpdateCountdown_ = 0;
        engine_.setPotentiometerPosition(bassPot_, appliedBass_);
        engine_.setPotentiometerPosition(treblePot_, appliedTreble_);
        lastSolve_ = {};
    }

    bool setBass(float normalized) noexcept {
        targetBass_ = clampPotPosition(normalized);
        return true;
    }

    bool setTreble(float normalized) noexcept {
        targetTreble_ = clampPotPosition(normalized);
        return true;
    }

    bool setControls(float bass, float treble) noexcept {
        setBass(bass);
        setTreble(treble);
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
        return {engine_.voltage(grid_), engine_.voltage(plate_), engine_.voltage(cathode_),
                engine_.voltage(toneIn_), engine_.voltage(toneOut_), engine_.voltage(outputNode_)};
    }

    float bass() const noexcept { return targetBass_; }
    float treble() const noexcept { return targetTreble_; }
    float appliedBass() const noexcept { return appliedBass_; }
    float appliedTreble() const noexcept { return appliedTreble_; }
    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
    static float approach(float current, float target, float maximumStep) noexcept {
        return current + std::clamp(target - current, -maximumStep, maximumStep);
    }

    void applySmoothedControls() noexcept {
        if (appliedBass_ == targetBass_ && appliedTreble_ == targetTreble_) {
            controlUpdateCountdown_ = 0;
            return;
        }
        if (controlUpdateCountdown_ > 0) {
            --controlUpdateCountdown_;
            return;
        }

        // Potentiometers are control signals, not oversampled audio sources.
        // Update at no less than 24 kHz while retaining the original 5 ms
        // ramp, matching TS808Circuit/DS1Circuit::applySmoothedControls().
        const int updateInterval = std::max(1,
            static_cast<int>(sampleRate_ / 24000.0));
        const float maximumStep = static_cast<float>(updateInterval) /
            static_cast<float>(std::max(1.0, sampleRate_ * 0.005));
        controlUpdateCountdown_ = updateInterval - 1;

        const float nextBass = approach(appliedBass_, targetBass_, maximumStep);
        const float nextTreble = approach(appliedTreble_, targetTreble_, maximumStep);

        if (nextBass != appliedBass_) {
            appliedBass_ = nextBass;
            engine_.setPotentiometerPosition(bassPot_, appliedBass_);
        }
        if (nextTreble != appliedTreble_) {
            appliedTreble_ = nextTreble;
            engine_.setPotentiometerPosition(treblePot_, appliedTreble_);
        }
    }

    bool finiteStages() const noexcept {
        const auto s = stageVoltages();
        return std::isfinite(s.grid) && std::isfinite(s.plate) && std::isfinite(s.cathode) &&
               std::isfinite(s.toneIn) && std::isfinite(s.toneOut) && std::isfinite(s.output);
    }

    // Bass and Treble share a single summing node (toneOut_) through their
    // own potentiometers. At an exact mechanical position of 0.0 or 1.0 one
    // pot's near-zero-resistance leg ties toneOut_ almost directly to a
    // second, independently driven node (bassNode/trebleNode), which the
    // engine's cached sparse Newton solver can resolve inconsistently when
    // *both* pots land on that floor simultaneously -- a fully open Bass and
    // fully open Treble knob at once diverged to a non-physical multi-volt
    // output in testing. A hairline margin away from the mechanical limits
    // is inaudible (real pots don't reach a mathematically exact endpoint
    // either) and keeps every control combination on the well-behaved side
    // of that edge case.
    static constexpr float potPositionMargin = 0.01f;
    static float clampPotPosition(float normalized) noexcept {
        return std::clamp(normalized, potPositionMargin, 1.0f - potPositionMargin);
    }

    static hq::ResistorSpec resistor(float ohms) noexcept {
        hq::ResistorSpec r{};
        r.resistanceOhms = std::max(1.0e-3f, ohms);
        r.tolerancePercent = 5.0f;
        r.powerRatingWatts = 0.5f;
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

    MnaCircuitEngine engine_;
    double sampleRate_ = 48000.0;
    SourceHandle supplySource_{};
    SourceHandle inputSource_{};
    Node supply_ = ground;
    Node inputJack_ = ground;
    Node grid_ = ground;
    Node plate_ = ground;
    Node cathode_ = ground;
    Node toneIn_ = ground;
    Node toneOut_ = ground;
    Node outputNode_ = ground;
    PotHandle bassPot_{};
    PotHandle treblePot_{};
    TriodeParasiticSubcircuit triode_{};
    MnaCircuitEngine::SolveStats lastSolve_{};
    float targetBass_ = defaultBass;
    float targetTreble_ = defaultTreble;
    float appliedBass_ = defaultBass;
    float appliedTreble_ = defaultTreble;
    int controlUpdateCountdown_ = 0;
};

} // namespace guitardsp::circuit
