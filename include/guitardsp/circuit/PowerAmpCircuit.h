#pragma once

#include "OperatingPointContinuation.h"
#include "PentodeParasiticSubcircuit.h"
#include "TransformerSubcircuit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace guitardsp::circuit {

// Component-level single-ended, self-biased EL34 pentode power-amp stage,
// built from ordinary MNA parts exactly like TS808Circuit/DS1Circuit/
// PreampCircuit.
//
// Signal path:
//
//   input coupling cap -> grid stopper -> 220k grid leak (self-bias
//   reference, same idea as PreampCircuit's triode stage)
//   -> EL34 pentode (PentodeParasiticSubcircuit: plate/grid/screen/cathode
//      plus Cgp/Cgk/Cpk/Csk and the positive-grid-current diode)
//   cathode -> Rk to ground, bypassed by a cap (self-bias)
//   B+ -> screen resistor -> screen, bypassed by a cap
//   B+ -> output transformer primary -> plate (the primary winding is the
//      stage's plate load and its DC path -- there is no separate plate
//      resistor, matching a real transformer-coupled power stage)
//   transformer secondary -> 8 ohm dummy speaker-load resistor (the
//      transformer blocks DC, so no output coupling cap is needed)
//
// Like PreampCircuit, this stage has no vref rail: the pentode's grid/
// cathode self-bias loop returns to true (0 V) ground, so only the B+
// supply needs DC-priming source stepping. B+ here is 420 V (EL34's 425 V
// nominal plate/screen rating), a much larger swing than PreampCircuit's
// 300 V triode rail, so priming uses proportionally more/finer steps.
class PowerAmpCircuit {
public:
    static constexpr float supplyVolts = 420.0f;

    struct StageVoltages {
        float grid = 0.0f;
        float plate = 0.0f;
        float screen = 0.0f;
        float cathode = 0.0f;
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
        screen_ = engine_.addNode();
        cathode_ = engine_.addNode();
        outputNode_ = engine_.addNode();

        // The B+ rail is source-stepped up from 0 V during priming, just like
        // PreampCircuit steps its 300 V rail, so Newton never has to jump
        // from an all-zero guess directly to a fully biased, high-voltage
        // pentode operating point.
        supplySource_ = engine_.addVoltageSource(supply_, ground, 0.0f);
        inputSource_ = engine_.addVoltageSource(inputJack_, ground, 0.0f);

        // Input coupling + grid stopper + grid leak. The grid leak is the
        // stage's DC bias reference: with no grid current, the grid sits at
        // 0 V and the cathode resistor alone sets Vgk (classic cathode
        // self-bias), same pattern as PreampCircuit's triode stage.
        engine_.addCapacitor(inputJack_, inputCoupled,
                             capacitor(100.0e-9f, 100.0f, hq::CapacitorTechnology::film));
        engine_.addResistor(inputCoupled, grid_, resistor(1500.0f));
        engine_.addResistor(grid_, ground, resistor(220000.0f));

        // EL34 single-ended power stage.
        pentode_ = addPentodeParasiticSubcircuit(engine_, plate_, grid_, screen_, cathode_,
                                                  hq::component_presets::pentodeEl34());

        // Cathode self-bias.
        engine_.addResistor(cathode_, ground, resistor(cathodeResistorOhms));
        engine_.addCapacitor(cathode_, ground,
                             capacitor(47.0e-6f, 63.0f, hq::CapacitorTechnology::electrolytic));

        // Screen supply, dropped and bypassed off B+.
        engine_.addResistor(supply_, screen_, resistor(470.0f));
        engine_.addCapacitor(screen_, ground,
                             capacitor(22.0e-6f, 450.0f, hq::CapacitorTechnology::electrolytic));

        // Output transformer: primary is the plate load and the plate's DC
        // path (B+ to plate), secondary drives a dummy 8 ohm speaker load.
        // turnsRatio ~19.4 reflects an 8 ohm load to roughly the 3 k plate
        // load typical of a single-ended EL34 stage (turnsRatio^2 * 8 ~ 3k).
        hq::TransformerSpec outputTransformer{};
        outputTransformer.name = "EL34 SE Output Transformer";
        outputTransformer.primaryInductanceHenries = 15.0f;
        outputTransformer.leakageInductanceHenries = 25.0e-3f;
        outputTransformer.primaryResistanceOhms = 200.0f;
        outputTransformer.secondaryResistanceOhms = 0.2f;
        outputTransformer.turnsRatio = 19.4f;
        outputTransformer.interwindingCapacitanceFarads = 100.0e-12f;
        outputTransformer.saturationFluxNormalized = 1.0f;
        // The single-ended primary carries the full idle plate current (no
        // push-pull DC cancellation), so the magnetizing branch's assumed
        // saturation current must clear that idle bias with headroom for AC
        // swing rather than the smaller push-pull-style default.
        outputTransformer.magnetizingSaturationCurrentAmps = 0.15f;
        outputTransformer.coreSaturationExponent = 2.0f;
        outputTransformer.minimumMagnetizingInductanceRatio = 0.08f;
        outputTransformer_ = outputTransformer;
        transformer_ = addTransformerSubcircuit(engine_, supply_, plate_,
                                                 outputNode_, ground, outputTransformer_);

        engine_.addResistor(outputNode_, ground, resistor(8.0f));

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return false;

        lastSolve_ = {};

        // Analytic DC operating-point solve (capacitors open, inductors
        // shorted, source-stepped Newton homotopy) replaces the previous
        // fixed-length silent transient warm-up: the cathode bypass cap's RC
        // (47 uF into ~1.2k, tens of ms) no longer needs to be waited out. The
        // 420 V B+ swing is larger than PreampCircuit's 300 V rail, so this
        // uses proportionally more homotopy steps.
        DcOperatingPointOptions dcOptions{};
        dcOptions.sourceSteps = 280;
        dcOptions.solvesPerStep = 3;
        // The default 1e-6 relative Newton delta tolerance sits right at this
        // stage's achievable float32 precision floor once the transformer's
        // VCVS/CCCS/zero-volt-current-sense loop is combined with 400+ V node
        // magnitudes (~4e-5 V absolute noise per accumulated-rounding solve,
        // an order of magnitude above 1e-6 relative of 400 V): the scaled
        // delta bounces around that floor forever instead of ever dropping
        // below it. 1e-5 is comfortably above that floor while still tight
        // enough for a converged operating-point seed.
        dcOptions.newtonTolerance = 1.0e-5f;
        const OperatingPointSourceTarget dcTargets[]{{supplySource_, supplyVolts}};
        const auto dcResult = establishDcOperatingPoint(engine_, dcTargets, dcOptions);
        lastSolve_ = dcResult.lastSolve;
        if (!dcResult.converged || !finiteStages()) return false;

        // The DC solve shorts every inductor, including the output
        // transformer's magnetizing branch, so its saturation-dependent
        // inductance spec was never exercised during the homotopy. Push one
        // update through now, from the just-solved idle magnetizing current,
        // so the first real trapezoidal sample already uses the correctly
        // saturated inductance instead of the unsaturated nominal value.
        // setInductorSpec() only marks the cache dirty; refresh it explicitly
        // so prepare() doesn't leave a pending rebuild for the first
        // real-time sample to discover as a surprise.
        updateOutputTransformerSaturation();
        engine_.refreshStaticCache();

        // Once the DC operating point is established, return to the automatic
        // solver for normal audio processing, matching PreampCircuit/
        // TS808Circuit/DS1Circuit's matched-residual policy so quiet/silent
        // guitar input converges in a single Newton solve instead of
        // limit-cycling near the noise floor.
        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        engine_.setNonlinearResidualTolerance(2.0e-5f);
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        lastSolve_ = {};
        lastMagnetizingInductanceHenries_ = -1.0f;
    }

    float processSample(float input) noexcept {
        engine_.setVoltageSource(inputSource_, input);
        lastSolve_ = engine_.processSample(40, 2.0e-5f);
        updateOutputTransformerSaturation();
        const float out = engine_.voltage(outputNode_);
        if (lastSolve_.singular || !std::isfinite(out)) return 0.0f;
        return out;
    }

    StageVoltages stageVoltages() const noexcept {
        return {engine_.voltage(grid_), engine_.voltage(plate_), engine_.voltage(screen_),
                engine_.voltage(cathode_), engine_.voltage(outputNode_)};
    }

    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
    static constexpr float cathodeResistorOhms = 1200.0f;

    // updateTransformerCoreSaturation() unconditionally calls
    // MnaCircuitEngine::setInductorSpec(), which marks the engine's static
    // matrix cache dirty and forces a full rebuild on the next solve
    // (MnaCircuitEngineCore::rebuildStaticCache() walks every component).
    // At silence/near-DC the magnetizing current -- and therefore the
    // saturation-adjusted inductance -- barely moves sample to sample, so
    // committing every sample would force a static-cache rebuild on every
    // single audio sample even with no input. Only push the update through
    // when the inductance has actually moved by more than float noise,
    // matching the "no static rebuilds while idle" behaviour PreampCircuit/
    // TS808Circuit/DS1Circuit already rely on.
    void updateOutputTransformerSaturation() noexcept {
        const float current = engine_.inductorCurrent(
            static_cast<std::size_t>(transformer_.magnetizing));
        const float inductance = detail::saturatedMagnetizingInductance(outputTransformer_, current);
        const float threshold = std::max(1.0e-9f, std::abs(lastMagnetizingInductanceHenries_) * 1.0e-4f);
        if (std::abs(inductance - lastMagnetizingInductanceHenries_) > threshold) {
            engine_.setInductorSpec(transformer_.magnetizing,
                                    detail::magnetizingSpec(outputTransformer_, inductance));
            lastMagnetizingInductanceHenries_ = inductance;
        }
    }

    bool finiteStages() const noexcept {
        const auto s = stageVoltages();
        return std::isfinite(s.grid) && std::isfinite(s.plate) && std::isfinite(s.screen) &&
               std::isfinite(s.cathode) && std::isfinite(s.output);
    }

    static hq::ResistorSpec resistor(float ohms) noexcept {
        hq::ResistorSpec r{};
        r.resistanceOhms = std::max(1.0e-3f, ohms);
        r.tolerancePercent = 5.0f;
        r.powerRatingWatts = 2.0f;
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

    MnaCircuitEngine engine_;
    double sampleRate_ = 48000.0;
    SourceHandle supplySource_{};
    SourceHandle inputSource_{};
    Node supply_ = ground;
    Node inputJack_ = ground;
    Node grid_ = ground;
    Node plate_ = ground;
    Node screen_ = ground;
    Node cathode_ = ground;
    Node outputNode_ = ground;
    PentodeParasiticSubcircuit pentode_{};
    TransformerSubcircuit transformer_{};
    hq::TransformerSpec outputTransformer_{};
    float lastMagnetizingInductanceHenries_ = -1.0f;
    MnaCircuitEngine::SolveStats lastSolve_{};
};

} // namespace guitardsp::circuit
