#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// Two-junction Ebers-Moll-style BJT macro assembled entirely from existing MNA
// primitives. This is a much better fit for complete pedal circuits than the old
// beta-multiplied forward-active helper because both B-E and B-C junctions remain
// explicit and saturation emerges when the B-C junction becomes forward biased.
//
// For an NPN device, define the two junction currents as
//   IF : base -> emitter diode current
//   IR : base -> collector diode current
// and use the classic Ebers-Moll terminal-current decomposition
//   IC = alphaF * IF - IR
//   IE = -IF + alphaR * IR
//   IB = (1-alphaF) * IF + (1-alphaR) * IR.
//
// The implementation realizes those equations with two diode branches sensed by
// zero-volt sources plus two CCCSs. PNP uses the same equations with junction
// polarity reversed. No beta-sized controlled current source is required, which
// materially improves Newton conditioning in floating emitter followers.
//
// Accuracy boundary: the catalog currently exposes one forward beta but no reverse
// beta, Early voltage, high-injection or charge-storage parameters. alphaR therefore
// uses a conservative engineering reverse-beta default. The junction capacitances
// are explicit and editable through BJTSpec::inputCapacitanceFarads.
struct BjtEbersMollSubcircuit {
    Node forwardSenseNode = ground;
    Node reverseSenseNode = ground;
    SourceHandle forwardCurrentSense{};
    SourceHandle reverseCurrentSense{};
    DiodeHandle baseEmitterJunction{};
    DiodeHandle baseCollectorJunction{};
    ControlledSourceHandle forwardTransport{};
    ControlledSourceHandle reverseTransport{};
    CapacitorHandle baseEmitterCapacitance{};
    CapacitorHandle baseCollectorCapacitance{};
    ResistorHandle collectorEmitterLeak{};
};

namespace bjt_ebers_moll_detail {

inline float forwardAlpha(const hq::BJTSpec& spec) noexcept {
    const float beta = std::max(1.0f, spec.beta);
    return beta / (beta + 1.0f);
}

inline constexpr float reverseBeta() noexcept {
    // Typical small-signal BJTs have reverse beta far below forward beta. The exact
    // value is part-specific; 2 is deliberately conservative until the catalog
    // grows a dedicated betaR / alphaR field or imports SPICE model parameters.
    return 2.0f;
}

inline float reverseAlpha() noexcept {
    constexpr float betaR = reverseBeta();
    return betaR / (betaR + 1.0f);
}

inline hq::DiodeSpec junction(const hq::BJTSpec& spec, const char* name) noexcept {
    hq::DiodeSpec diode{};
    diode.name = name;
    diode.technology = hq::DiodeTechnology::silicon;
    diode.nominalForwardVoltage = std::max(0.1f, spec.nominalVbe);
    diode.emissionCoefficient = 1.0f;
    diode.thermalVoltage = std::max(1.0e-4f, spec.thermalVoltage);

    // Base spreading / junction series resistance. At the nominal milliamp-scale
    // operating point this has negligible DC effect, while at a bad Newton iterate
    // it prevents the exponential branch from becoming unrealistically stiff.
    diode.seriesResistanceOhms = 75.0f;
    diode.junctionCapacitanceFarads = 0.0f; // explicit terminal caps below
    diode.reverseVoltageRating = std::max(5.0f, spec.maxCollectorVoltage);
    diode.currentRatingAmps = std::max(1.0e-3f, spec.maxCollectorCurrentAmps);

    // Calibrate the forward junction so nominalVbe corresponds to ~1 mA emitter
    // junction current. beta then controls the small difference between emitter and
    // collector transport through alphaF rather than multiplying a tiny base current.
    constexpr float referenceJunctionCurrent = 1.0e-3f;
    const float exponent = std::clamp(diode.nominalForwardVoltage /
                                      (diode.emissionCoefficient * diode.thermalVoltage),
                                      1.0f, 40.0f);
    diode.saturationCurrent = referenceJunctionCurrent /
        std::max(1.0f, std::exp(exponent) - 1.0f);
    return diode;
}

inline hq::CapacitorSpec junctionCap(float farads, float voltageRating) noexcept {
    hq::CapacitorSpec c{};
    c.capacitanceFarads = std::max(0.0f, farads);
    c.tolerancePercent = 0.0f;
    c.voltageRatingVolts = std::max(5.0f, voltageRating);
    c.esrOhms = 0.0f;
    c.leakageResistanceOhms = 1.0e12f;
    c.dielectricAbsorption = 0.0f;
    c.technology = hq::CapacitorTechnology::generic;
    return c;
}

inline hq::ResistorSpec ceLeak() noexcept {
    hq::ResistorSpec r{};
    r.resistanceOhms = 100.0e6f;
    r.tolerancePercent = 0.0f;
    r.powerRatingWatts = 1.0f;
    return r;
}

} // namespace bjt_ebers_moll_detail

inline BjtEbersMollSubcircuit addBjtEbersMollSubcircuit(
        MnaCircuitEngine& engine,
        Node collector,
        Node base,
        Node emitter,
        const hq::BJTSpec& spec) {
    BjtEbersMollSubcircuit handles{};
    handles.forwardSenseNode = engine.addNode();
    handles.reverseSenseNode = engine.addNode();

    // Both sense sources retain a base->junction-node orientation. For PNP the
    // diode directions reverse, so the sensed currents naturally become negative;
    // the same controlled-source gains then reverse transport direction as required.
    handles.forwardCurrentSense = engine.addVoltageSource(base, handles.forwardSenseNode, 0.0f);
    handles.reverseCurrentSense = engine.addVoltageSource(base, handles.reverseSenseNode, 0.0f);

    const bool pnp = spec.polarity == hq::TransistorPolarity::pnp;
    if (pnp) {
        handles.baseEmitterJunction = engine.addDiode(emitter, handles.forwardSenseNode,
            bjt_ebers_moll_detail::junction(spec, "BJT B-E junction"));
        handles.baseCollectorJunction = engine.addDiode(collector, handles.reverseSenseNode,
            bjt_ebers_moll_detail::junction(spec, "BJT B-C junction"));
    } else {
        handles.baseEmitterJunction = engine.addDiode(handles.forwardSenseNode, emitter,
            bjt_ebers_moll_detail::junction(spec, "BJT B-E junction"));
        handles.baseCollectorJunction = engine.addDiode(handles.reverseSenseNode, collector,
            bjt_ebers_moll_detail::junction(spec, "BJT B-C junction"));
    }

    // Forward transport: collector -> base for NPN. With a PNP junction the sensed
    // IF is negative, so the same positive alphaF automatically yields base ->
    // collector current, i.e. physical emitter-to-collector transport after the
    // terminal-current equations are combined.
    handles.forwardTransport = engine.addCccs(collector, base,
        handles.forwardCurrentSense, bjt_ebers_moll_detail::forwardAlpha(spec));

    // Reverse transport: emitter -> base. This is what allows the B-C junction to
    // participate during saturation instead of treating the collector as a passive
    // current sink.
    handles.reverseTransport = engine.addCccs(emitter, base,
        handles.reverseCurrentSense, bjt_ebers_moll_detail::reverseAlpha());

    const float total = std::max(0.0f, spec.inputCapacitanceFarads);
    handles.baseEmitterCapacitance = engine.addCapacitor(base, emitter,
        bjt_ebers_moll_detail::junctionCap(total * 0.70f, spec.maxCollectorVoltage));
    handles.baseCollectorCapacitance = engine.addCapacitor(base, collector,
        bjt_ebers_moll_detail::junctionCap(total * 0.30f, spec.maxCollectorVoltage));
    handles.collectorEmitterLeak = engine.addResistor(collector, emitter,
        bjt_ebers_moll_detail::ceLeak());
    return handles;
}

} // namespace guitardsp::circuit
