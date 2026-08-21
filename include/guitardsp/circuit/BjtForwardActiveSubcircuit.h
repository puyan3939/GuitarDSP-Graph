#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// Numerically robust forward-active BJT macro built from ordinary MNA primitives.
//
// The raw three-terminal BJT stamp is useful as a compact engineering device, but
// a floating/base-biased emitter follower can present it with a very large initial
// Vbe during startup.  This macro expresses the same forward-active mechanism in a
// SPICE-like decomposed form: a nonlinear base-emitter junction carries base current,
// a zero-volt source senses that branch current, and a CCCS transfers beta times that
// current from collector to emitter.  The diode already uses the engine's robust
// series-resistance Newton linearization, so startup from an uninitialized state is
// substantially better conditioned.
//
// This is deliberately *forward-active*, not yet a full Ebers-Moll/Gummel-Poon model.
// It is appropriate for the TS808's emitter followers.  Saturating gain stages such
// as the DS-1 transistor front end will get the later two-junction model.
struct BjtForwardActiveSubcircuit {
    Node baseSenseNode = ground;
    SourceHandle baseCurrentSense{};
    DiodeHandle baseEmitterJunction{};
    ControlledSourceHandle collectorTransfer{};
    CapacitorHandle baseEmitterCapacitance{};
    CapacitorHandle baseCollectorCapacitance{};
    ResistorHandle collectorEmitterLeak{};
};

namespace bjt_forward_detail {
inline hq::DiodeSpec baseEmitterJunction(const hq::BJTSpec& spec) noexcept {
    hq::DiodeSpec diode{};
    diode.name = "BJT base-emitter junction";
    diode.nominalForwardVoltage = std::max(0.1f, spec.nominalVbe);
    diode.emissionCoefficient = 1.0f;
    diode.thermalVoltage = std::max(1.0e-4f, spec.thermalVoltage);
    diode.seriesResistanceOhms = 1.0f;
    diode.junctionCapacitanceFarads = 0.0f; // explicit capacitors below
    diode.reverseVoltageRating = std::max(5.0f, spec.maxCollectorVoltage);
    diode.currentRatingAmps = std::max(1.0e-3f, spec.maxCollectorCurrentAmps);

    // Calibrate the junction so nominalVbe corresponds to ~1 mA collector
    // current at the requested beta.  This makes the catalog's Vbe/beta fields
    // actually determine the operating point instead of relying on a hidden Is.
    const float beta = std::max(1.0f, spec.beta);
    const float referenceBaseCurrent = 1.0e-3f / beta;
    const float exponent = std::clamp(diode.nominalForwardVoltage /
                                      (diode.emissionCoefficient * diode.thermalVoltage),
                                      1.0f, 40.0f);
    diode.saturationCurrent = referenceBaseCurrent /
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
} // namespace bjt_forward_detail

inline BjtForwardActiveSubcircuit addBjtForwardActiveSubcircuit(
        MnaCircuitEngine& engine,
        Node collector,
        Node base,
        Node emitter,
        const hq::BJTSpec& spec) {
    BjtForwardActiveSubcircuit handles{};
    handles.baseSenseNode = engine.addNode();
    handles.baseCurrentSense = engine.addVoltageSource(base, handles.baseSenseNode, 0.0f);
    handles.baseEmitterJunction = engine.addDiode(handles.baseSenseNode, emitter,
                                                   bjt_forward_detail::baseEmitterJunction(spec));

    const float polarity = spec.polarity == hq::TransistorPolarity::pnp ? -1.0f : 1.0f;
    // MNA voltage-source branch current is negative when the source supplies the
    // sensed base current, hence the minus sign for an NPN forward-current transfer.
    handles.collectorTransfer = engine.addCccs(collector, emitter,
        handles.baseCurrentSense, -polarity * std::max(1.0f, spec.beta));

    const float total = std::max(0.0f, spec.inputCapacitanceFarads);
    handles.baseEmitterCapacitance = engine.addCapacitor(base, emitter,
        bjt_forward_detail::junctionCap(total * 0.70f, spec.maxCollectorVoltage));
    handles.baseCollectorCapacitance = engine.addCapacitor(base, collector,
        bjt_forward_detail::junctionCap(total * 0.30f, spec.maxCollectorVoltage));
    handles.collectorEmitterLeak = engine.addResistor(collector, emitter,
                                                       bjt_forward_detail::ceLeak());
    return handles;
}

} // namespace guitardsp::circuit
