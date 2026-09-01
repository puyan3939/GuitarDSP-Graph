#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// Numerically robust forward-active BJT macro built from ordinary MNA primitives.
//
// The raw three-terminal BJT stamp is useful as a compact engineering device, but
// a floating/base-biased emitter follower can present it with a very large initial
// Vbe during startup. This macro expresses the same forward-active mechanism in a
// SPICE-like decomposed form: a nonlinear base-emitter junction carries base current,
// a zero-volt source senses that branch current, and a CCCS transfers beta times that
// current from collector to emitter. The diode already uses the engine's robust
// series-resistance Newton linearization, so startup from an uninitialized state is
// substantially better conditioned.
//
// This is deliberately *forward-active*, not yet a full Ebers-Moll/Gummel-Poon model.
// It is appropriate for emitter followers such as the TS808 input/output buffers.
// Saturating gain stages such as the DS-1 transistor front end will use the later
// two-junction BJT model.
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

    // A transistor's intrinsic B-E junction is not connected to the external base
    // through zero ohms. A modest base-spreading resistance both reflects that
    // physical loss and prevents the beta-controlled collector source from turning
    // a badly initialized exponential junction into an unrealistically stiff
    // hundreds-of-siemens Newton stamp. The nominal operating-point error is tiny
    // because base current is only a few microamps in these pedal buffers.
    diode.seriesResistanceOhms = 75.0f;
    diode.junctionCapacitanceFarads = 0.0f; // explicit capacitors below
    diode.reverseVoltageRating = std::max(5.0f, spec.maxCollectorVoltage);
    diode.currentRatingAmps = std::max(1.0e-3f, spec.maxCollectorCurrentAmps);

    // Calibrate the junction so nominalVbe corresponds to ~1 mA collector
    // current at the requested beta. This makes the catalog's Vbe/beta fields
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

    const bool pnp = spec.polarity == hq::TransistorPolarity::pnp;
    if (pnp) {
        // PNP forward junction current runs emitter -> base. The voltage-source
        // sensor keeps the same base->sense orientation, therefore sensed current
        // becomes negative and the same positive beta transfer below naturally
        // reverses collector current to emitter -> collector.
        handles.baseEmitterJunction = engine.addDiode(emitter, handles.baseSenseNode,
                                                       bjt_forward_detail::baseEmitterJunction(spec));
    } else {
        handles.baseEmitterJunction = engine.addDiode(handles.baseSenseNode, emitter,
                                                       bjt_forward_detail::baseEmitterJunction(spec));
    }

    // The sensed branch current is positive when NPN base current leaves the base
    // node toward the B-E junction. A positive CCCS gain therefore produces the
    // required collector -> emitter current. For PNP the sensed current is negative,
    // so this same stamp automatically yields emitter -> collector current.
    handles.collectorTransfer = engine.addCccs(collector, emitter,
        handles.baseCurrentSense, std::max(1.0f, spec.beta));

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
