#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// Explicit capacitance packs for the high-fidelity active-device path. The base
// ComponentCatalog currently carries one compact capacitance field per transistor
// family; these packs split that metadata into individually editable terminals.
// Callers can replace every value from datasheet/SPICE/measurement data later
// without changing topology or the MNA device stamp.
struct BjtParasiticValues {
    float baseEmitterFarads = 0.0f;
    float baseCollectorFarads = 0.0f;

    static BjtParasiticValues fromSpec(const hq::BJTSpec& spec) noexcept {
        const float total = std::max(0.0f, spec.inputCapacitanceFarads);
        return {0.70f * total, 0.30f * total};
    }
};

struct JfetParasiticValues {
    float gateSourceFarads = 0.0f;
    float gateDrainFarads = 0.0f;
    float drainSourceFarads = 0.0f;

    static JfetParasiticValues fromSpec(const hq::JFETSpec& spec) noexcept {
        const float cgs = std::max(0.0f, spec.gateSourceCapacitanceFarads);
        return {cgs, 0.35f * cgs, 0.15f * cgs};
    }
};

struct MosfetParasiticValues {
    float gateSourceFarads = 0.0f;
    float gateDrainFarads = 0.0f;
    float drainSourceFarads = 0.0f;

    static MosfetParasiticValues fromSpec(const hq::MOSFETSpec& spec) noexcept {
        const float gate = std::max(0.0f, spec.gateCapacitanceFarads);
        return {0.72f * gate, 0.28f * gate, 0.18f * gate};
    }
};

struct BjtParasiticSubcircuit {
    BjtHandle transistor{};
    CapacitorHandle baseEmitterCapacitance{};
    CapacitorHandle baseCollectorCapacitance{};
};

struct JfetParasiticSubcircuit {
    JfetHandle transistor{};
    CapacitorHandle gateSourceCapacitance{};
    CapacitorHandle gateDrainCapacitance{};
    CapacitorHandle drainSourceCapacitance{};
};

struct MosfetParasiticSubcircuit {
    MosfetHandle transistor{};
    CapacitorHandle gateSourceCapacitance{};
    CapacitorHandle gateDrainCapacitance{};
    CapacitorHandle drainSourceCapacitance{};
    DiodeHandle bodyDiode{};
};

namespace active_device_detail {
inline hq::CapacitorSpec parasiticCapacitor(float farads) noexcept {
    hq::CapacitorSpec c{};
    c.capacitanceFarads = std::max(0.0f, farads);
    c.tolerancePercent = 0.0f;
    c.voltageRatingVolts = 1000.0f;
    c.esrOhms = 0.0f;
    c.leakageResistanceOhms = 1.0e12f;
    c.dielectricAbsorption = 0.0f;
    c.technology = hq::CapacitorTechnology::generic;
    return c;
}

inline hq::DiodeSpec mosfetBodyDiode(const hq::MOSFETSpec& spec) noexcept {
    hq::DiodeSpec diode{};
    diode.name = "MOSFET body diode";
    diode.technology = hq::DiodeTechnology::silicon;
    diode.nominalForwardVoltage = std::max(0.05f, spec.bodyDiodeForwardVoltage);
    diode.emissionCoefficient = 1.8f;
    diode.thermalVoltage = 0.02585f;
    constexpr float referenceCurrent = 1.0e-3f;
    const float exponent = std::clamp(diode.nominalForwardVoltage /
        (diode.emissionCoefficient * diode.thermalVoltage), 1.0f, 30.0f);
    diode.saturationCurrent = referenceCurrent /
        std::max(1.0f, std::exp(exponent) - 1.0f);
    diode.seriesResistanceOhms = 1.0f;
    diode.junctionCapacitanceFarads = 0.0f;
    diode.reverseVoltageRating = std::max(1.0f, spec.maxDrainSourceVoltage);
    diode.currentRatingAmps = 1.0f;
    return diode;
}
} // namespace active_device_detail

inline BjtParasiticSubcircuit addBjtParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node collector,
        Node base,
        Node emitter,
        const hq::BJTSpec& spec,
        BjtParasiticValues parasitics) {
    BjtParasiticSubcircuit handles{};
    handles.transistor = engine.addBjt(collector, base, emitter, spec);
    handles.baseEmitterCapacitance = engine.addCapacitor(base, emitter,
        active_device_detail::parasiticCapacitor(parasitics.baseEmitterFarads));
    handles.baseCollectorCapacitance = engine.addCapacitor(base, collector,
        active_device_detail::parasiticCapacitor(parasitics.baseCollectorFarads));
    return handles;
}

inline BjtParasiticSubcircuit addBjtParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node collector,
        Node base,
        Node emitter,
        const hq::BJTSpec& spec) {
    return addBjtParasiticSubcircuit(engine, collector, base, emitter, spec,
                                     BjtParasiticValues::fromSpec(spec));
}

inline bool updateBjtParasiticSubcircuit(MnaCircuitEngine& engine,
                                         const BjtParasiticSubcircuit& handles,
                                         const hq::BJTSpec& spec,
                                         BjtParasiticValues parasitics) noexcept {
    bool ok = true;
    ok &= engine.setBjtSpec(handles.transistor, spec);
    ok &= engine.setCapacitorSpec(handles.baseEmitterCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.baseEmitterFarads));
    ok &= engine.setCapacitorSpec(handles.baseCollectorCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.baseCollectorFarads));
    return ok;
}

inline JfetParasiticSubcircuit addJfetParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node drain,
        Node gate,
        Node source,
        const hq::JFETSpec& spec,
        JfetParasiticValues parasitics) {
    JfetParasiticSubcircuit handles{};
    handles.transistor = engine.addJfet(drain, gate, source, spec);
    handles.gateSourceCapacitance = engine.addCapacitor(gate, source,
        active_device_detail::parasiticCapacitor(parasitics.gateSourceFarads));
    handles.gateDrainCapacitance = engine.addCapacitor(gate, drain,
        active_device_detail::parasiticCapacitor(parasitics.gateDrainFarads));
    handles.drainSourceCapacitance = engine.addCapacitor(drain, source,
        active_device_detail::parasiticCapacitor(parasitics.drainSourceFarads));
    return handles;
}

inline JfetParasiticSubcircuit addJfetParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node drain,
        Node gate,
        Node source,
        const hq::JFETSpec& spec) {
    return addJfetParasiticSubcircuit(engine, drain, gate, source, spec,
                                      JfetParasiticValues::fromSpec(spec));
}

inline bool updateJfetParasiticSubcircuit(MnaCircuitEngine& engine,
                                          const JfetParasiticSubcircuit& handles,
                                          const hq::JFETSpec& spec,
                                          JfetParasiticValues parasitics) noexcept {
    bool ok = true;
    ok &= engine.setJfetSpec(handles.transistor, spec);
    ok &= engine.setCapacitorSpec(handles.gateSourceCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.gateSourceFarads));
    ok &= engine.setCapacitorSpec(handles.gateDrainCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.gateDrainFarads));
    ok &= engine.setCapacitorSpec(handles.drainSourceCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.drainSourceFarads));
    return ok;
}

inline MosfetParasiticSubcircuit addMosfetParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node drain,
        Node gate,
        Node source,
        const hq::MOSFETSpec& spec,
        MosfetParasiticValues parasitics) {
    MosfetParasiticSubcircuit handles{};
    handles.transistor = engine.addMosfet(drain, gate, source, spec);
    handles.gateSourceCapacitance = engine.addCapacitor(gate, source,
        active_device_detail::parasiticCapacitor(parasitics.gateSourceFarads));
    handles.gateDrainCapacitance = engine.addCapacitor(gate, drain,
        active_device_detail::parasiticCapacitor(parasitics.gateDrainFarads));
    handles.drainSourceCapacitance = engine.addCapacitor(drain, source,
        active_device_detail::parasiticCapacitor(parasitics.drainSourceFarads));

    const auto diode = active_device_detail::mosfetBodyDiode(spec);
    if (spec.polarity == hq::TransistorPolarity::pChannel)
        handles.bodyDiode = engine.addDiode(drain, source, diode);
    else
        handles.bodyDiode = engine.addDiode(source, drain, diode);
    return handles;
}

inline MosfetParasiticSubcircuit addMosfetParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node drain,
        Node gate,
        Node source,
        const hq::MOSFETSpec& spec) {
    return addMosfetParasiticSubcircuit(engine, drain, gate, source, spec,
                                        MosfetParasiticValues::fromSpec(spec));
}

inline bool updateMosfetParasiticSubcircuit(MnaCircuitEngine& engine,
                                            const MosfetParasiticSubcircuit& handles,
                                            const hq::MOSFETSpec& spec,
                                            MosfetParasiticValues parasitics) noexcept {
    bool ok = true;
    ok &= engine.setMosfetSpec(handles.transistor, spec);
    ok &= engine.setCapacitorSpec(handles.gateSourceCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.gateSourceFarads));
    ok &= engine.setCapacitorSpec(handles.gateDrainCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.gateDrainFarads));
    ok &= engine.setCapacitorSpec(handles.drainSourceCapacitance,
        active_device_detail::parasiticCapacitor(parasitics.drainSourceFarads));
    ok &= engine.setDiodeSpec(handles.bodyDiode,
        active_device_detail::mosfetBodyDiode(spec));
    return ok;
}

} // namespace guitardsp::circuit
