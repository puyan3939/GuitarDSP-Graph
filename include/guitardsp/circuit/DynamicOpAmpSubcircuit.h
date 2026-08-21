#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// Circuit-level op-amp macro assembled from ordinary MNA primitives.
//
// Implemented today:
// - finite DC open-loop gain
// - dominant-pole / gain-bandwidth behavior
// - input offset and input bias current
// - finite output resistance
//
// Rail limiting, slew-rate and output-current limiting intentionally remain future
// work. A first diode-clamp experiment was removed because a hard overdrive could
// drive the internal high-gain state into Newton non-convergence. Shipping an
// explicit boundary is preferable to hiding an unstable "rail" approximation.
struct DynamicOpAmpSubcircuit {
    Node dominantPole = ground;
    Node outputDrive = ground;
    Node offsetNode = ground;

    ResistorHandle dominantResistance{};
    CapacitorHandle dominantCapacitance{};
    ControlledSourceHandle differentialGm{};
    ControlledSourceHandle offsetGm{};
    SourceHandle offsetVoltage{};
    ControlledSourceHandle outputFollower{};
    ResistorHandle outputResistance{};
};

namespace detail {
inline float opAmpOpenLoopGain(const hq::OpAmpSpec& spec) noexcept {
    return std::clamp(std::pow(10.0f, spec.openLoopGainDb / 20.0f), 1.0f, 1.0e7f);
}

inline hq::ResistorSpec opAmpDominantResistance() noexcept {
    hq::ResistorSpec r{};
    r.resistanceOhms = 1.0e6f;
    r.tolerancePercent = 0.0f;
    r.powerRatingWatts = 100.0f;
    return r;
}

inline hq::CapacitorSpec opAmpDominantCapacitance(const hq::OpAmpSpec& spec) noexcept {
    constexpr float pi = 3.14159265358979323846f;
    const float a0 = opAmpOpenLoopGain(spec);
    const float gbw = std::max(1.0f, spec.gainBandwidthHz);
    const float pole = std::max(0.01f, gbw / a0);
    const float r = opAmpDominantResistance().resistanceOhms;
    hq::CapacitorSpec c{};
    c.capacitanceFarads = 1.0f / (2.0f * pi * r * pole);
    c.esrOhms = 0.0f;
    c.leakageResistanceOhms = 1.0e12f;
    c.voltageRatingVolts = 1000.0f;
    c.technology = hq::CapacitorTechnology::generic;
    return c;
}

inline float opAmpInputTransconductance(const hq::OpAmpSpec& spec) noexcept {
    return opAmpOpenLoopGain(spec) / opAmpDominantResistance().resistanceOhms;
}

inline hq::ResistorSpec opAmpOutputResistance(const hq::OpAmpSpec& spec) noexcept {
    hq::ResistorSpec r{};
    r.resistanceOhms = std::max(0.01f, spec.outputResistanceOhms);
    r.tolerancePercent = 0.0f;
    r.powerRatingWatts = 100.0f;
    return r;
}
} // namespace detail

inline DynamicOpAmpSubcircuit addDynamicOpAmpSubcircuit(MnaCircuitEngine& engine,
                                                         Node output,
                                                         Node nonInverting,
                                                         Node inverting,
                                                         Node positiveRail,
                                                         Node negativeRail,
                                                         Node reference,
                                                         const hq::OpAmpSpec& spec) {
    // Supply nodes are retained in the public contract so a future bounded-output
    // stamp can be introduced without changing CircuitNetlist topology. They are
    // intentionally unused until that stamp is numerically robust.
    (void)positiveRail;
    (void)negativeRail;

    DynamicOpAmpSubcircuit handles{};
    handles.dominantPole = engine.addNode();
    handles.outputDrive = engine.addNode();
    handles.offsetNode = engine.addNode();

    handles.dominantResistance = engine.addResistor(handles.dominantPole, reference,
                                                     detail::opAmpDominantResistance());
    handles.dominantCapacitance = engine.addCapacitor(handles.dominantPole, reference,
                                                       detail::opAmpDominantCapacitance(spec));

    const float gm = detail::opAmpInputTransconductance(spec);
    handles.differentialGm = engine.addVccs(reference, handles.dominantPole,
                                             nonInverting, inverting, gm);

    handles.offsetVoltage = engine.addVoltageSource(handles.offsetNode, reference,
                                                     spec.inputOffsetVoltage);
    handles.offsetGm = engine.addVccs(reference, handles.dominantPole,
                                      handles.offsetNode, reference, gm);

    if (spec.inputBiasCurrentAmps != 0.0f) {
        engine.addCurrentSource(nonInverting, reference, spec.inputBiasCurrentAmps);
        engine.addCurrentSource(inverting, reference, spec.inputBiasCurrentAmps);
    }

    handles.outputFollower = engine.addVcvs(handles.outputDrive, reference,
                                             handles.dominantPole, reference, 1.0f);
    handles.outputResistance = engine.addResistor(handles.outputDrive, output,
                                                   detail::opAmpOutputResistance(spec));
    return handles;
}

inline bool updateDynamicOpAmpSubcircuit(MnaCircuitEngine& engine,
                                         const DynamicOpAmpSubcircuit& handles,
                                         const hq::OpAmpSpec& spec) noexcept {
    const float gm = detail::opAmpInputTransconductance(spec);
    bool ok = true;
    ok &= engine.setCapacitorSpec(handles.dominantCapacitance,
                                  detail::opAmpDominantCapacitance(spec));
    ok &= engine.setVccsTransconductance(handles.differentialGm, gm);
    ok &= engine.setVccsTransconductance(handles.offsetGm, gm);
    ok &= engine.setVoltageSource(handles.offsetVoltage, spec.inputOffsetVoltage);
    ok &= engine.setResistorSpec(handles.outputResistance, detail::opAmpOutputResistance(spec));
    return ok;
}

} // namespace guitardsp::circuit
