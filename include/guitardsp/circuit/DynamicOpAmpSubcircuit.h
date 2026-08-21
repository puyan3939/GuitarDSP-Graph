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
// - soft output rail clamps referenced to explicit supply nodes
//
// Slew-rate and output-current limiting remain explicit future upgrades because
// they require a current-limited dynamic source rather than a linear VCCS.
struct DynamicOpAmpSubcircuit {
    Node dominantPole = ground;
    Node outputDrive = ground;
    Node offsetNode = ground;
    Node positiveClamp = ground;
    Node negativeClamp = ground;

    ResistorHandle dominantResistance{};
    CapacitorHandle dominantCapacitance{};
    ControlledSourceHandle differentialGm{};
    ControlledSourceHandle offsetGm{};
    SourceHandle offsetVoltage{};
    ControlledSourceHandle outputFollower{};
    ResistorHandle outputResistance{};
    SourceHandle positiveClampOffset{};
    SourceHandle negativeClampOffset{};
    DiodeHandle positiveClampDiode{};
    DiodeHandle negativeClampDiode{};
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

inline hq::DiodeSpec opAmpRailClampDiode() noexcept {
    hq::DiodeSpec diode{};
    diode.name = "OpAmp soft rail clamp";
    diode.technology = hq::DiodeTechnology::silicon;
    diode.nominalForwardVoltage = 0.01f;
    diode.saturationCurrent = 1.0e-3f;
    diode.emissionCoefficient = 1.0f;
    diode.thermalVoltage = 0.02585f;
    diode.seriesResistanceOhms = 0.05f;
    diode.junctionCapacitanceFarads = 0.0f;
    diode.reverseVoltageRating = 1000.0f;
    diode.currentRatingAmps = 10.0f;
    return diode;
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
    DynamicOpAmpSubcircuit handles{};
    handles.dominantPole = engine.addNode();
    handles.outputDrive = engine.addNode();
    handles.offsetNode = engine.addNode();
    handles.positiveClamp = engine.addNode();
    handles.negativeClamp = engine.addNode();

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

    handles.positiveClampOffset = engine.addVoltageSource(handles.positiveClamp, positiveRail,
        -std::max(0.0f, spec.positiveRailHeadroomVolts));
    handles.negativeClampOffset = engine.addVoltageSource(handles.negativeClamp, negativeRail,
        std::max(0.0f, spec.negativeRailHeadroomVolts));

    // Clamp the dominant-pole state rather than the low-impedance follower output.
    // This prevents the ideal VCVS buffer from demanding an unbounded internal
    // voltage after the external output has already hit a supply rail.
    const auto clampDiode = detail::opAmpRailClampDiode();
    handles.positiveClampDiode = engine.addDiode(handles.dominantPole,
                                                  handles.positiveClamp, clampDiode);
    handles.negativeClampDiode = engine.addDiode(handles.negativeClamp,
                                                  handles.dominantPole, clampDiode);
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
    ok &= engine.setVoltageSource(handles.positiveClampOffset,
                                  -std::max(0.0f, spec.positiveRailHeadroomVolts));
    ok &= engine.setVoltageSource(handles.negativeClampOffset,
                                  std::max(0.0f, spec.negativeRailHeadroomVolts));
    return ok;
}

} // namespace guitardsp::circuit
