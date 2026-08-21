#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// Circuit-level audio op-amp macro assembled from ordinary MNA primitives.
//
// Implemented:
// - finite DC open-loop gain
// - dominant-pole / gain-bandwidth behavior
// - input offset and input bias current
// - rail headroom limiting through polynomial MOSFET shunts
// - slew-rate limiting through a current-limited JFET / capacitor stage
// - finite output resistance and smooth bidirectional output-current limiting
//
// The large-signal stage deliberately keeps high-gain, slew and load-current
// responsibilities separated. The dominant-pole command is buffered before the
// slew limiter, and the slew node is buffered again before the load-current path.
// Both the internal slew state and the externally visible output are constrained
// to supply-relative headroom. The second rail clamp matters when the current-
// limiter is feeding a high-impedance/nonlinear feedback network: a real output
// pin cannot drift beyond its supply rails merely because load current is small.
// Rail shunts use the engine's bounded square-law MOSFET stamp rather than an
// exponential Shockley clamp so gross internal errors remain recoverable by Newton.
struct DynamicOpAmpSubcircuit {
    Node dominantPole = ground;
    Node slewCommand = ground;
    Node outputDrive = ground;
    Node positiveClamp = ground;
    Node negativeClamp = ground;
    Node outputBufferDrive = ground;
    Node outputCurrentNode = ground;
    Node offsetNode = ground;

    ResistorHandle dominantResistance{};
    CapacitorHandle dominantCapacitance{};
    ControlledSourceHandle differentialGm{};
    ControlledSourceHandle offsetGm{};
    SourceHandle offsetVoltage{};
    ControlledSourceHandle slewCommandFollower{};

    JfetHandle slewLimiter{};
    CapacitorHandle slewCapacitance{};
    SourceHandle positiveClampOffset{};
    SourceHandle negativeClampOffset{};
    MosfetHandle positiveRailShunt{};
    MosfetHandle negativeRailShunt{};

    ControlledSourceHandle outputFollower{};
    ResistorHandle outputResistance{};
    JfetHandle outputCurrentLimiter{};
    MosfetHandle outputPositiveRailShunt{};
    MosfetHandle outputNegativeRailShunt{};
    ResistorHandle outputLeakResistance{};
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

inline hq::CapacitorSpec genericCircuitCapacitor(float farads) noexcept {
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

inline hq::CapacitorSpec opAmpDominantCapacitance(const hq::OpAmpSpec& spec) noexcept {
    constexpr float pi = 3.14159265358979323846f;
    const float a0 = opAmpOpenLoopGain(spec);
    const float gbw = std::max(1.0f, spec.gainBandwidthHz);
    const float pole = std::max(0.01f, gbw / a0);
    const float r = opAmpDominantResistance().resistanceOhms;
    return genericCircuitCapacitor(1.0f / (2.0f * pi * r * pole));
}

inline float opAmpInputTransconductance(const hq::OpAmpSpec& spec) noexcept {
    return opAmpOpenLoopGain(spec) / opAmpDominantResistance().resistanceOhms;
}

inline constexpr float opAmpSlewCapacitanceFarads() noexcept {
    return 100.0e-12f;
}

inline hq::JFETSpec currentLimiterJfet(float currentLimitAmps,
                                       const char* name) noexcept {
    hq::JFETSpec j{};
    j.name = name;
    j.polarity = hq::TransistorPolarity::nChannel;
    j.idssAmps = std::max(1.0e-8f, currentLimitAmps);
    j.pinchOffVoltage = -1.0f;
    j.lambda = 0.0f;
    j.gateSourceCapacitanceFarads = 0.0f;
    j.maxDrainSourceVoltage = 1000.0f;
    return j;
}

inline hq::JFETSpec opAmpSlewLimiter(const hq::OpAmpSpec& spec) noexcept {
    const float current = std::max(1.0f, spec.slewRateVoltsPerSecond) *
                          opAmpSlewCapacitanceFarads();
    return currentLimiterJfet(current, "OpAmp slew current limiter");
}

inline hq::JFETSpec opAmpOutputCurrentLimiter(const hq::OpAmpSpec& spec) noexcept {
    return currentLimiterJfet(std::max(1.0e-5f, spec.outputCurrentLimitAmps),
                              "OpAmp output current limiter");
}

inline constexpr float opAmpRailShuntThresholdVolts() noexcept { return 0.03f; }

inline hq::MOSFETSpec opAmpRailShunt(hq::TransistorPolarity polarity) noexcept {
    hq::MOSFETSpec device{};
    device.name = polarity == hq::TransistorPolarity::pChannel
        ? "OpAmp negative-rail shunt" : "OpAmp positive-rail shunt";
    device.polarity = polarity;
    device.thresholdVoltage = opAmpRailShuntThresholdVolts();
    // A moderate square-law coefficient is intentionally used here. A very stiff
    // artificial shunt approximates an ideal clamp but produces enormous Newton
    // curvature after a bad iterate. 20 mA/V^2 still clamps decisively around audio
    // rails while allowing the global circuit Newton solve to recover smoothly.
    device.transconductance = 0.02f;
    device.lambda = 0.0f;
    device.bodyDiodeForwardVoltage = 1000.0f;
    device.gateCapacitanceFarads = 0.0f;
    device.maxDrainSourceVoltage = 1000.0f;
    return device;
}

inline float positiveClampOffset(const hq::OpAmpSpec& spec) noexcept {
    return std::max(0.0f, spec.positiveRailHeadroomVolts) +
           opAmpRailShuntThresholdVolts();
}

inline float negativeClampOffset(const hq::OpAmpSpec& spec) noexcept {
    return std::max(0.0f, spec.negativeRailHeadroomVolts) +
           opAmpRailShuntThresholdVolts();
}

inline hq::ResistorSpec opAmpOutputResistance(const hq::OpAmpSpec& spec) noexcept {
    hq::ResistorSpec r{};
    r.resistanceOhms = std::max(0.01f, spec.outputResistanceOhms);
    r.tolerancePercent = 0.0f;
    r.powerRatingWatts = 100.0f;
    return r;
}

inline hq::ResistorSpec opAmpOutputLeak() noexcept {
    hq::ResistorSpec r{};
    r.resistanceOhms = 10.0e6f;
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
    DynamicOpAmpSubcircuit handles{};
    handles.dominantPole = engine.addNode();
    handles.slewCommand = engine.addNode();
    handles.outputDrive = engine.addNode();
    handles.positiveClamp = engine.addNode();
    handles.negativeClamp = engine.addNode();
    handles.outputBufferDrive = engine.addNode();
    handles.outputCurrentNode = engine.addNode();
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

    handles.slewCommandFollower = engine.addVcvs(handles.slewCommand, reference,
                                                  handles.dominantPole, reference, 1.0f);
    handles.slewLimiter = engine.addJfet(handles.slewCommand, handles.outputDrive,
                                          handles.outputDrive,
                                          detail::opAmpSlewLimiter(spec));
    handles.slewCapacitance = engine.addCapacitor(handles.outputDrive, reference,
        detail::genericCircuitCapacitor(detail::opAmpSlewCapacitanceFarads()));

    handles.positiveClampOffset = engine.addVoltageSource(positiveRail,
        handles.positiveClamp, detail::positiveClampOffset(spec));
    handles.negativeClampOffset = engine.addVoltageSource(handles.negativeClamp,
        negativeRail, detail::negativeClampOffset(spec));
    handles.positiveRailShunt = engine.addMosfet(handles.outputDrive,
        handles.outputDrive, handles.positiveClamp,
        detail::opAmpRailShunt(hq::TransistorPolarity::nChannel));
    handles.negativeRailShunt = engine.addMosfet(handles.outputDrive,
        handles.outputDrive, handles.negativeClamp,
        detail::opAmpRailShunt(hq::TransistorPolarity::pChannel));

    handles.outputFollower = engine.addVcvs(handles.outputBufferDrive, reference,
                                             handles.outputDrive, reference, 1.0f);
    handles.outputResistance = engine.addResistor(handles.outputBufferDrive,
                                                   handles.outputCurrentNode,
                                                   detail::opAmpOutputResistance(spec));
    handles.outputCurrentLimiter = engine.addJfet(handles.outputCurrentNode, output,
                                                   output,
                                                   detail::opAmpOutputCurrentLimiter(spec));

    handles.outputPositiveRailShunt = engine.addMosfet(output, output,
        handles.positiveClamp, detail::opAmpRailShunt(hq::TransistorPolarity::nChannel));
    handles.outputNegativeRailShunt = engine.addMosfet(output, output,
        handles.negativeClamp, detail::opAmpRailShunt(hq::TransistorPolarity::pChannel));
    handles.outputLeakResistance = engine.addResistor(output, reference,
                                                       detail::opAmpOutputLeak());
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
    ok &= engine.setJfetSpec(handles.slewLimiter, detail::opAmpSlewLimiter(spec));
    ok &= engine.setVoltageSource(handles.positiveClampOffset,
                                  detail::positiveClampOffset(spec));
    ok &= engine.setVoltageSource(handles.negativeClampOffset,
                                  detail::negativeClampOffset(spec));
    ok &= engine.setMosfetSpec(handles.positiveRailShunt,
        detail::opAmpRailShunt(hq::TransistorPolarity::nChannel));
    ok &= engine.setMosfetSpec(handles.negativeRailShunt,
        detail::opAmpRailShunt(hq::TransistorPolarity::pChannel));
    ok &= engine.setResistorSpec(handles.outputResistance,
                                  detail::opAmpOutputResistance(spec));
    ok &= engine.setJfetSpec(handles.outputCurrentLimiter,
                              detail::opAmpOutputCurrentLimiter(spec));
    ok &= engine.setMosfetSpec(handles.outputPositiveRailShunt,
        detail::opAmpRailShunt(hq::TransistorPolarity::nChannel));
    ok &= engine.setMosfetSpec(handles.outputNegativeRailShunt,
        detail::opAmpRailShunt(hq::TransistorPolarity::pChannel));
    return ok;
}

} // namespace guitardsp::circuit
