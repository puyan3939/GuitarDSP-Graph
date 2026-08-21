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
// - rail headroom limiting through bounded clamp nodes
// - slew-rate limiting through a current-limited JFET / capacitor stage
// - complementary BJT output stage with finite output resistance/current limit
//
// The large-signal stage intentionally avoids an ideal hard-clamped dependent
// voltage source. The earlier diode-on-high-gain-node experiment could diverge
// under severe overdrive. Here the dominant-pole command is buffered, then passed
// through a bounded-current element before rail clamps and the output pair. That
// keeps the internal command finite and gives the Newton solver a smooth path.
struct DynamicOpAmpSubcircuit {
    Node dominantPole = ground;
    Node slewCommand = ground;
    Node outputDrive = ground;
    Node positiveClamp = ground;
    Node negativeClamp = ground;
    Node sourceBase = ground;
    Node sinkBase = ground;
    Node sourceEmitter = ground;
    Node sinkEmitter = ground;
    Node offsetNode = ground;

    ResistorHandle dominantResistance{};
    CapacitorHandle dominantCapacitance{};
    ControlledSourceHandle differentialGm{};
    ControlledSourceHandle offsetGm{};
    SourceHandle offsetVoltage{};
    ControlledSourceHandle outputFollower{};

    JfetHandle slewLimiter{};
    CapacitorHandle slewCapacitance{};
    SourceHandle positiveClampOffset{};
    SourceHandle negativeClampOffset{};
    DiodeHandle positiveClampDiode{};
    DiodeHandle negativeClampDiode{};

    SourceHandle sourceBias{};
    SourceHandle sinkBias{};
    BjtHandle sourceTransistor{};
    BjtHandle sinkTransistor{};
    ResistorHandle sourceOutputResistance{};
    ResistorHandle sinkOutputResistance{};
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
    // A small fixed integration capacitor lets Idss = C * slew-rate map directly
    // to the requested large-signal dV/dt while keeping currents pedal-scale.
    return 100.0e-12f;
}

inline hq::JFETSpec opAmpSlewLimiter(const hq::OpAmpSpec& spec) noexcept {
    hq::JFETSpec j{};
    j.name = "OpAmp slew current limiter";
    j.polarity = hq::TransistorPolarity::nChannel;
    j.idssAmps = std::max(1.0e-8f,
        std::max(1.0f, spec.slewRateVoltsPerSecond) * opAmpSlewCapacitanceFarads());
    j.pinchOffVoltage = -1.0f;
    j.lambda = 0.0f;
    j.gateSourceCapacitanceFarads = 0.0f;
    j.maxDrainSourceVoltage = 1000.0f;
    return j;
}

inline hq::DiodeSpec opAmpRailClampDiode() noexcept {
    hq::DiodeSpec diode{};
    diode.name = "OpAmp smooth rail clamp";
    diode.technology = hq::DiodeTechnology::silicon;
    diode.nominalForwardVoltage = 0.025f;
    diode.saturationCurrent = 1.0e-3f;
    diode.emissionCoefficient = 1.0f;
    diode.thermalVoltage = 0.02585f;
    diode.seriesResistanceOhms = 0.5f;
    diode.junctionCapacitanceFarads = 0.0f;
    diode.reverseVoltageRating = 1000.0f;
    diode.currentRatingAmps = 1.0f;
    return diode;
}

inline constexpr float opAmpClampForwardEstimate() noexcept { return 0.025f; }
inline constexpr float opAmpOutputBiasVoltage() noexcept { return 0.62f; }

inline float positiveClampOffset(const hq::OpAmpSpec& spec) noexcept {
    return std::max(0.0f, spec.positiveRailHeadroomVolts) + opAmpClampForwardEstimate();
}

inline float negativeClampOffset(const hq::OpAmpSpec& spec) noexcept {
    return std::max(0.0f, spec.negativeRailHeadroomVolts) + opAmpClampForwardEstimate();
}

inline hq::BJTSpec opAmpOutputTransistor(const hq::OpAmpSpec& spec,
                                         hq::TransistorPolarity polarity) noexcept {
    hq::BJTSpec b{};
    b.name = polarity == hq::TransistorPolarity::pnp
        ? "OpAmp PNP output device" : "OpAmp NPN output device";
    b.polarity = polarity;
    b.beta = 220.0f;
    b.nominalVbe = opAmpOutputBiasVoltage();
    b.saturationVoltage = 0.10f;
    b.thermalVoltage = 0.02585f;
    b.maxCollectorVoltage = 1000.0f;
    b.maxCollectorCurrentAmps = std::max(1.0e-5f, spec.outputCurrentLimitAmps);
    b.inputCapacitanceFarads = 0.0f;
    return b;
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
    handles.sourceBase = engine.addNode();
    handles.sinkBase = engine.addNode();
    handles.sourceEmitter = engine.addNode();
    handles.sinkEmitter = engine.addNode();
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

    // Buffer the dominant-pole command so the slew limiter cannot load the high
    // impedance compensation node. A JFET with gate tied to source behaves as a
    // smooth bidirectional current limiter in this engineering device model.
    handles.outputFollower = engine.addVcvs(handles.slewCommand, reference,
                                             handles.dominantPole, reference, 1.0f);
    handles.slewLimiter = engine.addJfet(handles.slewCommand, handles.outputDrive,
                                          handles.outputDrive,
                                          detail::opAmpSlewLimiter(spec));
    handles.slewCapacitance = engine.addCapacitor(handles.outputDrive, reference,
        detail::genericCircuitCapacitor(detail::opAmpSlewCapacitanceFarads()));

    // Clamp nodes follow the supply rails with the requested headroom. The clamp
    // diodes are intentionally low-Vf smooth junctions; the offset compensates
    // their nominal drop so the output-drive node approaches the specified limit.
    handles.positiveClampOffset = engine.addVoltageSource(positiveRail,
        handles.positiveClamp, detail::positiveClampOffset(spec));
    handles.negativeClampOffset = engine.addVoltageSource(handles.negativeClamp,
        negativeRail, detail::negativeClampOffset(spec));
    handles.positiveClampDiode = engine.addDiode(handles.outputDrive,
        handles.positiveClamp, detail::opAmpRailClampDiode());
    handles.negativeClampDiode = engine.addDiode(handles.negativeClamp,
        handles.outputDrive, detail::opAmpRailClampDiode());

    // Class-AB-like complementary emitter followers. Bias sources compensate the
    // model Vbe so the external feedback loop sees an output centered on the slew
    // node. The BJT collector-current cap supplies the explicit output-current
    // limit; emitter resistors provide the requested finite output resistance.
    handles.sourceBias = engine.addVoltageSource(handles.sourceBase, handles.outputDrive,
                                                  detail::opAmpOutputBiasVoltage());
    handles.sinkBias = engine.addVoltageSource(handles.outputDrive, handles.sinkBase,
                                                detail::opAmpOutputBiasVoltage());
    handles.sourceTransistor = engine.addBjt(positiveRail, handles.sourceBase,
        handles.sourceEmitter,
        detail::opAmpOutputTransistor(spec, hq::TransistorPolarity::npn));
    handles.sinkTransistor = engine.addBjt(negativeRail, handles.sinkBase,
        handles.sinkEmitter,
        detail::opAmpOutputTransistor(spec, hq::TransistorPolarity::pnp));
    handles.sourceOutputResistance = engine.addResistor(handles.sourceEmitter, output,
                                                         detail::opAmpOutputResistance(spec));
    handles.sinkOutputResistance = engine.addResistor(handles.sinkEmitter, output,
                                                       detail::opAmpOutputResistance(spec));
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
    ok &= engine.setBjtSpec(handles.sourceTransistor,
        detail::opAmpOutputTransistor(spec, hq::TransistorPolarity::npn));
    ok &= engine.setBjtSpec(handles.sinkTransistor,
        detail::opAmpOutputTransistor(spec, hq::TransistorPolarity::pnp));
    ok &= engine.setResistorSpec(handles.sourceOutputResistance,
                                  detail::opAmpOutputResistance(spec));
    ok &= engine.setResistorSpec(handles.sinkOutputResistance,
                                  detail::opAmpOutputResistance(spec));
    return ok;
}

} // namespace guitardsp::circuit
