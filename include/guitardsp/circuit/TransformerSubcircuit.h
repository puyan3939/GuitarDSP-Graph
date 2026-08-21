#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::circuit {

// A transformer is deliberately expressed as a reusable subcircuit instead of a
// new monolithic solver device. The model combines primary/secondary leakage and
// winding resistance, a magnetizing inductance, an ideal ratio implemented by
// VCVS+CCCS, a zero-volt secondary current sensor, and an inter-winding capacitor.
//
// The magnetizing branch can be updated sample-by-sample with a smooth
// current-dependent inductance collapse. This is an engineering core-saturation
// approximation; hysteresis/remanence and a measured B-H loop are still future work.
struct TransformerSubcircuit {
    Node primaryCore = ground;
    Node secondaryDrive = ground;
    Node secondaryChain = ground;

    InductorHandle primaryLeakage{};
    InductorHandle magnetizing{};
    InductorHandle secondaryLeakage{};
    CapacitorHandle interwindingCapacitance{};
    SourceHandle secondaryCurrentSense{};
    ControlledSourceHandle voltageRatio{};
    ControlledSourceHandle currentRatio{};
};

namespace detail {
inline float safeTurnsRatio(float ratio) noexcept {
    const float magnitude = std::max(1.0e-3f, std::abs(ratio));
    return ratio < 0.0f ? -magnitude : magnitude;
}

inline hq::InductorSpec primaryLeakageSpec(const hq::TransformerSpec& transformer) noexcept {
    hq::InductorSpec spec{};
    spec.inductanceHenries = std::max(1.0e-12f, 0.5f * transformer.leakageInductanceHenries);
    spec.seriesResistanceOhms = std::max(0.0f, transformer.primaryResistanceOhms);
    spec.parasiticCapacitanceFarads = 0.0f;
    spec.currentRatingAmps = 10.0f;
    spec.saturationCurrentAmps = 10.0f;
    return spec;
}

inline hq::InductorSpec magnetizingSpec(const hq::TransformerSpec& transformer,
                                        float inductanceOverride = -1.0f) noexcept {
    hq::InductorSpec spec{};
    spec.inductanceHenries = inductanceOverride > 0.0f
        ? inductanceOverride
        : std::max(1.0e-9f, transformer.primaryInductanceHenries);
    spec.seriesResistanceOhms = 0.0f;
    spec.parasiticCapacitanceFarads = 0.0f;
    spec.currentRatingAmps = 10.0f;
    spec.saturationCurrentAmps = std::max(1.0e-6f, transformer.magnetizingSaturationCurrentAmps);
    return spec;
}

inline hq::InductorSpec secondaryLeakageSpec(const hq::TransformerSpec& transformer) noexcept {
    const float ratio = safeTurnsRatio(transformer.turnsRatio);
    const float ratioSquared = ratio * ratio;
    hq::InductorSpec spec{};
    spec.inductanceHenries = std::max(1.0e-12f,
        0.5f * transformer.leakageInductanceHenries / ratioSquared);
    spec.seriesResistanceOhms = std::max(0.0f, transformer.secondaryResistanceOhms);
    spec.parasiticCapacitanceFarads = 0.0f;
    spec.currentRatingAmps = 10.0f;
    spec.saturationCurrentAmps = 10.0f;
    return spec;
}

inline hq::CapacitorSpec interwindingSpec(const hq::TransformerSpec& transformer) noexcept {
    hq::CapacitorSpec spec{};
    spec.capacitanceFarads = std::max(0.0f, transformer.interwindingCapacitanceFarads);
    spec.esrOhms = 0.0f;
    spec.leakageResistanceOhms = 1.0e12f;
    spec.voltageRatingVolts = 1000.0f;
    spec.technology = hq::CapacitorTechnology::generic;
    return spec;
}

inline float saturatedMagnetizingInductance(const hq::TransformerSpec& transformer,
                                            float magnetizingCurrentAmps) noexcept {
    const float nominal = std::max(1.0e-9f, transformer.primaryInductanceHenries);
    const float fluxScale = std::max(0.05f, transformer.saturationFluxNormalized);
    const float kneeCurrent = std::max(1.0e-6f,
        transformer.magnetizingSaturationCurrentAmps * fluxScale);
    const float exponent = std::max(1.0f, transformer.coreSaturationExponent);
    const float minimumRatio = std::clamp(transformer.minimumMagnetizingInductanceRatio,
                                          0.001f, 1.0f);
    const float normalized = std::abs(magnetizingCurrentAmps) / kneeCurrent;
    const float unsaturatedWeight = 1.0f / (1.0f + std::pow(normalized, exponent));
    const float ratio = minimumRatio + (1.0f - minimumRatio) * unsaturatedWeight;
    return nominal * ratio;
}
} // namespace detail

inline TransformerSubcircuit addTransformerSubcircuit(MnaCircuitEngine& engine,
                                                        Node primaryPositive,
                                                        Node primaryNegative,
                                                        Node secondaryPositive,
                                                        Node secondaryNegative,
                                                        const hq::TransformerSpec& spec) {
    TransformerSubcircuit handles{};
    handles.primaryCore = engine.addNode();
    handles.secondaryDrive = engine.addNode();
    handles.secondaryChain = engine.addNode();

    handles.primaryLeakage = engine.addInductor(primaryPositive, handles.primaryCore,
                                                 detail::primaryLeakageSpec(spec));
    handles.magnetizing = engine.addInductor(handles.primaryCore, primaryNegative,
                                              detail::magnetizingSpec(spec));

    const float ratio = detail::safeTurnsRatio(spec.turnsRatio);
    handles.voltageRatio = engine.addVcvs(handles.secondaryDrive, secondaryNegative,
                                           handles.primaryCore, primaryNegative,
                                           1.0f / ratio);
    handles.secondaryCurrentSense = engine.addVoltageSource(handles.secondaryDrive,
                                                             handles.secondaryChain,
                                                             0.0f);
    handles.currentRatio = engine.addCccs(handles.primaryCore, primaryNegative,
                                           handles.secondaryCurrentSense,
                                           1.0f / ratio);
    handles.secondaryLeakage = engine.addInductor(handles.secondaryChain, secondaryPositive,
                                                   detail::secondaryLeakageSpec(spec));
    handles.interwindingCapacitance = engine.addCapacitor(handles.primaryCore,
                                                           handles.secondaryDrive,
                                                           detail::interwindingSpec(spec));
    return handles;
}

inline bool updateTransformerSubcircuit(MnaCircuitEngine& engine,
                                        const TransformerSubcircuit& handles,
                                        const hq::TransformerSpec& spec) noexcept {
    const float ratio = detail::safeTurnsRatio(spec.turnsRatio);
    bool ok = true;
    ok &= engine.setInductorSpec(handles.primaryLeakage, detail::primaryLeakageSpec(spec));
    ok &= engine.setInductorSpec(handles.magnetizing, detail::magnetizingSpec(spec));
    ok &= engine.setVcvsGain(handles.voltageRatio, 1.0f / ratio);
    ok &= engine.setCccsGain(handles.currentRatio, 1.0f / ratio);
    ok &= engine.setInductorSpec(handles.secondaryLeakage, detail::secondaryLeakageSpec(spec));
    ok &= engine.setCapacitorSpec(handles.interwindingCapacitance, detail::interwindingSpec(spec));
    return ok;
}

// Call after processSample() to prepare the magnetizing inductance for the next
// sample. This keeps the main solver generic while still allowing a nonlinear
// transformer core. The one-sample state update is deterministic and allocation-free.
inline float updateTransformerCoreSaturation(MnaCircuitEngine& engine,
                                             const TransformerSubcircuit& handles,
                                             const hq::TransformerSpec& spec) noexcept {
    const float current = engine.inductorCurrent(static_cast<std::size_t>(handles.magnetizing));
    const float inductance = detail::saturatedMagnetizingInductance(spec, current);
    engine.setInductorSpec(handles.magnetizing, detail::magnetizingSpec(spec, inductance));
    return inductance;
}

inline float transformerSecondaryCurrent(const MnaCircuitEngine& engine,
                                         const TransformerSubcircuit& handles) noexcept {
    return engine.currentThroughVoltageSource(handles.secondaryCurrentSense);
}

} // namespace guitardsp::circuit
