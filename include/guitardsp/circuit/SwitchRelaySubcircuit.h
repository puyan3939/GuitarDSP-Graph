#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace guitardsp::circuit {

struct SwitchSubcircuit {
    hq::SwitchContactForm form = hq::SwitchContactForm::spst;
    ResistorHandle pole1A{};
    ResistorHandle pole1B{};
    ResistorHandle pole2A{};
    ResistorHandle pole2B{};
};

namespace detail {
inline hq::ResistorSpec contactResistance(float ohms) noexcept {
    hq::ResistorSpec r{};
    r.resistanceOhms = std::max(1.0e-6f, ohms);
    r.tolerancePercent = 0.0f;
    r.powerRatingWatts = 100.0f;
    r.temperatureCoefficientPpm = 0.0f;
    r.excessNoiseFactor = 0.0f;
    return r;
}

inline float closedContactResistance(const hq::SwitchSpec& spec) noexcept {
    return std::max(1.0e-6f, spec.closedResistanceOhms);
}

inline float openContactResistance(const hq::SwitchSpec& spec) noexcept {
    return std::max(closedContactResistance(spec) * 10.0f, spec.openResistanceOhms);
}
} // namespace detail

// For SPST only common1/throwA1 are used. For SPDT use common1 + throwA1/throwB1.
// For DPDT the second pole uses common2 + throwA2/throwB2.
inline SwitchSubcircuit addSwitchSubcircuit(MnaCircuitEngine& engine,
                                             Node common1,
                                             Node throwA1,
                                             Node throwB1,
                                             Node common2,
                                             Node throwA2,
                                             Node throwB2,
                                             const hq::SwitchSpec& spec,
                                             bool thrown = false) {
    SwitchSubcircuit handles{};
    handles.form = spec.form;
    const float closed = detail::closedContactResistance(spec);
    const float open = detail::openContactResistance(spec);

    if (spec.form == hq::SwitchContactForm::spst) {
        handles.pole1A = engine.addResistor(common1, throwA1,
            detail::contactResistance(thrown ? closed : open));
        return handles;
    }

    handles.pole1A = engine.addResistor(common1, throwA1,
        detail::contactResistance(thrown ? open : closed));
    handles.pole1B = engine.addResistor(common1, throwB1,
        detail::contactResistance(thrown ? closed : open));

    if (spec.form == hq::SwitchContactForm::dpdt) {
        handles.pole2A = engine.addResistor(common2, throwA2,
            detail::contactResistance(thrown ? open : closed));
        handles.pole2B = engine.addResistor(common2, throwB2,
            detail::contactResistance(thrown ? closed : open));
    }
    return handles;
}

inline bool setSwitchState(MnaCircuitEngine& engine,
                           const SwitchSubcircuit& handles,
                           const hq::SwitchSpec& spec,
                           bool thrown) noexcept {
    const float closed = detail::closedContactResistance(spec);
    const float open = detail::openContactResistance(spec);
    bool ok = true;

    if (handles.form == hq::SwitchContactForm::spst)
        return engine.setResistance(handles.pole1A, thrown ? closed : open);

    ok &= engine.setResistance(handles.pole1A, thrown ? open : closed);
    ok &= engine.setResistance(handles.pole1B, thrown ? closed : open);
    if (handles.form == hq::SwitchContactForm::dpdt) {
        ok &= engine.setResistance(handles.pole2A, thrown ? open : closed);
        ok &= engine.setResistance(handles.pole2B, thrown ? closed : open);
    }
    return ok;
}

struct RelaySubcircuit {
    Node coilPositive = ground;
    Node coilNegative = ground;
    Node coilSenseNode = ground;
    InductorHandle coil{};
    SourceHandle coilCurrentSense{};
    SwitchSubcircuit contacts{};
};

struct RelayRuntimeState {
    bool energized = false;
    bool requestedState = false;
    bool contactState = false;
    std::size_t delaySamplesRemaining = 0;
    std::size_t bounceSamplesRemaining = 0;
    std::size_t bounceSegmentSamples = 1;
    std::size_t bounceSegmentCounter = 0;
    std::uint8_t bounceTransitionsRemaining = 0;
};

namespace detail {
inline hq::InductorSpec relayCoilSpec(const hq::RelaySpec& spec) noexcept {
    hq::InductorSpec coil{};
    coil.inductanceHenries = std::max(1.0e-9f, spec.coilInductanceHenries);
    coil.seriesResistanceOhms = std::max(1.0e-3f, spec.coilResistanceOhms);
    coil.currentRatingAmps = std::max(0.01f, 2.0f * spec.coilRatedVoltage /
                                              std::max(1.0f, spec.coilResistanceOhms));
    coil.parasiticCapacitanceFarads = 0.0f;
    coil.saturationCurrentAmps = 10.0f;
    return coil;
}

inline hq::SwitchSpec relayContactSpec(const hq::RelaySpec& spec) noexcept {
    hq::SwitchSpec contact{};
    contact.name = spec.name;
    switch (spec.form) {
        case hq::RelayContactForm::spstNormallyOpen:
            contact.form = hq::SwitchContactForm::spst;
            break;
        case hq::RelayContactForm::spdt:
            contact.form = hq::SwitchContactForm::spdt;
            break;
        case hq::RelayContactForm::dpdt:
        default:
            contact.form = hq::SwitchContactForm::dpdt;
            break;
    }
    contact.closedResistanceOhms = spec.closedContactResistanceOhms;
    contact.openResistanceOhms = spec.openContactResistanceOhms;
    contact.bounceMilliseconds = spec.bounceMilliseconds;
    contact.bounceTransitions = spec.bounceTransitions;
    return contact;
}

inline std::size_t millisecondsToSamples(float milliseconds, double sampleRate) noexcept {
    if (milliseconds <= 0.0f) return 0U;
    const double samples = static_cast<double>(milliseconds) * 0.001 * std::max(1.0, sampleRate);
    return static_cast<std::size_t>(std::max(1.0, std::round(samples)));
}
} // namespace detail

// Contact semantics:
// - SPST NO: common1/throwA1 close when energized.
// - SPDT: throwA is NC, throwB is NO.
// - DPDT: both A throws are NC and both B throws are NO.
inline RelaySubcircuit addRelaySubcircuit(MnaCircuitEngine& engine,
                                           Node coilPositive,
                                           Node coilNegative,
                                           Node common1,
                                           Node throwA1,
                                           Node throwB1,
                                           Node common2,
                                           Node throwA2,
                                           Node throwB2,
                                           const hq::RelaySpec& spec) {
    RelaySubcircuit relay{};
    relay.coilPositive = coilPositive;
    relay.coilNegative = coilNegative;
    relay.coilSenseNode = engine.addNode();
    relay.coil = engine.addInductor(coilPositive, relay.coilSenseNode,
                                    detail::relayCoilSpec(spec));
    relay.coilCurrentSense = engine.addVoltageSource(relay.coilSenseNode, coilNegative, 0.0f);
    relay.contacts = addSwitchSubcircuit(engine, common1, throwA1, throwB1,
                                         common2, throwA2, throwB2,
                                         detail::relayContactSpec(spec), false);
    return relay;
}

inline float relayCoilCurrent(const MnaCircuitEngine& engine,
                              const RelaySubcircuit& relay) noexcept {
    return engine.currentThroughVoltageSource(relay.coilCurrentSense);
}

inline bool applyRelayContactState(MnaCircuitEngine& engine,
                                   const RelaySubcircuit& relay,
                                   const hq::RelaySpec& spec,
                                   bool energized) noexcept {
    return setSwitchState(engine, relay.contacts, detail::relayContactSpec(spec), energized);
}

inline bool updateRelayRuntime(MnaCircuitEngine& engine,
                               const RelaySubcircuit& relay,
                               RelayRuntimeState& state,
                               const hq::RelaySpec& spec,
                               double sampleRate) noexcept {
    const float coilR = std::max(1.0e-3f, spec.coilResistanceOhms);
    const float pickupCurrent = std::max(0.0f, spec.pickupVoltage) / coilR;
    const float dropoutCurrent = std::max(0.0f, spec.dropoutVoltage) / coilR;
    const float currentMagnitude = std::abs(relayCoilCurrent(engine, relay));

    const bool requested = state.energized
        ? currentMagnitude > dropoutCurrent
        : currentMagnitude >= pickupCurrent;

    if (requested != state.requestedState) {
        state.requestedState = requested;
        state.delaySamplesRemaining = detail::millisecondsToSamples(
            requested ? spec.operateMilliseconds : spec.releaseMilliseconds, sampleRate);
    }

    if (state.energized != state.requestedState) {
        if (state.delaySamplesRemaining > 0U) {
            --state.delaySamplesRemaining;
        } else {
            state.energized = state.requestedState;
            state.contactState = state.energized;
            state.bounceSamplesRemaining = detail::millisecondsToSamples(spec.bounceMilliseconds,
                                                                          sampleRate);
            state.bounceTransitionsRemaining = spec.bounceTransitions;
            const std::size_t segments = static_cast<std::size_t>(spec.bounceTransitions) + 1U;
            state.bounceSegmentSamples = std::max<std::size_t>(1U,
                state.bounceSamplesRemaining / std::max<std::size_t>(1U, segments));
            state.bounceSegmentCounter = state.bounceSegmentSamples;
            applyRelayContactState(engine, relay, spec, state.contactState);
        }
    }

    if (state.bounceSamplesRemaining > 0U) {
        --state.bounceSamplesRemaining;
        if (state.bounceTransitionsRemaining > 0U) {
            if (state.bounceSegmentCounter > 0U) --state.bounceSegmentCounter;
            if (state.bounceSegmentCounter == 0U) {
                state.contactState = !state.contactState;
                --state.bounceTransitionsRemaining;
                state.bounceSegmentCounter = state.bounceSegmentSamples;
                applyRelayContactState(engine, relay, spec, state.contactState);
            }
        }
        if (state.bounceSamplesRemaining == 0U) {
            state.contactState = state.energized;
            state.bounceTransitionsRemaining = 0U;
            applyRelayContactState(engine, relay, spec, state.contactState);
        }
    }

    return state.energized;
}

inline bool updateRelaySpec(MnaCircuitEngine& engine,
                            const RelaySubcircuit& relay,
                            RelayRuntimeState& state,
                            const hq::RelaySpec& spec) noexcept {
    bool ok = engine.setInductorSpec(relay.coil, detail::relayCoilSpec(spec));
    ok &= applyRelayContactState(engine, relay, spec, state.contactState);
    return ok;
}

} // namespace guitardsp::circuit
