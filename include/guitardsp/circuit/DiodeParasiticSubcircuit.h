#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>

namespace guitardsp::circuit {

// A diode in a real pedal is not only a static I-V curve.  The catalog already
// carries junction capacitance, so this helper makes that value participate in
// the actual transient solve without hiding it inside the nonlinear stamp.
// Keeping the capacitor explicit also means the value can later be replaced by
// datasheet/SPICE/measurement data without changing the diode equation itself.
struct DiodeParasiticSubcircuit {
    DiodeHandle diode{};
    CapacitorHandle junctionCapacitance{};
};

namespace diode_parasitic_detail {
inline hq::CapacitorSpec junctionCapacitor(const hq::DiodeSpec& spec) noexcept {
    hq::CapacitorSpec c{};
    c.capacitanceFarads = std::max(0.0f, spec.junctionCapacitanceFarads);
    c.tolerancePercent = 0.0f;
    c.voltageRatingVolts = std::max(1.0f, spec.reverseVoltageRating);
    c.esrOhms = 0.0f;
    c.leakageResistanceOhms = 1.0e12f;
    c.dielectricAbsorption = 0.0f;
    c.technology = hq::CapacitorTechnology::generic;
    return c;
}
} // namespace diode_parasitic_detail

inline DiodeParasiticSubcircuit addDiodeParasiticSubcircuit(
        MnaCircuitEngine& engine,
        Node anode,
        Node cathode,
        const hq::DiodeSpec& spec) {
    DiodeParasiticSubcircuit handles{};
    handles.diode = engine.addDiode(anode, cathode, spec);
    handles.junctionCapacitance = engine.addCapacitor(
        anode, cathode, diode_parasitic_detail::junctionCapacitor(spec));
    return handles;
}

inline bool updateDiodeParasiticSubcircuit(MnaCircuitEngine& engine,
                                            const DiodeParasiticSubcircuit& handles,
                                            const hq::DiodeSpec& spec) noexcept {
    bool ok = true;
    ok &= engine.setDiodeSpec(handles.diode, spec);
    ok &= engine.setCapacitorSpec(handles.junctionCapacitance,
                                  diode_parasitic_detail::junctionCapacitor(spec));
    return ok;
}

} // namespace guitardsp::circuit
