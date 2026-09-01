#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>

namespace guitardsp::circuit {

// Extends the nonlinear plate/grid/cathode triode stamp with external parasitics
// that are naturally represented as ordinary circuit elements:
// - Cgp (Miller-sensitive grid/plate capacitance)
// - Cgk
// - Cpk
// - positive-grid current through an engineering diode branch
//
// Keeping these outside the core triode stamp makes them individually editable
// and visible to a future schematic UI.
struct TriodeParasiticSubcircuit {
    TriodeHandle triode{};
    CapacitorHandle gridPlateCapacitance{};
    CapacitorHandle gridCathodeCapacitance{};
    CapacitorHandle plateCathodeCapacitance{};
    DiodeHandle gridCurrentDiode{};
};

namespace detail {
inline hq::CapacitorSpec triodeCapacitor(float farads) noexcept {
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

inline hq::DiodeSpec triodeGridCurrentDiode(const hq::TriodeSpec& spec) noexcept {
    hq::DiodeSpec diode{};
    diode.name = "Triode positive-grid current";
    diode.technology = hq::DiodeTechnology::silicon;
    diode.nominalForwardVoltage = 0.45f;
    diode.saturationCurrent = std::max(1.0e-18f, spec.gridCurrentSaturationAmps);
    diode.emissionCoefficient = std::max(0.5f, spec.gridCurrentEmissionCoefficient);
    diode.thermalVoltage = 0.02585f;
    diode.seriesResistanceOhms = 100.0f;
    diode.junctionCapacitanceFarads = 0.0f;
    diode.reverseVoltageRating = 1000.0f;
    diode.currentRatingAmps = 0.05f;
    return diode;
}
} // namespace detail

inline TriodeParasiticSubcircuit addTriodeParasiticSubcircuit(MnaCircuitEngine& engine,
                                                               Node plate,
                                                               Node grid,
                                                               Node cathode,
                                                               const hq::TriodeSpec& spec) {
    TriodeParasiticSubcircuit handles{};
    handles.triode = engine.addTriode(plate, grid, cathode, spec);
    handles.gridPlateCapacitance = engine.addCapacitor(grid, plate,
        detail::triodeCapacitor(spec.gridPlateCapacitanceFarads));
    handles.gridCathodeCapacitance = engine.addCapacitor(grid, cathode,
        detail::triodeCapacitor(spec.gridCathodeCapacitanceFarads));
    handles.plateCathodeCapacitance = engine.addCapacitor(plate, cathode,
        detail::triodeCapacitor(spec.plateCathodeCapacitanceFarads));
    handles.gridCurrentDiode = engine.addDiode(grid, cathode,
        detail::triodeGridCurrentDiode(spec));
    return handles;
}

inline bool updateTriodeParasiticSubcircuit(MnaCircuitEngine& engine,
                                            const TriodeParasiticSubcircuit& handles,
                                            const hq::TriodeSpec& spec) noexcept {
    bool ok = true;
    ok &= engine.setTriodeSpec(handles.triode, spec);
    ok &= engine.setCapacitorSpec(handles.gridPlateCapacitance,
        detail::triodeCapacitor(spec.gridPlateCapacitanceFarads));
    ok &= engine.setCapacitorSpec(handles.gridCathodeCapacitance,
        detail::triodeCapacitor(spec.gridCathodeCapacitanceFarads));
    ok &= engine.setCapacitorSpec(handles.plateCathodeCapacitance,
        detail::triodeCapacitor(spec.plateCathodeCapacitanceFarads));
    ok &= engine.setDiodeSpec(handles.gridCurrentDiode,
        detail::triodeGridCurrentDiode(spec));
    return ok;
}

} // namespace guitardsp::circuit
