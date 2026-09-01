#pragma once

#include "MnaCircuitEngine.h"

#include <algorithm>

namespace guitardsp::circuit {

// Extends the nonlinear plate/grid/screen/cathode pentode stamp with external
// parasitics that are naturally represented as ordinary circuit elements:
// - Cgp (grid1/plate; small because the screen grid shields the control grid)
// - Cgk
// - Cpk
// - Csk (screen/cathode bypass)
// - positive-grid current through an engineering diode branch
//
// Keeping these outside the core pentode stamp makes them individually editable
// and visible to a future schematic UI, mirroring TriodeParasiticSubcircuit.
struct PentodeParasiticSubcircuit {
    PentodeHandle pentode{};
    CapacitorHandle gridPlateCapacitance{};
    CapacitorHandle gridCathodeCapacitance{};
    CapacitorHandle plateCathodeCapacitance{};
    CapacitorHandle screenCathodeCapacitance{};
    DiodeHandle gridCurrentDiode{};
};

namespace detail {
inline hq::CapacitorSpec pentodeCapacitor(float farads) noexcept {
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

inline hq::DiodeSpec pentodeGridCurrentDiode(const hq::PentodeSpec& spec) noexcept {
    hq::DiodeSpec diode{};
    diode.name = "Pentode positive-grid current";
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

inline PentodeParasiticSubcircuit addPentodeParasiticSubcircuit(MnaCircuitEngine& engine,
                                                                 Node plate,
                                                                 Node grid,
                                                                 Node screen,
                                                                 Node cathode,
                                                                 const hq::PentodeSpec& spec) {
    PentodeParasiticSubcircuit handles{};
    handles.pentode = engine.addPentode(plate, grid, screen, cathode, spec);
    handles.gridPlateCapacitance = engine.addCapacitor(grid, plate,
        detail::pentodeCapacitor(spec.gridPlateCapacitanceFarads));
    handles.gridCathodeCapacitance = engine.addCapacitor(grid, cathode,
        detail::pentodeCapacitor(spec.gridCathodeCapacitanceFarads));
    handles.plateCathodeCapacitance = engine.addCapacitor(plate, cathode,
        detail::pentodeCapacitor(spec.plateCathodeCapacitanceFarads));
    handles.screenCathodeCapacitance = engine.addCapacitor(screen, cathode,
        detail::pentodeCapacitor(spec.screenCathodeCapacitanceFarads));
    handles.gridCurrentDiode = engine.addDiode(grid, cathode,
        detail::pentodeGridCurrentDiode(spec));
    return handles;
}

inline bool updatePentodeParasiticSubcircuit(MnaCircuitEngine& engine,
                                             const PentodeParasiticSubcircuit& handles,
                                             const hq::PentodeSpec& spec) noexcept {
    bool ok = true;
    ok &= engine.setPentodeSpec(handles.pentode, spec);
    ok &= engine.setCapacitorSpec(handles.gridPlateCapacitance,
        detail::pentodeCapacitor(spec.gridPlateCapacitanceFarads));
    ok &= engine.setCapacitorSpec(handles.gridCathodeCapacitance,
        detail::pentodeCapacitor(spec.gridCathodeCapacitanceFarads));
    ok &= engine.setCapacitorSpec(handles.plateCathodeCapacitance,
        detail::pentodeCapacitor(spec.plateCathodeCapacitanceFarads));
    ok &= engine.setCapacitorSpec(handles.screenCathodeCapacitance,
        detail::pentodeCapacitor(spec.screenCathodeCapacitanceFarads));
    ok &= engine.setDiodeSpec(handles.gridCurrentDiode,
        detail::pentodeGridCurrentDiode(spec));
    return ok;
}

} // namespace guitardsp::circuit
