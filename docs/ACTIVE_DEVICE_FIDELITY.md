# Active-device fidelity layer

This document records the large-signal op-amp and transistor-parasitic work added during the Phase 3 MNA acceleration branch.

## Dynamic op-amp

`DynamicOpAmpSubcircuit` now models the following effects inside the circuit solve:

- finite DC open-loop gain;
- dominant-pole / gain-bandwidth behavior;
- input offset voltage and input bias current;
- finite output resistance;
- positive and negative rail headroom;
- slew-rate limiting;
- bidirectional output-current limiting.

The large-signal path is intentionally split into separate responsibilities:

1. the high-gain differential stage drives a dominant-pole capacitor;
2. an ideal buffer isolates that high-impedance compensation node;
3. a bounded-current JFET stage charges a dedicated slew capacitor, with `Idss = C * slewRate`;
4. polynomial MOSFET shunts limit the slew node against rail-relative clamp nodes;
5. a second buffer isolates the slew state from the load;
6. a finite output resistance and a bounded-current JFET stage drive the external output.

The rail shunts deliberately do **not** use a Shockley diode on the high-gain internal node. An earlier exponential clamp could require many junction iterations after severe overdrive. The current rail limiter uses the MNA MOSFET square-law stamp so the restoring Jacobian remains polynomial even when the internal command is far outside the supply rails.

The model is still an engineering macro model. It is not a manufacturer transistor-level macromodel for a JRC4558, TL072, or any other named part until measured/SPICE-derived parameters justify that claim.

## BJT parasitics

`BjtParasiticSubcircuit` expands the nonlinear BJT stamp with ordinary MNA capacitors for:

- Cbe, base to emitter;
- Cbc, base to collector.

The normal `CircuitNetlist::addBjt()` path now compiles through this parasitic subcircuit automatically. The current compact `BJTSpec::inputCapacitanceFarads` is split 70/30 between Cbe and Cbc as an engineering fallback. The lower-level subcircuit API accepts explicit Cbe/Cbc values when better datasheet, SPICE, or measured data are available.

## JFET parasitics

`JfetParasiticSubcircuit` adds:

- Cgs;
- Cgd;
- Cds.

The normal netlist path compiles these automatically. `JFETSpec::gateSourceCapacitanceFarads` supplies the existing catalog value; Cgd and Cds are engineering fallback ratios until explicit device data are supplied. The low-level subcircuit API accepts all three values independently.

## MOSFET parasitics and body diode

`MosfetParasiticSubcircuit` adds:

- Cgs;
- Cgd;
- Cds;
- the intrinsic body-diode branch.

The normal netlist path compiles this network automatically. The body diode uses `MOSFETSpec::bodyDiodeForwardVoltage` to derive an engineering Shockley model and is oriented according to N- or P-channel polarity. Explicit Cgs/Cgd/Cds values can be supplied through the low-level subcircuit API.

## Realtime implications

All parasitic capacitances are ordinary trapezoidal MNA companion elements. Their matrix conductance terms participate in the Phase 3 static/companion cache, while their history currents are updated through the per-sample RHS. This is important: adding device capacitances does not revert the whole circuit to full linear restamping every sample.

The op-amp rail/slew/current stages are nonlinear and therefore use the Newton path. The Phase 3 solver still uses a dense solve for each nonlinear Newton iteration; fixed-pattern sparse nonlinear acceleration remains later work.

## Regression coverage

`ActiveDeviceFidelityTests` verifies:

- severe op-amp overdrive remains finite and returns to Newton convergence;
- configured slew rate bounds the output step;
- output voltage is bounded by rail headroom;
- output-current limiting bounds voltage into a heavy load;
- BJT Cbe, JFET Cgs, and MOSFET Cgs change the actual MNA transient response;
- the MOSFET body diode clamps reverse drain excursion.

`CircuitNetlistTests` additionally verifies that ordinary BJT/JFET/MOSFET definitions compile into their parasitic subcircuits rather than silently discarding the catalog capacitance data.

## Current modelling boundary

The topology and parameter plumbing are now present, but the fallback capacitance splits/ratios are not manufacturer-accurate device data. The next fidelity step is to feed explicit Cbe/Cbc, Cgs/Cgd/Cds, voltage-dependent junction capacitance, and named op-amp large-signal parameters from datasheets, SPICE models, or measurements. The architecture is designed so those data can replace the fallback values without changing the circuit topology contract.
