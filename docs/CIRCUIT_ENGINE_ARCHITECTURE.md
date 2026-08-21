# Circuit engine architecture

The circuit engine is the layer that turns editable electronic component values into an audio-rate solver. It is intentionally separate from named pedal/amp nodes so the same resistor, capacitor, diode or active-device model can be reused across circuits.

## Current execution model

Topology construction and component selection happen on the control thread. `MnaCircuitEngine::prepare()` allocates the dense MNA matrix, right-hand side, solution vectors and branch unknowns. Audio-rate `processSample()` reuses that storage and performs no intentional heap allocation.

The engine currently supports:

- resistors
- capacitors using trapezoidal companion models
- inductors using trapezoidal branch companion models
- three-terminal potentiometers with linear/audio/reverse-audio taper and realtime position updates
- ideal current sources
- ideal voltage sources with realtime value updates
- voltage-controlled current sources (VCCS)
- voltage-controlled voltage sources (VCVS) with an MNA branch unknown
- Shockley diodes with junction series resistance and Newton iteration
- first-generation BJT nonlinear three-terminal stamps
- first-generation JFET nonlinear three-terminal stamps
- first-generation MOSFET nonlinear three-terminal stamps
- arbitrary node graphs with ground node 0
- partial-pivot Gaussian elimination
- convergence/singular statistics for analyzer and test reporting
- mild startup damping for nonlinear Newton solves

The component values come from `ComponentCatalog.h`, so nominal value, rating/tolerance metadata and nonlinear DSP parameters remain distinct. The active-device stamps consume those same BJT/JFET/MOSFET specs rather than embedding pedal-specific constants in the solver.

## What the current active stamps mean

The current BJT/JFET/MOSFET implementations are engineering device stamps, not full SPICE-equivalent manufacturer models.

- BJT: exponential base-emitter current, beta-derived base current and a collector-emitter saturation factor.
- JFET: Shockley-style gate/source control, drain/source direction term and channel-length modulation.
- MOSFET: square-law triode/saturation regions with channel-length modulation.

They are sufficient to build and validate transistor-biased circuit topologies in the common MNA system. Named hardware claims still require better parameter provenance and, where justified, higher-order models such as Ebers-Moll/Gummel-Poon or measured device fitting.

## Why MNA first

Modified Nodal Analysis is the most useful general foundation for pedal and amplifier schematics because arbitrary R/C/L networks, voltage sources, controlled sources and nonlinear devices can share one topology. It also gives a direct path to importing component-value schematics rather than rewriting each filter as a hand-derived transfer function.

A future WDF layer can still be added for circuit regions where tree decomposition gives a large realtime performance advantage. The intended architecture is hybrid rather than forcing every circuit into one solver:

```text
Component catalog
      |
Circuit/netlist description
      |
      +--> MNA/DK solver       arbitrary tightly coupled networks
      |
      +--> WDF solver          efficient passive/tree-like subcircuits
      |
      +--> specialized stage   measured/optimized device blocks when justified
```

## Realtime boundary

The audio callback must not:

- allocate memory
- change topology
- parse files/JSON
- rebuild matrices/vectors
- acquire locks

Changing a voltage source, controlled-source coefficient or potentiometer position is allowed because it only updates pre-existing scalar values. Component-value editing that changes numerical coefficients but not topology is intended to use the same prepared-control path. Topology edits require compiling a new circuit instance and swapping it through the graph host.

## Next device/engine stamps

The next high-value work is:

1. generic prepared component handles so R/C/L/device specs can be edited without topology rebuild
2. current-controlled sources (CCCS/CCVS) and explicit branch-current handles
3. op-amp macro-model using finite open-loop gain, dominant pole/GBW, output impedance and later slew/rail limiting
4. triode plate/grid/cathode nonlinear stamp using the existing 12AX7/12AT7 models
5. transformer coupled-inductor model including leakage, winding resistance and core saturation
6. switches and relays for pedal bypass/channel topology compilation
7. BJT junction capacitances and a stronger Ebers-Moll/Gummel-Poon-inspired model
8. MOSFET body diode/capacitances and JFET gate-junction behavior

## Numerical roadmap

The current dense Gaussian solver is a correctness/reference engine. It is appropriate for small pedal/amp subcircuits and CI regression but should not be the final performance path for large circuits.

Planned upgrades:

- compile constant matrix structure once
- separate static and dynamic stamps
- sparse/fixed-pattern factorization for linear sections
- stronger Newton residual damping, voltage limiting and continuation for difficult nonlinear networks
- per-device convergence diagnostics
- optional backward Euler fallback after non-convergence
- subcircuit partitioning so only nonlinear regions iterate
- oversampled circuit islands chosen by the graph quality policy
- exact measured latency accounting when circuit islands include resampling
- optional WDF compilation for suitable passive/tree regions

## Validation policy

A circuit implementation is not called hardware-equivalent solely because the schematic topology is represented. Each named model should distinguish:

- topology correctness
- datasheet/SPICE parameter provenance
- measured component/circuit data
- whole-device measured audio fitting

Regression tests should cover DC operating points, AC/frequency response, transient response, nonlinear transfer curves, harmonic content, convergence and block/sample continuity before a circuit is promoted into a production named model.
