# Circuit engine architecture

The circuit engine is the layer that turns editable electronic component values into an audio-rate solver. It is intentionally separate from named pedal/amp nodes so the same resistor, capacitor, diode or active-device model can be reused across circuits.

## Current execution model

Topology construction and component selection happen on the control thread. `MnaCircuitEngine::prepare()` allocates the dense MNA matrix, right-hand side, solution vectors and branch unknowns. Audio-rate `processSample()` reuses that storage and performs no intentional heap allocation.

The initial engine supports:

- resistors
- capacitors using trapezoidal companion models
- inductors using trapezoidal branch companion models
- ideal current sources
- ideal voltage sources with realtime value updates
- Shockley diodes with junction series resistance and Newton iteration
- arbitrary node graphs with ground node 0
- partial-pivot Gaussian elimination
- convergence/singular statistics for analyzer and test reporting

The component values come from `ComponentCatalog.h`, so nominal value, rating/tolerance metadata and nonlinear DSP parameters remain distinct.

## Why MNA first

Modified Nodal Analysis is the most useful general foundation for pedal and amplifier schematics because arbitrary R/C/L networks, voltage sources and nonlinear devices can share one topology. It also gives a direct path to importing component-value schematics rather than rewriting each filter as a hand-derived transfer function.

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

Changing a voltage/control source is allowed because it only updates pre-existing scalar values. Component-value editing that changes the numerical coefficients but not topology can later be exposed as a prepared control update. Topology edits require compiling a new circuit instance and swapping it through the graph host.

## Next device stamps

The next high-value stamps are:

1. potentiometer as a three-terminal element with editable taper/position
2. BJT Ebers-Moll/Gummel-Poon-inspired nonlinear stamp
3. JFET and MOSFET nonlinear stamps
4. op-amp macro-model with finite open-loop gain, dominant pole, slew/rails
5. triode plate/grid/cathode stamp using the existing triode model
6. transformer coupled-inductor model including leakage and winding resistance
7. controlled sources (VCCS/VCVS/CCCS/CCVS)
8. switches and relays for pedal bypass/channel topology compilation

## Numerical roadmap

The current dense Gaussian solver is a correctness/reference engine. It is appropriate for small pedal/amp subcircuits and CI regression but should not be the final performance path for large circuits.

Planned upgrades:

- compile constant matrix structure once
- separate static and dynamic stamps
- sparse/fixed-pattern factorization for linear sections
- Newton residual damping and voltage limiting for difficult nonlinear networks
- per-device convergence diagnostics
- optional backward Euler fallback after non-convergence
- subcircuit partitioning so only nonlinear regions iterate
- oversampled circuit islands chosen by the graph quality policy
- exact measured latency accounting when circuit islands include resampling

## Validation policy

A circuit implementation is not called hardware-equivalent solely because the schematic topology is represented. Each named model should distinguish:

- topology correctness
- datasheet/SPICE parameter provenance
- measured component/circuit data
- whole-device measured audio fitting

Regression tests should cover DC operating points, AC/frequency response, transient response, nonlinear transfer curves, harmonic content, convergence and block/sample continuity before a circuit is promoted into a production named model.
