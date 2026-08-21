# Circuit engine architecture

The circuit engine is the layer that turns editable electronic component values into an audio-rate solver. It is intentionally separate from named pedal/amp nodes so the same resistor, capacitor, diode or active-device model can be reused across circuits.

## Current execution model

Topology construction and component selection happen on the control thread. `MnaCircuitEngine::prepare()` allocates the dense MNA matrix, right-hand side, solution vectors and branch unknowns. Audio-rate `processSample()` reuses that storage and performs no intentional heap allocation.

The engine currently supports:

- resistors with stable handles for prepared value/spec replacement
- capacitors using trapezoidal companion models, with stable handles for capacitance/spec replacement
- inductors using trapezoidal branch companion models, with stable handles for inductance/spec replacement
- three-terminal potentiometers with linear/audio/reverse-audio taper and prepared position/spec updates
- ideal current sources
- ideal voltage sources with prepared value updates and branch-current probing
- voltage-controlled current sources (VCCS)
- voltage-controlled voltage sources (VCVS) with an MNA branch unknown
- current-controlled current sources (CCCS) driven from a voltage-source branch current
- current-controlled voltage sources (CCVS) with transresistance and their own MNA branch unknown
- Shockley diodes with junction series resistance, Newton iteration and replaceable device specs
- first-generation BJT nonlinear three-terminal stamps with replaceable device specs
- first-generation JFET nonlinear three-terminal stamps with replaceable device specs
- first-generation MOSFET nonlinear three-terminal stamps with replaceable device specs
- a finite-open-loop-gain op-amp macro stamp with replaceable `OpAmpSpec`
- a nonlinear plate/grid/cathode triode stamp using the shared 12AX7/12AT7 model family
- arbitrary node graphs with ground node 0
- partial-pivot Gaussian elimination
- convergence/singular statistics for analyzer and test reporting
- mild startup damping for nonlinear Newton solves

The component values come from `ComponentCatalog.h`, so nominal value, rating/tolerance metadata and nonlinear DSP parameters remain distinct. The active-device stamps consume those same component specs rather than embedding pedal- or amplifier-specific constants in the solver.

## What the current active stamps mean

The current BJT/JFET/MOSFET/op-amp/triode implementations are engineering device stamps, not full SPICE-equivalent manufacturer models.

- BJT: exponential base-emitter current, beta-derived base current and a collector-emitter saturation factor.
- JFET: Shockley-style gate/source control, drain/source direction term and channel-length modulation.
- MOSFET: square-law triode/saturation regions with channel-length modulation.
- Op-amp: algebraic finite open-loop gain and input offset in the common MNA solve. `gainBandwidthHz`, slew rate, rail headroom, output current limit and noise are catalogued but not yet enforced by this MNA macro.
- Triode: the shared nonlinear plate-current model is stamped directly as a plate-to-cathode current controlled by grid-to-cathode and plate-to-cathode voltage. Numerical derivatives form the Newton Jacobian. Grid current and inter-electrode capacitances are not yet modeled.

They are sufficient to build and validate active circuit topologies in the common MNA system. Named hardware claims still require better parameter provenance and, where justified, higher-order models or measured device fitting.

## Prepared component handles

R/C/L, potentiometers, diodes, BJT/JFET/MOSFET, op-amps and triodes return stable handles when added to a topology. A handle allows the numerical component spec to be changed without changing node connectivity or resizing the MNA system. This is the basis for circuit-level editing such as:

```text
C8: 47 nF -> 22 nF
R12: 4.7 kOhm -> 2.2 kOhm
Q1: 2N3904-style -> 2N5088-style
D1: 1N4148-style -> 1N34A-style
IC1: JRC4558-style -> edited op-amp spec
V1: 12AX7 -> 12AT7
```

`CircuitUpdateQueue` is the realtime handoff for those edits. It is a fixed-capacity SPSC queue: the control/UI thread writes commands, and the audio thread drains them at a block boundary before processing samples. The queue stores scalar edits and complete component specs without growing or allocating storage. Overflow is rejected rather than allocating or blocking.

Direct setter calls still do not claim lock-free concurrent mutation. The audio thread reads component structs while solving, so callers should either use `CircuitUpdateQueue`, otherwise serialize direct writes at a block boundary, or submit an inactive/prepared circuit and swap it through the graph host.

## Why MNA first

Modified Nodal Analysis is the most useful general foundation for pedal and amplifier schematics because arbitrary R/C/L networks, independent sources, all four controlled-source families and nonlinear devices can share one topology. It also gives a direct path to importing component-value schematics rather than rewriting each filter as a hand-derived transfer function.

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
- race with direct component-spec writes from another thread

Topology-preserving edits should arrive through the fixed-capacity block-boundary update queue. Topology edits require compiling a new circuit instance and swapping it through the graph host.

## Next device/engine work

The next high-value work is:

1. transformer/coupled-inductor stamp including winding resistance and leakage, followed later by core saturation and parasitic capacitance
2. op-amp dominant-pole/GBW behavior, output impedance, current limiting, rail limiting and slew-rate dynamics
3. triode grid-current onset and inter-electrode capacitances
4. switches and relays for pedal bypass/channel topology compilation
5. BJT junction capacitances and a stronger Ebers-Moll/Gummel-Poon-inspired model
6. MOSFET body diode/capacitances and JFET gate-junction behavior
7. typed branch-current probes beyond independent voltage sources where a circuit needs them
8. netlist/subcircuit descriptions that can compile schematic-like component graphs into prepared MNA islands

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
