# Circuit engine architecture

The circuit engine turns editable electronic component values into an audio-rate solver. It is intentionally separate from named pedal/amp nodes so the same resistor, capacitor, diode, transistor, tube, transformer, switch or relay can be reused across circuits.

## Current execution model

Topology construction and component selection happen on the control thread. `MnaCircuitEngine::prepare()` allocates the dense MNA matrix, right-hand side, solution vectors and branch unknowns. Audio-rate `processSample()` reuses that storage and performs no intentional heap allocation.

The engine/subcircuit layer currently supports:

- resistors, capacitors and inductors with stable prepared handles
- trapezoidal capacitor and inductor companion models
- three-terminal potentiometers with linear/audio/reverse-audio taper
- ideal current/voltage sources and voltage-source branch-current probing
- VCCS, VCVS, CCCS and CCVS controlled sources
- Shockley diodes with series resistance and Newton iteration
- first-generation BJT, JFET and MOSFET nonlinear three-terminal stamps
- a finite-open-loop-gain algebraic op-amp stamp
- a dynamic op-amp subcircuit with dominant pole/GBW, input offset/bias and output resistance
- nonlinear plate/grid/cathode triode stamps
- triode Cgp/Cgk/Cpk parasitics and a positive-grid-current branch
- transformer subcircuits with leakage/winding resistance, magnetizing L, ratio constraints, secondary-current sensing and inter-winding capacitance
- a smooth current-dependent transformer magnetizing-inductance collapse for engineering core saturation
- SPST, SPDT and DPDT switch subcircuits implemented with stable contact-resistance handles
- electromechanical relay subcircuits with an R-L coil in the MNA system, physical coil-current sensing, pickup/dropout hysteresis, operate/release delay and deterministic contact bounce
- arbitrary node graphs with ground node 0
- partial-pivot Gaussian elimination and convergence/singular statistics
- mild startup damping for nonlinear Newton solves

The component values come from `ComponentCatalog.h`, so nominal value, rating/tolerance metadata and nonlinear DSP parameters remain distinct.

## Component catalog boundary

The catalog now includes resistor, capacitor, inductor, potentiometer, diode, BJT, JFET, MOSFET, op-amp, triode, power tube, transformer, optocoupler, switch and relay categories.

Switch specs include contact form, open/closed resistance and bounce parameters. Relay specs include contact form, coil voltage/resistance/inductance, pickup/dropout voltage, operate/release timing, contact resistance and bounce parameters. The included presets are engineering starting points, not manufacturer-identical models.

## What the current active models mean

The current BJT/JFET/MOSFET/op-amp/triode/transformer/relay implementations are engineering models, not full manufacturer SPICE or measured unit models.

- BJT: exponential base-emitter current, beta-derived base current and collector-emitter saturation factor.
- JFET: Shockley-style gate/source control, drain/source direction term and channel-length modulation.
- MOSFET: square-law triode/saturation regions with channel-length modulation.
- Dynamic op-amp: finite DC open-loop gain, dominant pole derived from GBW, input offset/bias and finite output resistance. Rail limiting, slew-rate limiting and output-current limiting are intentionally **not** claimed yet. A first diode-rail experiment was removed after overdrive regression exposed Newton divergence; a bounded-source/output-stage stamp is required before this is promoted.
- Triode: nonlinear plate current plus editable inter-electrode capacitances and an engineering positive-grid-current branch.
- Transformer: linear leakage/winding/parasitic network plus smooth current-dependent magnetizing-inductance collapse. Hysteresis, remanence and measured B-H fitting remain future work.
- Relay: physical R-L coil current determines pickup/dropout state with timing and deterministic contact bounce. Magnetic hysteresis/contact arcing are not yet measured physical models.

Named hardware claims still require parameter provenance and measured fitting.

## Prepared component handles and realtime edits

R/C/L, potentiometers, diodes, BJT/JFET/MOSFET, op-amps and triodes return stable handles when added to a topology. Transformer, switch and relay subcircuits expose stable handles for their internal elements. Topology-preserving numerical edits therefore do not require resizing the MNA system.

`CircuitUpdateQueue` is the realtime handoff for normal component edits. It is a fixed-capacity SPSC queue: the UI/control thread writes commands and the audio thread drains them at a block boundary. Overflow is rejected rather than allocating or blocking.

Switch/relay contact transitions are also topology-preserving: routing is represented by changing pre-existing contact resistances rather than adding/removing graph nodes during audio processing.

## Stable-ID schematic/netlist layer

`CircuitNetlist` is the schematic-facing layer above MNA. User-facing node IDs and component IDs are stable and separate from solver indexes. The in-memory netlist can currently compile:

- R/C/L and potentiometers
- independent voltage sources and all four controlled-source families
- diode/BJT/JFET/MOSFET
- algebraic and dynamic op-amp definitions
- basic and parasitic-aware triodes
- transformer subcircuits
- SPST/SPDT/DPDT switches
- electromechanical relays

`CompiledCircuit` retains component-ID-to-handle/subcircuit bindings, including switch handles, relay subcircuits and relay runtime state. That is the contract intended for a future Circuit Mode UI and serialization layer.

```text
Circuit Mode / schematic UI
          |
     CircuitNetlist
(stable component + node IDs)
          |
       compile
          |
   CompiledCircuit
(ID -> component/subcircuit binding)
          |
    MnaCircuitEngine
          |
R/C/L + nonlinear devices + switches/relays
```

## Relay execution model

A relay is intentionally not a magic boolean router.

```text
coil supply
   |
coil R + L
   |
0 V current sensor
   |
MNA ground

measured coil current
   -> pickup/dropout hysteresis
   -> operate/release timer
   -> deterministic contact bounce
   -> SPST/SPDT/DPDT contact resistances
```

This means a relay can react to the electrical coil drive rather than a hidden UI state. A flyback diode is deliberately **not** baked into the relay; if a circuit needs one it should be wired explicitly as a real diode, which keeps the schematic behavior visible and editable.

## Why MNA first

Modified Nodal Analysis is the most useful general foundation for pedal and amplifier schematics because arbitrary R/C/L networks, independent sources, all controlled-source families, nonlinear devices and electromechanical subcircuits can share one topology.

A future WDF layer can still be added for circuit regions where tree decomposition gives a large realtime performance advantage. The intended architecture is hybrid:

```text
Component catalog
      |
Circuit/netlist description
      |
      +--> MNA/DK solver       arbitrary tightly coupled networks
      |
      +--> WDF solver          efficient passive/tree-like subcircuits
      |
      +--> specialized stage   measured/optimized blocks when justified
```

## Realtime boundary

The audio callback must not allocate memory, parse files/JSON, rebuild topology, resize solver storage, acquire locks or race with control-thread spec writes. Topology-preserving edits should use prepared handles/block-boundary queues. Topology edits require compiling a new circuit and swapping it through the graph host.

## Next engine work

With switch/relay coverage in place, the next high-value work is:

1. a numerically bounded op-amp output-stage stamp with rail and output-current limiting, then slew-rate dynamics
2. stronger active-device parasitics: BJT junction capacitances, MOSFET body diode/capacitances and JFET gate-junction behavior
3. transformer hysteresis/remanence and measured B-H fitting
4. relay flyback/arcing examples as explicit schematic components rather than hidden behavior
5. static/dynamic matrix separation and fixed-pattern/sparse acceleration
6. stronger Newton continuation/voltage limiting and backward-Euler fallback
7. CircuitNetlist serialization/import after the component contract stabilizes
8. migration of named pedals/amps from specialized nodes toward schematic-backed implementations where performance permits

## Validation policy

A circuit implementation is not called hardware-equivalent solely because the schematic topology is represented. Each named model should distinguish topology correctness, datasheet/SPICE parameter provenance, measured component/circuit data and whole-device measured audio fitting.

Regression coverage now includes DC operating points, R/C/L transients, nonlinear diode/transistor biasing, controlled sources, prepared component edits, transformer ratio behavior, transformer saturation-law behavior, triode parasitic stability, switch routing, relay coil pickup/dropout/timing/contact transfer, graph behavior and the existing pedal/amp/cab DSP tests.
