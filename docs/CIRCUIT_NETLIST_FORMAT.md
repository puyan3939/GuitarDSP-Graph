# Circuit netlist JSON format

This document specifies the JSON format loaded by
`include/guitardsp/circuit/NetlistLoader.h` (`guitardsp::circuit::NetlistCircuit`)
and used by `data/circuits/ts808.json` / `data/circuits/ds1.json` /
`data/circuits/preamp.json` / `data/circuits/poweramp.json`.

## Why this format exists

`TS808Circuit` and `DS1Circuit` (`include/guitardsp/circuit/TS808Circuit.h`,
`DS1Circuit.h`) are hand-written C++ classes that build a component-level MNA
circuit by calling `MnaCircuitEngine::addNode/addResistor/...` and the
subcircuit helpers (`addBjtEbersMollSubcircuit`, `addDiodeParasiticSubcircuit`,
`addDynamicOpAmpSubcircuit`) directly in `prepare()`. Every new pedal/circuit
previously meant writing (and hand-verifying) a new C++ class like these.

This netlist format lets the same physical circuit be described as data
instead: an ordered list of node/component operations that
`NetlistCircuit::prepare()` replays onto a fresh `MnaCircuitEngine`, plus a
small amount of metadata (I/O ports, user-facing controls, DC-priming/warm-up
parameters) that a hand-written class would otherwise hard-code.

**Numerical parity depends on operation order.** `MnaCircuitEngine::addNode()`
assigns MNA unknown indices in call order, so a netlist document must list
nodes and components in *exactly* the order the equivalent hand-written
`prepare()` would call them. `data/circuits/ts808.json` and `ds1.json` are
transcribed op-for-op from `TS808Circuit::prepare()` / `DS1Circuit::prepare()`
for this reason; see `tests/NetlistParityTests.cpp`, which asserts
sample-by-sample output agreement between each hand-written class and its
JSON netlist across a matrix of control settings.

## Real-time contract

Parsing and loading (`NetlistCircuit::loadFromJson` / `loadFromFile` /
`prepare`) must only run on the control thread, exactly like
`TS808Circuit::prepare()` does today. Nothing in the loader runs from
`NetlistCircuit::processSample()`, which only touches the already-prepared
`MnaCircuitEngine`. Do not call the loader from the audio callback path.

## Top-level document shape

```jsonc
{
  "name": "TS808",                  // informational
  "description": "...",             // informational
  "ops": [ /* see "Ops" below */ ],
  "ports": {
    "input":  "inputSource",        // required: name of a "voltageSource" op driven by processSample(input)
    "output": "outputNode",         // required: name of a "node" op read back by processSample()
    "supply": "supplySource",       // optional: DC supply rail source, used for priming (see below)
    "vref":   "vrefSource"          // optional: DC bias/reference rail source, used for priming
  },
  "controls": {
    "drive": { "pot": "drivePot", "invert": true },
    "tone":  { "pot": "tonePot",  "invert": true },
    "level": { "pot": "levelPot", "invert": false }
  },
  "simulation": { /* see "Simulation parameters" below */ }
}
```

## Ops

`ops` is an ordered array. Each entry is an object with an `"op"` field
selecting its kind, plus kind-specific fields below. Node/source/pot
references are always by the `"name"` a previous op registered; the implicit
ground node is always available as `"ground"`.

| op              | fields                                                                 | registers name as |
|-----------------|-------------------------------------------------------------------------|--------------------|
| `node`          | `name`                                                                   | node |
| `voltageSource` | `name`, `p`, `n`, `volts`                                                | voltage source |
| `resistor`      | `a`, `b`, `ohms`                                                         | (none) |
| `capacitor`     | `a`, `b`, `farads`, `voltageRating`, `technology`                        | (none) |
| `potentiometer` | `name`, `high`, `wiper`, `low`, `ohms`, `taper`, `position`              | pot (and its wiper node, under the pot's own name) |
| `opAmp`         | `output`, `nonInverting`, `inverting`, `reference`, `preset`\|`spec`     | (none) |
| `dynamicOpAmp`  | `name`, `output`, `nonInverting`, `inverting`, `positiveRail`, `negativeRail`, `reference`, `preset`\|`spec` | (none) |
| `bjtEbersMoll`  | `name`, `collector`, `base`, `emitter`, `preset`\|`spec` (with `overrides`)| (none) |
| `diodeParasitic`| `name`, `anode`, `cathode`, `preset`\|`spec` (with `overrides`)          | (none) |
| `triodeParasitic`| `name`, `plate`, `grid`, `cathode`, `preset`\|`spec` (with `overrides`) | (none) |
| `pentodeParasitic`| `name`, `plate`, `grid`, `screen`, `cathode`, `preset`\|`spec` (with `overrides`) | (none) |
| `transformer`   | `name`, `primaryPositive`, `primaryNegative`, `secondaryPositive`, `secondaryNegative`, `spec` | (none) |

Notes:

- `opAmp` compiles to a plain finite-gain `MnaCircuitEngine::addOpAmp` (used
  where the hand-written circuit keeps a linear op-amp stage, e.g. TS808's
  tone buffer). `dynamicOpAmp` compiles to the full large-signal
  `addDynamicOpAmpSubcircuit` macro (saturation, slew-rate limiting, rail
  clamping) used for stages that actually clip.
- `bjtEbersMoll` and `diodeParasitic` always compile to
  `addBjtEbersMollSubcircuit` / `addDiodeParasiticSubcircuit` — the same
  two-junction BJT and capacitance-aware diode macros TS808Circuit/DS1Circuit
  use. This is deliberately different from the generic `bjt`/`diode` ops in
  the existing control-thread `CircuitNetlist` builder
  (`include/guitardsp/circuit/CircuitNetlist.h`), which compile to
  `addBjtParasiticSubcircuit`/plain `addDiode` — a different nonlinear model.
  A netlist that mixed the two would not reach parity with a hand-written
  pedal class, so this format only exposes the pedal-parity primitives.
- `triodeParasitic` compiles to `addTriodeParasiticSubcircuit` — the plate/
  grid/cathode Koren-model triode stamp plus its Cgp/Cgk/Cpk parasitic
  capacitances and positive-grid-current diode, exactly as PreampCircuit uses
  it. Built-in presets: `"12ax7"`, `"12at7"`.
- `pentodeParasitic` compiles to `addPentodeParasiticSubcircuit` — the plate/
  grid/screen/cathode pentode stamp plus its Cgp/Cgk/Cpk/Csk parasitic
  capacitances and positive-grid-current diode, exactly as PowerAmpCircuit
  uses it. Built-in presets: `"el34"`, `"6l6gc"`, `"kt88"`.
- `transformer` compiles to `addTransformerSubcircuit` — primary/secondary
  leakage and winding resistance, a magnetizing inductance, an ideal-ratio
  VCVS+CCCS pair, a zero-volt secondary current sensor and an interwinding
  capacitor, exactly as PowerAmpCircuit's output transformer uses it. Unlike
  the other nonlinear ops it has no catalog preset (`hq::TransformerSpec` has
  no `component_presets` entry), so `spec` must supply every field explicitly
  — see `data/circuits/poweramp.json`. `NetlistCircuit::processSample()`
  (and its silent warm-up in `prepare()`) re-saturates every declared
  transformer's magnetizing inductance after each sample, mirroring
  `PowerAmpCircuit::updateOutputTransformerSaturation()` exactly, including
  the "only push an update through when it moved more than float noise"
  guard that avoids forcing a static-matrix-cache rebuild every sample at
  idle.
- `technology` (capacitor) is one of `"generic"`, `"ceramic"`, `"film"`,
  `"electrolytic"`, `"tantalum"`. `taper` (potentiometer) is one of
  `"linear"`, `"audio"`, `"reverseAudio"`.
- Resistor/capacitor/potentiometer tolerance and power-rating metadata are
  fixed to the same pedal-grade engineering defaults
  `TS808Circuit`/`DS1Circuit` use internally (5% / 0.25 W for resistors, 20% /
  0.25 W for pots, technology-dependent ESR/leakage/tolerance for capacitors)
  — a netlist only needs to supply the primary electrical value(s). See
  `NetlistCircuit::resistorSpec/capacitorSpec/potSpec` in NetlistLoader.h if a
  future netlist needs different metadata.

### Device specs: `preset` vs `spec`/`overrides`

`opAmp`, `dynamicOpAmp`, `bjtEbersMoll` and `diodeParasitic` ops take either:

- `"preset": "<name>"` — start from a named built-in catalog preset
  (`guitardsp::hq::component_presets`), optionally followed by
  `"overrides": { ...fields to change... }`; or
- `"spec": { ...every field... }` — a fully explicit spec with no preset
  base (used e.g. for DS-1's bespoke NJM3404-style op-amp, which is not
  derived from any catalog preset).

Built-in preset names: `"2n3904"`, `"2n5088"` (BJT), `"1n4148"`, `"1n34a"`,
`"redLed"` (diode), `"jrc4558"`, `"tl072"` (op-amp).

Overridable fields mirror `guitardsp::hq::{BJTSpec,DiodeSpec,OpAmpSpec}`
(`include/guitardsp/hq/ComponentCatalog.h`) field names exactly, e.g. for a
BJT: `name`, `polarity`, `beta`, `nominalVbe`, `saturationVoltage`,
`thermalVoltage`, `maxCollectorVoltage`, `maxCollectorCurrentAmps`,
`inputCapacitanceFarads`.

## Ports

- `input` / `output` are required: `processSample(x)` drives the named
  voltage source with `x` and returns the voltage at the named node.
- `supply` / `vref` are optional. When `supply` is present, `prepare()` runs
  the same source-stepping DC-priming continuation TS808Circuit/DS1Circuit
  use (ramping the supply source, and the vref source if also present, from
  0 V up to `simulation.supplyVolts`/`vrefVolts` over `simulation.sourceSteps`
  steps) before the silence warm-up. `vref` only matters for circuits whose
  active devices bias around a mid-supply virtual ground (TS808/DS1's
  transistor/op-amp stages, which specify both); a self-biased triode stage
  returns to true ground instead and only needs `supply` (see
  `data/circuits/preamp.json`). Omit `supply` entirely for a circuit with no
  biased/nonlinear DC operating point to prime.

## Controls

Each entry in `controls` names a user-facing control (e.g. `"drive"`,
`"tone"`, `"level"`) and binds it to a previously declared potentiometer,
plus whether the control's normalized `[0,1]` value is inverted before being
applied as the pot's mechanical position (`position = invert ? 1 - value :
value`). The initial/default value of each control is derived from the
bound pot's initial `"position"` in its `ops` entry — there is no separate
default field, so the two must agree by construction.

`NetlistCircuit::setControl(name, normalized)` sets a control's target value;
`processSample()` ramps the applied pot position toward that target using
the same >=24 kHz-update / 5 ms-ramp policy as
`TS808Circuit`/`DS1Circuit::applySmoothedControls()`.

## Simulation parameters

All fields in `simulation` are optional and default to the values below
(TS808's warm-up window; override per-circuit as needed, see `ds1.json` for a
circuit using a shorter warm-up):

| field                        | default  | meaning |
|-------------------------------|---------|---------|
| `sourceSteps`                 | 128     | DC-priming continuation steps |
| `solvesPerStep`               | 2       | Newton solves per priming step |
| `warmupSecondsFraction`       | 0.08    | silent warm-up length, as a fraction of sample rate |
| `warmupMinSamples`            | 512     | warm-up length lower bound |
| `warmupMaxSamples`            | 8192    | warm-up length upper bound |
| `nonlinearResidualTolerance`  | 2.0e-5  | `MnaCircuitEngine::setNonlinearResidualTolerance` after priming |
| `newtonMaxIterations`         | 40      | `processSample()`'s Newton iteration cap |
| `newtonTolerance`             | 2.0e-5  | `processSample()`'s Newton voltage tolerance |
| `supplyVolts`                 | 9.0     | DC-priming target for `ports.supply` |
| `vrefVolts`                   | 4.5     | DC-priming target for `ports.vref` |

## Loading a netlist

```cpp
#include "guitardsp/circuit/NetlistLoader.h"

guitardsp::circuit::NetlistCircuit circuit;
std::string error;
if (!circuit.loadFromFile("data/circuits/ts808.json", &error)) { /* handle error */ }
if (!circuit.prepare(48000.0, &error)) { /* handle error */ }
circuit.setControl("drive", 0.6f);
circuit.setControl("tone", 0.5f);
circuit.setControl("level", 0.55f);
const float y = circuit.processSample(x); // real-time safe from here on
```

`loadFromFile`/`loadFromJson`/`prepare` must be called off the audio thread,
same as any other circuit's `prepare()`.

## Amp/cabinet netlist status

This format was designed to bring TS808/DS-1 to numerical parity as the
first data-driven pedals; component-level amp stages built directly on
`MnaCircuitEngine` (`PreampCircuit`, `PowerAmpCircuit`) have since reached the
same parity, via `data/circuits/preamp.json` / `poweramp.json` and the
`triodeParasitic`/`pentodeParasitic`/`transformer` ops above (see issue #43).
`FullAmpCircuit` (`include/guitardsp/circuit/FullAmpCircuit.h`) cascades
`PreampCircuit`'s and `PowerAmpCircuit`'s `processSample()` calls rather than
sharing one `MnaCircuitEngine`, so it is netlist-ified today as two separate
netlists driven in series (load `preamp.json` and `poweramp.json` into two
`NetlistCircuit` instances and feed one's output into the other's input) —
there is no single combined `fullamp.json` document, and adding one would
require merging both circuits into one `MnaCircuitEngine` (see
`FullAmpCircuit.h`'s comment on why that was deferred).

What is still out of scope: amp models under `include/guitardsp/hq/`
(`ReferenceAmpTopologyNode.h`, `AmpFamilyNodes.h`, etc.) are parameterized DSP
stage compositions, not `guitardsp::circuit` MNA netlists, so they are not
representable in this format without first re-modelling them at the
component level — see the policy in `CLAUDE.md` for new amp work. Cabinets
(`CabinetChainNode.h`) are speaker-dynamics plus measured-impulse-response
convolution, not a circuit network, so they are out of scope for
netlist-ification entirely.
