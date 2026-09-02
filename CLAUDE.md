# CLAUDE.md

Guidance for Claude Code (and other agents) working in this repository.

## Project overview

GuitarDSP-Graph is a next-generation, graph-based guitar DSP engine. Instead of a
fixed MAIN/CLEAN/SUB signal chain, the engine is built around a directed audio
graph so that simple rigs, complex multi-amp rigs, DI taps, pitch branches,
studio chains and experimental DSP can all be composed from the same core.

Long-term vision (see `docs/ARCHITECTURE.md` for the full phase roadmap):

1. **Phase 1** — finish a complete, physically-modelled JUCE desktop app: real
   circuit-based (MNA — Modified Nodal Analysis) simulation of amps, cabinets
   and stomp boxes, running on the graph engine.
2. **Phase 2** — turn the engine into a platform where circuits/block diagrams
   can be composed and swapped by anyone, not just hand-authored in C++.
3. **Phase 3** — port the platform to dedicated hardware (e.g. SHARC/PIC
   targets).

**Current state of the code (important):** the graph kernel is done and a real
MNA circuit engine, two component-level pedal circuits, a JSON netlist format
for them, and an optional JUCE standalone host have all landed on `main`.
Concretely:

- **Graph kernel** (`include/guitardsp/graph/`, `src/graph/`) — topology,
  validation, deterministic scheduling, latency analysis, hot-swappable
  compiled plans via `RealtimeGraphHost`. This was the original Phase 0 scope
  and is unchanged in spirit, just built out further (node registry, graph
  documents, rig topology builder).
- **MNA circuit engine** (`include/guitardsp/circuit/`, principally
  `MnaCircuitEngine.h` / `MnaCircuitEngineCore.h`) — a real Modified Nodal
  Analysis solver: R/C/L with trapezoidal companion models, potentiometers,
  ideal sources, all four controlled-source families, Shockley diodes,
  BJT/JFET/MOSFET, algebraic and dynamic op-amps, nonlinear **triode and
  pentode** stamps with Miller/Cgk/Cpk (and pentode Csk) parasitic subcircuits
  and positive-grid-current branches, transformer subcircuits with saturation,
  and switch/electromechanical-relay subcircuits — plus a fixed-pattern sparse
  solver path (`FixedPatternSparseSolver.h`) for realtime performance. See
  `docs/CIRCUIT_ENGINE_ARCHITECTURE.md` for the authoritative design doc,
  including what's still an "engineering model" rather than a
  measured/manufacturer-equivalent one.
- **TS808/DS-1 at component level** — `TS808Circuit.h` / `DS1Circuit.h` are
  hand-written C++ classes that build the real op-amp/BJT/diode pedal topology
  directly on `MnaCircuitEngine`. `data/circuits/ts808.json` / `ds1.json` are
  the *same* circuits transcribed op-for-op into a JSON netlist format, loaded
  via `NetlistLoader.h` (`guitardsp::circuit::NetlistCircuit`). See
  `docs/CIRCUIT_NETLIST_FORMAT.md` for the format spec and
  `tests/NetlistParityTests.cpp`, which asserts sample-by-sample agreement
  between each hand-written class and its JSON netlist. Do not confuse this
  format with `CircuitNetlist.h` below — different layer, different purpose.
- **`CircuitNetlist.h`** — a separate, in-memory, stable-ID schematic layer
  above the engine (stable node/component IDs independent of solver indices,
  compiling to a `CompiledCircuit`). It's the intended compile target for a
  future Circuit Mode UI. It is programmatic C++ today; JSON
  serialization/import of *this* layer (as opposed to the narrower
  TS808/DS-1-parity format above) is still future work — see "Next engine
  work" in `docs/CIRCUIT_ENGINE_ARCHITECTURE.md`.
- **Component-level amp stages exist and are netlist-ified, but the
  `include/guitardsp/hq/` amp/cabinet families are not.**
  `PreampCircuit.h`/`PowerAmpCircuit.h` (12AX7 common-cathode preamp +
  Bass/Treble tone stack; EL34 single-ended power stage with output
  transformer) are hand-written `guitardsp::circuit` classes built directly
  on `MnaCircuitEngine`, the same pattern as TS808/DS-1, with JSON netlist
  equivalents (`data/circuits/preamp.json`, `poweramp.json`) and parity tests
  in `tests/NetlistParityTests.cpp`. `FullAmpCircuit.h` cascades the two as
  independent engines rather than one combined netlist (see its header
  comment for why). See "New amp work" below for the policy this sets for
  future amp stages. This does *not* extend to `include/guitardsp/hq/`'s amp
  families (`ReferenceAmpTopologyNode.h`, `AmpFamilyNodes.h`,
  power-tube/tone-stack families, etc.) or cabinets (`CabinetChainNode.h`,
  speaker dynamics + measured-IR convolution) — those are still parameterized
  DSP-stage compositions, not MNA netlists, and re-modelling them at the
  component level has **not** started; see the "Amp/cabinet netlist status"
  section at the bottom of `docs/CIRCUIT_NETLIST_FORMAT.md`. Don't assume a
  netlist-ified equivalent of one of those `hq/` amp families exists just
  because PreampCircuit/PowerAmpCircuit/TS808/DS-1 have one.
- **JUCE standalone host** — `apps/GuitarDSPApp/Main.cpp`,
  `include/guitardsp/app/`, `src/app/` (`RealtimeAudioEngine`, `LiveRig`,
  performance monitor, cabinet IR loading). Built behind the
  `GUITARDSP_BUILD_AUDIO_APP` CMake option (CI builds it in a separate
  `build-audio-app` job); not built by default. See
  `docs/REALTIME_AUDIO_HOST.md`. This does **not** change the "keep the core
  JUCE-free" rule below — the adapter depends on the core, never the reverse.
- **`bench/`** — `MnaBenchmark.cpp` and `LiveRigBenchmark.cpp`, built behind
  `GUITARDSP_BUILD_BENCHMARKS` (CI enables this for the main build/test job).

When working in this repo, check what actually exists in `include/`, `src/`,
`apps/`, `bench/`, `data/` and `tests/` rather than trusting a summary like
this one at face value — this file has gone stale before as the codebase grew
past what it described; re-verify claims about what is/isn't implemented
against the code before relying on them.

## Architecture

- `include/guitardsp/graph/AudioNode.h` — `AudioNode` interface (`prepare`,
  `reset`, `process`, `latencySamples`) and the `ProcessContext`/`AudioBlock`
  value types used by the real-time audio path.
- `include/guitardsp/graph/Graph.h` / `src/graph/Graph.cpp` — the `Graph`
  class: owns node instances, connections, topological scheduling, cycle
  detection and cumulative-latency analysis. `Graph::compile()` validates and
  builds the execution plan.
- `include/guitardsp/circuit/` — the MNA circuit engine and its subcircuit
  helpers (`MnaCircuitEngine.h`, `MnaCircuitEngineCore.h`,
  `TriodeParasiticSubcircuit.h`, `PentodeParasiticSubcircuit.h`,
  `TransformerSubcircuit.h`, `CircuitUpdateQueue.h` for realtime component
  edits, `NetlistLoader.h`, `CircuitNetlist.h`, `TS808Circuit.h`,
  `DS1Circuit.h`, `OperatingPointContinuation.h` for DC-priming continuation).
  `docs/CIRCUIT_ENGINE_ARCHITECTURE.md` and `docs/CIRCUIT_NETLIST_FORMAT.md`
  are the authoritative design docs for this layer — read them before adding
  a new circuit element or pedal.
- `include/guitardsp/hq/` — higher-quality DSP nodes: amp/cabinet families,
  pedal topology nodes, oversampling/ADAA/measurement infrastructure. See
  `docs/HQ_DSP.md`.
- `apps/GuitarDSPApp/`, `include/guitardsp/app/`, `src/app/` — the optional
  JUCE standalone host adapter (`GUITARDSP_BUILD_AUDIO_APP`). See
  `docs/REALTIME_AUDIO_HOST.md`.
- `bench/` — benchmark executables (`GUITARDSP_BUILD_BENCHMARKS`).
- `tests/GraphTests.cpp` and the rest of `tests/` — unit tests for the graph
  core, circuit engine, HQ nodes and app layer, run via CTest.
- `docs/ARCHITECTURE.md` — the authoritative design document: connection
  semantics (split/merge/tap/send-return), the real-time contract, node
  families, quality/oversampling policy, and the full phase roadmap. Read this
  before making architectural changes.

**Key constraint — keep the core JUCE-free.** `GuitarDSPGraphCore` (everything
under `include/guitardsp/` and `src/`, including the circuit engine and HQ
nodes) is intentionally free of JUCE and any other heavyweight dependency so
it stays portable and unit-testable without a GUI or audio device, and so it
can eventually be ported to embedded targets (SHARC/PIC). The JUCE host shell
now lives in `apps/GuitarDSPApp/` as a separate adapter layer that depends on
the core — never the other way around; do not let JUCE types leak back into
`include/guitardsp/` or `src/graph/`, `src/app/` etc. that are part of
`GuitarDSPGraphCore`. Be careful about introducing *any* new third-party
dependency into the core — prefer the standard library.

**Real-time contract.** The audio callback path (`AudioNode::process`,
`MnaCircuitEngine::processSample`, `NetlistCircuit::processSample` and
anything reachable from them) must not allocate, lock, touch the filesystem,
parse JSON, or rebuild graph/circuit topology. Graph and circuit edits happen
off the audio thread (e.g. `CircuitUpdateQueue` for topology-preserving
component edits, `NetlistLoader`'s `loadFromFile`/`loadFromJson`/`prepare` for
loading a netlist); a compiled, immutable execution plan/prepared circuit is
swapped in at a safe block boundary. Keep this invariant in mind for any
change inside `src/graph/`, `include/guitardsp/circuit/` or new DSP nodes.

## Nonlinear solver design (read before adding a new nonlinear circuit element)

`MnaCircuitEngineCore::processSample()` (in
`include/guitardsp/circuit/MnaCircuitEngineCore.h`) globalizes each Newton
step with a **backtracking line search**, not a fixed damping schedule.

Why this matters for anyone adding a new nonlinear device stamp: an earlier
version used a fixed damping factor (0.65 for the first 3 iterations, then
0.85). That made the damped update a deterministic, time-invariant map, and
around a high-gain feedback stage that map could have a locally repelling
fixed point — so the Newton iterate could settle into an exact repeating
limit cycle no matter how many iterations were allowed. This was reported as
DS-1 silent-input hiss (issue #14) and later found to be a structural
property of fixed damping itself, not specific to any one circuit (issue
#16).

The current design instead:

- Computes a candidate full Newton step, then **halves the step (up to
  `kMaxLineSearchBacktracks = 6` times)** until the trial state's residual
  norm is non-increasing relative to the current candidate's. Because every
  accepted step never increases the merit value, no state can exactly recur —
  this rules out limit cycles structurally rather than by detecting one after
  the fact. "Non-increasing" (not strictly decreasing) is deliberate: once a
  circuit is within float precision of its operating point there's nothing
  left to shrink, and a strict `<` would reject every alpha on noise alone.
- Applies a **single shared scalar trust-region clamp** per iteration
  (computed once across all node unknowns), rather than an independent
  per-node clamp — an independent per-node clamp can break an exact linear
  identity between two coupled unknowns (e.g. a floating voltage source's
  `V_pos - V_neg = V_source` row, or a subcircuit's zero-volt current-sense
  source) until the solve fully converges. The clamp's ceiling was raised
  from 25 V to 60 V (issue #27) so pentode/beam-tetrode power stages with
  400–800 V plate/screen rails still reach their operating point within a
  handful of cold-start samples; low-voltage diode/BJT/JFET junctions are
  unaffected since the clamp only engages above ~100 V candidates.
- Falls back to the old fixed-damping step **only** when no backtrack finds a
  non-increasing residual at all — i.e. the Newton direction genuinely wasn't
  a descent direction (e.g. a JFET/MOSFET candidate sitting exactly on its
  pinch-off/threshold kink, where the model's derivative collapses near
  zero). This keeps that corner case exactly as robust as before line search
  existed, without weakening the general limit-cycle guarantee.

**Implication for new nonlinear elements:** don't add a bespoke fixed-damping
schedule for a new device stamp — that's the exact class of bug this
mechanism was built to eliminate structurally, and a local damping hack can
silently reintroduce a limit cycle around a different feedback topology. New
nonlinear stamps should rely on the shared line-search/trust-region machinery
already in `MnaCircuitEngineCore.h`; if a new device needs special handling,
extend that shared mechanism rather than special-casing damping per element.
`OperatingPointContinuation.h` (source-stepping DC priming before audio-rate
solving) is the separate mechanism for getting a circuit to a plausible
starting operating point in the first place — it's not a substitute for the
per-sample line search.

## Build, test

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

CI (`.github/workflows/ci.yml`) additionally configures with
`-DGUITARDSP_GRAPH_WARNINGS_AS_ERRORS=ON -DGUITARDSP_BUILD_BENCHMARKS=ON`;
consider building the same way locally before pushing:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DGUITARDSP_GRAPH_WARNINGS_AS_ERRORS=ON -DGUITARDSP_BUILD_BENCHMARKS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

CI also has a separate `build-audio-app` job that configures with
`-DGUITARDSP_BUILD_AUDIO_APP=ON -DGUITARDSP_FETCH_JUCE=ON` and builds the
`GuitarDSPApp` target; it's not part of the default configure above since it
pulls in JUCE. Use the same flags locally (or `-DGUITARDSP_JUCE_PATH=...` to
point at an existing JUCE checkout instead of fetching) if you're touching
`apps/GuitarDSPApp/`, `include/guitardsp/app/` or `src/app/`.

### Loading a JSON circuit netlist

TS808/DS-1 can be driven from their JSON netlists (`data/circuits/ts808.json`,
`ds1.json`) instead of the hand-written C++ classes, via
`include/guitardsp/circuit/NetlistLoader.h`:

```cpp
#include "guitardsp/circuit/NetlistLoader.h"

guitardsp::circuit::NetlistCircuit circuit;
std::string error;
if (!circuit.loadFromFile("data/circuits/ts808.json", &error)) { /* handle error */ }
if (!circuit.prepare(48000.0, &error)) { /* handle error */ }
circuit.setControl("drive", 0.6f);
const float y = circuit.processSample(x); // real-time safe from here on
```

`loadFromFile`/`loadFromJson`/`prepare` are control-thread-only, same as any
other circuit's `prepare()` — see the real-time contract above. Full format
spec, op list and the `preset`/`spec` device-override mechanism are in
`docs/CIRCUIT_NETLIST_FORMAT.md`. `tests/NetlistParityTests.cpp` is the
regression that keeps a netlist and its hand-written equivalent numerically
identical; if you add or edit a netlist op, keep both in mind. Note this
format is a narrow, pedal-parity-focused schema — it is not the general
`CircuitNetlist.h` schematic layer (see "Current state" above).

## New amp work: MNA-first policy

(Established in issue #43, superseding an earlier scoping call in issue #24
that treated amp netlist-ification as out of scope — at that point the only
amp models in the repo were `include/guitardsp/hq/`'s parameterized DSP-stage
compositions, which genuinely aren't representable in the netlist format.
`PreampCircuit`/`PowerAmpCircuit` didn't exist yet.)

**New amp stages should be implemented as component-level MNA circuits
following the `PreampCircuit`/`PowerAmpCircuit` pattern** — a hand-written
`guitardsp::circuit` class built directly on `MnaCircuitEngine` and its
subcircuit helpers (`TriodeParasiticSubcircuit.h`,
`PentodeParasiticSubcircuit.h`, `TransformerSubcircuit.h`, etc.), the same
discipline TS808/DS-1 already follow — **with a JSON netlist
(`data/circuits/*.json`) and a `tests/NetlistParityTests.cpp` parity check
added in the same change**, not deferred as follow-up work. See
`docs/CIRCUIT_NETLIST_FORMAT.md` for the `triodeParasitic`/
`pentodeParasitic`/`transformer` ops this requires.

This does not retroactively require netlist-ifying (or otherwise touching)
the existing behavioral `include/guitardsp/hq/` amp families
(`ReferenceAmpTopologyNode.h`, `AmpFamilyNodes.h`, etc.) — re-modelling those
at the component level is a separate, larger effort and out of scope here.
A multi-stage amp that cascades independently-prepared circuits (see
`FullAmpCircuit.h`) does not need one combined netlist either: cascading two
separately-loaded `NetlistCircuit`s reproduces the same signal path (see
"Amp/cabinet netlist status" in `docs/CIRCUIT_NETLIST_FORMAT.md`).

## Development rules

- When resolving merge conflicts, respect the intent of each side's change.
  If the correct resolution isn't clear, stop and report back via a GitHub
  issue rather than guessing.
- Never force-push or rewrite published history.
- Treat `.github/workflows/` (CI and Actions permissions) with extra care —
  changes there need a human review before merging; do not modify Actions
  permissions/secrets unilaterally.
- Prefer small, reviewable PRs that keep `ctest` green.
