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

**Current state of the code (important):** the repository is still early —
Phase 0 of the roadmap below. Today it contains only the directed audio graph
kernel (topology, validation, deterministic scheduling, latency analysis) as a
dependency-free C++ library. There is **no JUCE integration, no MNA circuit
engine, and no circuit/netlist data files yet** — those belong to later
phases. When working in this repo, check what actually exists in `include/`,
`src/` and `tests/` rather than assuming later-phase pieces (amp/cabinet
simulation, netlist JSON loading, benchmarks) are already implemented.

## Architecture

- `include/guitardsp/graph/AudioNode.h` — `AudioNode` interface (`prepare`,
  `reset`, `process`, `latencySamples`) and the `ProcessContext`/`AudioBlock`
  value types used by the real-time audio path.
- `include/guitardsp/graph/Graph.h` / `src/graph/Graph.cpp` — the `Graph`
  class: owns node instances, connections, topological scheduling, cycle
  detection and cumulative-latency analysis. `Graph::compile()` validates and
  builds the execution plan.
- `tests/GraphTests.cpp` — unit tests for the graph core, run via CTest.
- `docs/ARCHITECTURE.md` — the authoritative design document: connection
  semantics (split/merge/tap/send-return), the real-time contract, node
  families, quality/oversampling policy, and the full phase roadmap. Read this
  before making architectural changes.

**Key constraint — keep the core JUCE-free.** `GuitarDSPGraphCore` (everything
under `include/guitardsp/` and `src/`) is intentionally free of JUCE and any
other heavyweight dependency so it stays portable and unit-testable without a
GUI or audio device, and so it can eventually be ported to embedded targets
(SHARC/PIC). When a JUCE host/app shell is added (Phase 2 of the roadmap in
`docs/ARCHITECTURE.md`), it must live in a separate adapter layer that depends
on the core, never the other way around. Be careful about introducing *any*
new third-party dependency into the core — prefer the standard library.

**Real-time contract.** The audio callback path (`AudioNode::process` and
anything reachable from it) must not allocate, lock, touch the filesystem,
parse JSON, or rebuild graph topology. Graph edits happen off the audio
thread; a compiled, immutable execution plan is swapped in at a safe block
boundary. Keep this invariant in mind for any change inside `src/graph/` or
new DSP nodes.

## Build, test

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

CI (`.github/workflows/ci.yml`) additionally configures with
`-DGUITARDSP_GRAPH_WARNINGS_AS_ERRORS=ON`; consider building the same way
locally before pushing:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DGUITARDSP_GRAPH_WARNINGS_AS_ERRORS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

There is no `bench/` directory yet — benchmark targets referenced in future
plans (e.g. live-rig or MNA-solver benchmarks) do not exist in the codebase
today. If you add benchmarks, wire them into CMake and document how to run
them here.

## Development rules

- When resolving merge conflicts, respect the intent of each side's change.
  If the correct resolution isn't clear, stop and report back via a GitHub
  issue rather than guessing.
- Never force-push or rewrite published history.
- Treat `.github/workflows/` (CI and Actions permissions) with extra care —
  changes there need a human review before merging; do not modify Actions
  permissions/secrets unilaterally.
- Prefer small, reviewable PRs that keep `ctest` green.
