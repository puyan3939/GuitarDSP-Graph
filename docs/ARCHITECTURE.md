# GuitarDSP-Graph Architecture

## Core principle

The graph is the product. Artist rigs, conventional pedalboards, studio chains and experimental signal systems are presets built on top of the same graph engine.

The old fixed-chain model is intentionally not copied. Existing GuitarDSP-Pro DSP algorithms will be migrated as reusable nodes only after their signal-flow assumptions are removed.

## Graph model

A graph contains node instances and directed connections. Nodes do not know where they live in the graph.

Planned connection semantics:

- normal edge: audio continues to the next node
- split: one output fans out to multiple downstream nodes
- merge: multiple upstream blocks are summed with per-edge gain/polarity/delay
- tap: copies a signal without interrupting the main path
- send/return: explicit loop semantics for external or internal FX chains
- output: routes a graph signal to a logical/physical output bus

The graph compiler must reject cycles unless a future explicit FeedbackNode owns the delay required to make the feedback path causal.

## Real-time contract

The audio callback must not allocate, lock, access files, parse JSON, rebuild topology or perform unbounded work.

Graph editing occurs off the audio thread. A compiled immutable execution plan is swapped into the audio engine at a safe block boundary.

Phase 0 currently implements topology validation, deterministic topological scheduling and cumulative latency analysis. Audio-buffer scheduling comes next.

## Latency compensation

Each node declares algorithmic latency. During compilation the engine computes cumulative latency for every path.

At every merge, shorter branches will receive automatically generated compensation delays so all inputs arrive sample-aligned. Users will still have a manual fine-delay and polarity control because two physically modelled paths can sound better intentionally misaligned.

Future latency sources include:

- oversampling filters
- pitch shifting
- lookahead dynamics
- convolution/IR partitioning
- linear-phase filtering
- external plugin/loop nodes

## Node instance policy

There are no global singleton pedals or amps. If a graph needs two DS-1 nodes, three delays or four amp nodes, they are independent instances with independent state.

This is required for real rigs where the same device family can appear multiple times or parallel branches need independent history.

## DSP node families

### Utility

Input, Output, Gain, Pan, Polarity, Delay, Split, Merge, Tap, Send, Return, Mixer, Mid/Side.

### Dynamics

Input gate, keyed post gate, expander, studio compressor, guitar compressor, transient shaper, multiband compressor, dynamic EQ.

### Drive / preamp

Models should be topology-specific where the circuit behavior matters. Generic models remain useful, but signature circuits should not be reduced to one waveshaper.

Initial targets: TS-style, DS-1-style, BD-2-style, RAT-style, Muff-style, Klon-style, BB-style preamp, Schaffer-style preamp, Fuzz Factory-style, NG-3-style.

### Pitch

Separate algorithms and controls for:

- polyphonic octave generation (POG-style)
- monophonic/bass octave behavior (OC-style)
- whole-signal polyphonic transpose (Drop-style)
- continuous expression pitch (Whammy-style)

### Amplifier

Amp nodes are component-aware systems rather than arbitrary serial waveshapers. A typical valve amp model can expose internally meaningful blocks such as input loading, triode stages, coupling networks, tone stack, recovery stage, phase inverter, power stage, negative feedback, presence/resonance and transformer behavior.

### Cabinet / speaker

Measured factory/user IR remains the baseline. Future cabinet nodes can combine dual/multi IR, microphone blending, phase alignment, room/early-reflection paths and nonlinear speaker dynamics.

## Quality architecture

Each nonlinear node declares a quality requirement rather than inheriting a global fixed oversampling factor.

Example policy:

- utility/EQ/delay: 1x
- compressor: 1x or 2x
- mild OD: 4x-8x
- hard clipping/fuzz: 8x-16x
- amp nonlinear core: 8x-16x
- pitch core: algorithm-specific, potentially 16x

Studio mode may raise these ceilings. Live mode prioritizes latency and CPU stability.

High-quality nonlinear work should evaluate:

- oversampling filter stopband and transition quality
- ADAA where mathematically appropriate
- circuit-specific diode/transistor/triode behavior
- alias-energy measurement in CI/measurement tests
- DC and denormal stability
- parameter smoothing at every sensitive control boundary

## Measurement is a first-class subsystem

Any node output should be tap-able by the analyzer without altering the audible graph.

Target measurements:

- waveform
- FFT/spectrum
- THD and harmonic distribution
- alias-energy estimate
- RMS/peak/crest factor
- LUFS where appropriate
- phase/correlation
- node and graph latency
- compressor/gate gain reduction

The analyzer and signal generator are intended to make GuitarDSP-Graph useful as both an instrument processor and a DSP development laboratory.

## Presets and scenes

A preset stores the complete graph: node types, node parameters, connections, mixer settings, quality policy and output routing.

A scene/snapshot stores a performance state over the graph. It may change parameters, bypass states, edge gains and eventually topology. Topology-changing scenes should crossfade compiled plans to avoid clicks.

## Migration from GuitarDSP-Pro

Do not port the old SignalChain wholesale.

Migration order:

1. common DSP primitives and measurement helpers
2. Factory/User IR engine
3. latest dynamics
4. high-quality pitch cores
5. selected pedal models after node-state isolation review
6. HQ amp building blocks
7. analyzer UI/data path
8. preset conversion tools

Every migrated block gets standalone tests before becoming a graph node.

## Phase roadmap

### Phase 0 — graph kernel

Topology, validation, deterministic schedule, latency analysis, CI.

### Phase 1 — real-time audio plan

Preallocated per-node buffers, split/merge/tap utility nodes, automatic compensation delay insertion, immutable compiled-plan swap.

### Phase 2 — JUCE host/app shell

Audio device, graph editor, node inspector, preset serialization and analyzer window.

### Phase 3 — high-value DSP migration

Factory IR, dynamics, pitch, current HQ amp blocks and selected existing pedals.

### Phase 4 — circuit-specific expansion

DS-1/BD-2/TS/NG-3/Fuzz Factory families, component-aware amp families, Drop/POG/Whammy separation.

### Phase 5 — studio/performance system

Multi-output matrix, snapshots, MIDI/expression, looper/sampler, advanced delay/modulation and measurement automation.
