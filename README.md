# GuitarDSP-Graph

Next-generation graph-based guitar DSP engine.

The goal is not to clone a particular artist rig. The goal is to model real signal systems deeply enough that simple rigs, complex multi-amp rigs, direct/DI taps, pitch branches, studio chains, and experimental DSP can all be built from the same engine.

## Phase 0 goals

- Directed audio graph instead of a fixed MAIN/CLEAN/SUB chain
- Reusable node instances with explicit input/output ports
- Split, merge, tap and output nodes as first-class graph elements
- Deterministic topological scheduling
- Graph validation (duplicate IDs, missing sources, cycles)
- Per-node latency declaration and graph latency analysis
- No heap allocation inside the audio processing pass
- Pure C++ graph core so it can be unit-tested without a GUI or audio device
- JUCE adapter layer added after the graph core is stable

## Planned node families

- Utility: input, output, gain, polarity, delay, split, merge, tap, send/return
- Dynamics: gate, expander, studio compressor, guitar compressor, transient shaper, dynamic EQ
- Drive: TS-style, DS-1-style, BD-2-style, RAT-style, Klon-style, Muff-style, Fuzz Factory-style, NG-3-style, preamps
- Pitch: POG-style poly-octave, OC-style octave, Drop-style polyphonic transpose, Whammy-style expression pitch
- Amp: Plexi/JCM/DSL families, Fender clean families, bass amp families, extensible component-aware amp nodes
- Cabinet: factory/user IR, multi-IR, mic blend, phase alignment, speaker dynamics
- Time/mod: multi-delay, Eventide-style routing, modulation, reverb
- Performance: scenes/snapshots, expression mapping, MIDI/control, multi-output matrix
- Measurement: waveform, FFT, THD, harmonic spectrum, alias energy, phase, latency, gain reduction

## Quality philosophy

Nonlinear nodes should own their quality policy. Clean utility nodes stay at 1x where possible; clipping, fuzz, amp and pitch nodes can request higher internal rates. The long-term target is adaptive oversampling, high-quality resampling filters, optional ADAA for suitable nonlinearities, latency-aware parallel routing, and explicit quality modes for live vs studio use.

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Phase 0 intentionally has no JUCE dependency.
