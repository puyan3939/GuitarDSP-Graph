# Audio-host readiness gates

Before wiring JUCE audio I/O into `GuitarDSP-Graph`, three engine-level gates are treated as quality requirements rather than deferred cleanup.

## 1. Nonlinear operating-point continuation

`circuit/OperatingPointContinuation.h` standardizes control-thread startup for nonlinear circuits, and provides two related mechanisms:

- `establishOperatingPoint()`: independent supplies are source-stepped from zero under the engine's normal trapezoidal companion model, then selected probe nodes are allowed to settle until their windowed DC means stop moving. This is a time-domain/quasi-DC continuation, not an analytic solve -- it is still watching an exponentially decaying transient settle, just through a windowed mean that rejects trapezoidal ringing.
- `establishDcOperatingPoint()`: independent supplies are source-stepped from zero under `MnaCircuitEngine::setOperatingPointMode(true)` -- a genuine SPICE-style static DC matrix where every capacitor is stamped as an open circuit and every inductor as its bare series resistance, with no trapezoidal history term at all. Because that system has no memory, the homotopy's own Newton convergence at the final source step *is* the operating point; there is no settling time to wait out, regardless of a capacitor's real RC time constant. The resulting node voltages become each capacitor/inductor's initial `previousVoltage`/`previousCurrent` history for realtime trapezoidal processing to continue from.

This replaces the idea that every future pedal or amp should invent its own startup ramp. `TS808Circuit`, `DS1Circuit`, `PreampCircuit`, `PowerAmpCircuit`, `CompressorCircuit` and the data-driven `NetlistCircuit` (`NetlistLoader.h`) all call `establishDcOperatingPoint()` from `prepare()`; new circuit implementations should do the same rather than inventing a bespoke private primer.

## 2. Realtime execution audit

Prepared audio processing must not allocate heap memory. `CompiledAudioGraph` now reserves its callback input/output pointer tables during build so even the first audio callback does not allocate. `AudioReadinessTests` arms a global allocation counter only around prepared realtime work and checks both:

- a nonlinear MNA circuit over hundreds of audio-rate source updates and Newton solves;
- the first `CompiledAudioGraph` process callback, including an oversampled branch and PDC.

The audit is structural: setup, source stepping, graph building, IR loading, topology compilation and vector sizing remain control-thread work. The audio path is expected to reuse prepared storage only.

Matrix-affecting parameter automation can still trigger a numeric static-matrix rebuild. That rebuild is preallocated and allocation-free, but it can be computationally heavier than steady-state processing. Future UI/automation work should therefore deliver circuit coefficient changes at a bounded control/block rate rather than rebuilding a large MNA matrix for every oversampled sample.

## 3. Oversampling latency and PDC

The polyphase oversampler uses the same odd linear-phase FIR for interpolation and decimation. The two FIRs have a combined high-rate group delay of `taps - 1`. Decimation is now performed on the phase congruent with that combined delay:

`decimationPhase = (taps - 1) mod factor`

so the effective base-rate round-trip impulse response is centered on an integer sample. `latencySamples()` therefore reports the actual aligned round-trip delay rather than the previous one-filter approximation.

`AudioReadinessTests` verifies the reported delay against the measured identity impulse peak for 2x, 4x, 8x and 16x configurations.

A second PDC issue was also fixed: `CompiledAudioGraph` previously compiled cumulative graph latency before calling each node's `prepare()`. Any quality-dependent node, including oversampling nodes, could therefore report zero/stale latency during PDC construction. Graph build now performs topology validation first, prepares nodes, recompiles latency, and only then creates edge/sink compensation delays. An impulse regression test verifies that a direct branch and an 8x oversampled branch line up at the same sample.

## Circuit-pedal quality policy

`TS808CircuitNode` and `DS1CircuitNode` now oversample the complete component-level MNA circuit, not merely a final waveshaper. Their internal circuit sample rate follows `QualityPolicy`:

- Eco: 2x nonlinear processing
- Live: 4x
- High: 8x
- Studio: 16x

The circuit is prepared at `hostSampleRate * oversamplingFactor`, and the resampler latency is exposed through the normal graph node latency contract so PDC can align parallel dry/direct/amp paths.

## Gate to the JUCE host

The audio-host phase should begin only after the complete CI suite is green with these tests enabled. At that point the next layer can focus on device I/O, buffer sizes, XRUN monitoring, input/output trim and graph hot-swap behavior rather than discovering basic solver-startup or latency-contract defects while listening.
