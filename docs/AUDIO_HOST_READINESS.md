# Audio-host readiness gates

Before wiring JUCE audio I/O into `GuitarDSP-Graph`, three engine-level gates are treated as quality requirements rather than deferred cleanup.

## 1. Nonlinear operating-point continuation

`circuit/OperatingPointContinuation.h` standardizes control-thread startup for nonlinear circuits. Independent supplies are source-stepped from zero with the dense partial-pivot MNA solver, then selected probe nodes are allowed to settle until their voltages stop moving. The resulting nonlinear solution and dynamic companion history are retained as the initial condition for realtime processing.

This replaces the idea that every future pedal or amp should invent its own startup ramp. Existing TS808 and DS-1 circuit classes already use the same source-stepping principle internally; new circuit implementations can use the shared helper directly and those existing private primers can be migrated without changing their audio topology.

Accuracy boundary: this is a time-domain/quasi-DC continuation. `MnaCircuitEngine` currently keeps trapezoidal capacitor/inductor companions active, so it is not yet a separate SPICE-style static DC matrix where capacitors are analytically open circuits and inductors are shorts. The distinction is intentional and documented rather than hidden.

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
