# HQ DSP foundation acceptance criteria

This document defines what "solid" means for the reusable high-quality DSP layer before measured pedal and amplifier models are built on top of it.

## 1. Resampling / oversampling

- Nonlinear nodes select oversampling from the central quality policy.
- Production nonlinear path uses polyphase interpolation rather than explicit zero stuffing plus a full FIR per high-rate sample.
- No allocation in the audio callback.
- DC gain after warm-up must remain within ±0.25 dB.
- Passband and stopband measurements will be added as automated regression metrics.
- Every oversampled node reports algorithmic latency to graph PDC.
- Eco / Live / High / Studio remain policy decisions, not hard-coded inside individual pedal models.

## 2. SIMD

- Portable scalar behavior is the reference.
- x86 SSE2 and ARM NEON paths must match scalar semantics within floating-point tolerance.
- DSP correctness must never depend on SIMD availability.
- SIMD is used only after the scalar numerical contract is covered by tests.

## 3. FFT / alias measurement

- Radix-2 forward/inverse FFT round-trip is regression tested.
- Parseval scaling is regression tested.
- Alias analysis separates expected harmonic bins from residual non-harmonic energy for periodic test tones.
- Pedal and amplifier CI can later fail on alias-energy regressions, not only on crashes or NaNs.

## 4. Cabinet convolution

- Direct FIR remains the short-IR correctness reference.
- Long IR uses uniform partitioned FFT convolution.
- Partitioned convolution must numerically match direct convolution within a defined tolerance on deterministic signals.
- IR preparation and FFT partition creation happen off the realtime thread.
- Audio processing performs no allocation.
- Future production node will add streaming adaptation for arbitrary host block sizes and stereo/multi-IR blending.

## 5. Device models

- Diode, BJT and triode component data are separate from pedal/amp topology.
- Diode pair solver supports asymmetric device choices.
- BJT and triode stage models include dynamic bias memory rather than only a static transfer function.
- Named devices (12AX7, 12AT7, silicon, germanium, LED, transistor presets) are parameter presets, not separate ad-hoc nonlinear equations.
- Hardware-equivalent claims require measured fitting. Until then these models are reference circuit-inspired components.

## Release gate before measured pedal/amp work

The HQ foundation is considered ready for measured model development when:

1. All graph and HQ CI tests pass with warnings-as-errors.
2. FFT, convolution and resampling numerical regression tests pass.
3. Alias residual measurement is available to model-specific CI.
4. Production nonlinear reference node uses the polyphase path.
5. Device-stage solvers remain finite under stress tests.
6. Realtime paths are allocation-free after prepare/build.

After this gate, the next layer is measured topology work: DS-1, TS808, BD-2, fuzz circuits, then reusable preamp / phase-inverter / power-amp / transformer blocks for amplifier models.
