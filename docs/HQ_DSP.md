# High-Quality DSP Foundation

This layer is intentionally separate from graph topology. The graph decides *where* processing happens; HQ DSP primitives decide *how accurately* nonlinear and bandwidth-expanding processing is performed.

## Quality policy

`ProcessingQuality` maps nonlinear nodes to progressively more expensive processing:

| Mode | Drive/Amp oversampling | Resampler taps |
|---|---:|---:|
| Eco | 2x | 23 |
| Live | 4x | 31 |
| High | 8x | 47 |
| Studio | 16x | 63 |

Linear utility nodes remain at base rate unless their algorithm explicitly requires otherwise. This prevents wasting CPU on operations that cannot create aliases.

## Oversampling

The first reference implementation uses an odd-length Blackman-windowed sinc low-pass and preallocated work/state buffers. It is deliberately simple and deterministic so later polyphase/SIMD implementations can be checked against it. No allocation occurs inside `process()`.

Future production work:
- polyphase implementation to avoid zero-stuff multiply cost
- SIMD kernels
- measured passband/stopband CI thresholds
- minimum-latency half-band option for Live mode
- optional higher-tap Studio filters

## ADAA

First-order antiderivative anti-aliasing is available for tanh and polynomial soft clipping. ADAA complements oversampling; it does not replace proper band-limiting. The intended path for hard nonlinear nodes is oversampling + ADAA where the transfer permits a stable antiderivative.

## Circuit primitives

The HQ layer now separates mechanisms instead of representing an amplifier as repeated generic `gain -> tanh -> low-pass` blocks.

Current reusable primitives:
- implicit antiparallel diode solver using Newton iterations
- dynamic asymmetric triode stage with independent bias-memory and sag terms
- low-frequency-sensitive transformer saturation
- one-pole state blocks for coupling/envelope mechanisms

These are architecture primitives, not claims of exact named hardware. Named pedal/amp models must later be fitted against schematics, component values, SPICE/reference data, and/or measurements.

## Measurement

Offline harmonic helpers provide a CI-visible baseline for:
- fundamental amplitude
- THD
- upper-band energy proxy

The next measurement layer should add FFT-based alias energy, swept-sine transfer plots, multitone tests, null tests against reference implementations, latency, CPU cost, and denormal/NaN stress tests.

## Rules for production models

1. Do not increase oversampling globally just because one node is nonlinear.
2. Every nonlinear model declares its quality requirement.
3. Every oversampled node reports latency to the graph PDC system.
4. Circuit-specific models should expose physical mechanisms internally even if the UI remains musician-friendly.
5. Reference and optimized implementations should coexist long enough for numerical A/B tests.
6. Silence, tiny-signal gain, DC drift, alias energy, and parameter-edge stability belong in CI.
