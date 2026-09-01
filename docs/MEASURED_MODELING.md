# Measured modeling workflow

The HQ topology nodes are circuit-inspired reference models until they are fitted to repeatable measurements. This document defines the path from a topology model to a hardware-equivalent claim.

## Measurement layers

Use multiple independent measurement layers. A single frequency sweep is not enough for a nonlinear device.

1. **Small-signal frequency response** at several control positions and at a level below obvious clipping.
2. **Level-dependent frequency response** to expose nonlinear loading and compression.
3. **Steady-state harmonic spectrum / THD** for several sine frequencies and input levels.
4. **Transfer behaviour** using controlled sine or stepped-envelope stimuli. Avoid interpreting DC sweeps through AC-coupled pedals as a static transfer curve.
5. **Dynamic response**: attack, recovery, bias shift, sag, gating/starvation and cleanup behaviour.
6. **Noise floor** with the input terminated by a documented impedance.
7. **Latency and phase** where the model contains lookahead, oversampling, pitch or convolution stages.

## Reference-point format

The initial in-code fit API accepts weighted points:

```text
frequency_hz, magnitude_db, weight
100,          -2.8,         0.5
250,          -0.7,         1.0
1000,          2.1,         1.5
4000,         -4.2,         1.0
8000,        -10.5,         0.8
```

Transfer references use:

```text
input, output, weight
-0.50, -0.43, 1.0
-0.25, -0.28, 1.0
 0.00,  0.00, 1.0
 0.25,  0.31, 1.0
 0.50,  0.46, 1.0
```

The repository currently provides `MeasuredFit.h` for weighted error metrics and `OfflineModelEvaluator.h` for deterministic sine sweeps and small grid searches. A future dataset loader should preserve measurement metadata rather than reducing captures to anonymous arrays.

## Metadata that must accompany captures

Record at minimum:

- exact hardware revision / serial or board revision when known
- power-supply voltage
- source impedance and input termination
- output load impedance
- audio-interface model and gain settings
- sample rate and bit depth
- control positions
- bypass reference level
- stimulus amplitude in Vrms or dBu, not only normalized digital level
- ambient/noise-floor notes when relevant

Without this metadata a fit can still be useful, but it should not be described as a hardware-equivalent validation.

## Fitting order

Fit in a physically meaningful order to avoid compensating one wrong block with another:

```text
Input/loading and coupling
-> linear frequency shaping
-> gain-stage operating point
-> nonlinear device parameters
-> tone/output network
-> dynamic memory (bias/sag/starve)
-> final level
```

For an amplifier, expand this to preamp stage by stage, then tone stack, phase inverter, power stage, negative feedback and transformer/speaker interaction.

## Acceptance criteria

A named hardware model should eventually have explicit pass/fail limits for each control region. Suggested starting targets are not product claims; they are engineering gates to refine with real data:

- small-signal response weighted RMS error: <= 1.0 dB across the guitar band
- important-band max response error: <= 2.5 dB
- steady sine harmonic-level error: <= 3 dB for dominant harmonics
- no NaN/Inf or solver divergence over the tested input range
- alias residual must not regress against the previous accepted model at equal quality mode
- control sweeps must remain smooth unless the hardware measurement demonstrates a discontinuity

## CI strategy

Keep small synthetic datasets in unit tests to validate the fitting machinery itself. Store real hardware-derived reference datasets separately with provenance, then run deterministic regression checks against them. CI should fail when a model exceeds its accepted fit envelope, not merely when output samples differ bit-for-bit after a legitimate algorithm improvement.
