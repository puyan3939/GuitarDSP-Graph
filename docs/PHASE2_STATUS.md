# Phase 2 HQ DSP status

Current branch: `feature/phase2-hq-dsp`

## Foundation complete enough for topology work

- centralized Eco / Live / High / Studio quality policy
- polyphase oversampling through 16x
- ADAA nonlinear primitives
- implicit diode solvers
- SSE2 / NEON / scalar SIMD helpers
- radix-2 FFT and alias-residual analysis
- uniform partitioned convolution and streaming host-block adapter
- graph-ready partitioned cabinet node with deterministic PDC latency
- reusable diode, BJT, triode, cathode/emitter-memory, sag and transformer primitives
- weighted measured-fit metrics
- deterministic offline sine-sweep evaluator
- 1-D parameter grid fitting for initial model calibration
- deterministic multi-parameter coordinate-grid fitting for bounded offline calibration
- reusable EL34 / 6L6GC / KT88 power-tube family transfer primitives
- nonlinear speaker-dynamics layer for voice-coil compression, excursion and LF resonance

## Topology nodes

- HQ DS-1 topology
- HQ TS808 topology
- HQ BD-2 topology reference
- reusable two-transistor feedback fuzz topology
- generic reference amplifier topology
- HQ speaker dynamics node

The HQ nodes are registered in `NodeRegistry`, so graph documents can instantiate them by stable type IDs instead of requiring hard-coded construction.

## Amplifier building blocks

The reference amp now exercises reusable contracts for:

```text
input coupling
-> common-cathode triode V1
-> interstage coupling
-> common-cathode triode V2
-> three-band tone stack abstraction
-> differential phase inverter
-> negative feedback
-> selectable push-pull EL34 / 6L6GC / KT88 power stage with sag/crossover
-> transformer saturation
```

The selectable power-tube family is exposed as an amp parameter and is processed inside the oversampled power stage. EL34, 6L6GC and KT88 currently provide distinct engineering transfer behavior; they are not measured tube fits yet.

This remains an unnamed engineering reference. Named Marshall/Fender/Ampeg models require topology-specific networks and measured fitting.

## Current validation

The CI suite covers graph runtime, PDC, node registry, hot swap, multi-output, HQ numerical regression, DS-1, TS808, measured fitting, BD-2, fuzz, reference amp topology, multi-parameter calibration, power-tube family differentiation, selectable power-tube behavior inside the amp, and speaker nonlinear dynamics.

Latest code checkpoint `e201d19b429c583dff512551cda0b89d0a2aa1ea` passed configure, build and the full test suite in Graph Core CI run #133.

The measured-fit synthetic regressions intentionally verify that known DS-1 control settings can be recovered from generated reference response points. This validates the fitting machinery; it is not hardware reference data.

## Next high-value work

1. introduce real measurement datasets with provenance and metadata
2. fit DS-1 / TS808 / BD-2 control positions against those datasets
3. replace the abstract three-band stack with topology-specific Marshall/Fender tone-stack solvers
4. add named preamp topologies and topology-specific NFB/presence networks
5. fit EL34 / 6L6GC / KT88 power-stage parameters against reference data
6. combine speaker dynamics with partitioned cabinet IR as a production cabinet chain
7. expose quality and measurement information to the future analyzer/UI
