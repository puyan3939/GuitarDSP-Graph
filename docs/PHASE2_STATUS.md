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

## Topology nodes

- HQ DS-1 topology
- HQ TS808 topology
- HQ BD-2 topology reference
- reusable two-transistor feedback fuzz topology
- generic reference amplifier topology

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
-> push-pull power stage with sag/crossover
-> transformer saturation
```

This remains an unnamed engineering reference. Named Marshall/Fender/Ampeg models require topology-specific networks and measured fitting.

## Current validation

The CI suite covers graph runtime, PDC, node registry, hot swap, multi-output, HQ numerical regression, DS-1, TS808, measured fitting, BD-2, fuzz and the reference amp topology.

The measured-fit synthetic regression intentionally verifies that a known DS-1 tone setting can be recovered from generated reference response points. This validates the fitting machinery; it is not hardware reference data.

Validated checkpoint `d49bd7414978dd009ec1b4d79ebf09eb5b8eb43d`: Graph Core CI runs #105 and #106 both completed successfully with configure, build and all 15 tests passing.

## Next high-value work

1. introduce real measurement datasets with provenance and metadata
2. fit DS-1 / TS808 / BD-2 control positions against those datasets
3. add device-family variations for transistor/diode parameters
4. build named amp topology layers on the reusable amp blocks
5. add speaker nonlinear dynamics on top of the linear partitioned IR path
6. expose quality and measurement information to the future analyzer/UI
