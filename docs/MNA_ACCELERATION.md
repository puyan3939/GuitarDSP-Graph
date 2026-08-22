# MNA acceleration

This phase keeps the existing `CircuitNetlist` and `MnaCircuitEngine` contracts while reducing work that was previously repeated at audio rate.

## Baseline cost

The reference implementation rebuilt the complete dense MNA matrix on every Newton iteration and then performed a fresh partial-pivot Gaussian elimination. That is simple and robust for correctness work, but it repeats linear stamping that does not depend on the Newton guess.

For a nonlinear circuit with `k` Newton iterations and matrix dimension `n`, the expensive part was effectively repeated `k` times per sample: linear stamps + nonlinear stamps + dense `O(n^3)` factorization/solve.

## Phase 3 fast path

The accelerated core separates the system into terms that change at different rates.

### Cached matrix terms

`prepare()` compiles one dense base matrix containing:

- resistor conductances
- potentiometer conductances
- capacitor trapezoidal companion conductances and leakage
- inductor branch structure and companion impedance
- independent voltage-source branch structure
- VCCS/VCVS/CCCS/CCVS coefficients
- finite-gain op-amp macro coefficients
- constant current-source RHS values

A matrix-affecting component edit marks this cache dirty. The next sample rebuilds it once without changing topology or allocating new workspace.

Independent voltage-source values intentionally do **not** invalidate the matrix. This matters for audio-rate source excitation and relay/control tests: only their RHS values change.

### Dynamic RHS terms

Per-sample state contributes only RHS terms:

- current independent voltage-source values
- capacitor trapezoidal history current
- inductor trapezoidal history voltage

The matrix remains unchanged until a matrix-affecting component edit occurs.

### Linear circuits

If no nonlinear devices are present, the cached matrix is LU-factorized once with partial pivoting. Each sample then performs only RHS construction, pivot application, forward substitution and back substitution. Audio-rate voltage-source changes therefore reuse the same factorization.

This changes the steady-state linear solve path from repeated dense factorization toward an `O(n^2)` triangular-solve cost per sample after the one-time factorization.

### Nonlinear circuits

For diode/BJT/JFET/MOSFET/triode circuits, the complete source and reactive
history RHS is assembled once per audio sample. Each Newton iteration restores
only the matrix entries touched by nonlinear devices, then stamps the unchanged
component equations and equivalent sources. Passive and controlled-source stamps
are not rebuilt or copied as a complete dense matrix inside the iteration.

`prepare()` performs symbolic analysis once and compiles:

- a Markowitz-style minimum-fill ordering of the matched MNA rows and columns;
- the original sparse matrix entries;
- the exact structural LU fill pattern;
- every diagonal and lower-factor location;
- every elimination target address;
- the rows needed for scaled backward-error validation.

The ordering never removes an unknown or changes a circuit equation. For the
complete 48-unknown TS808, it reduces symbolic factor entries from 323 to 177
and elimination multiply/subtract updates from 375 to 47. The 57-unknown DS-1
factor similarly falls from 453 entries to 219. Internal columns are scattered
back to the original schematic-facing MNA order after each solve.

Numeric sparse LU and its temporary forward/back-substitution vectors use
double precision even though the external audio and component contract remains
single precision. This avoids the amplified rounding jitter that previously kept
pedal op-amp nodes in a 40-iteration limit cycle. Newton convergence is scaled to
each unknown's voltage/current magnitude, and a bounded previous-sample predictor
provides its initial guess. Small corrections use full Newton steps; larger
transitions retain the existing trust-region damping.

Newton accepts a physically converged candidate when its freshly restamped
nonlinear KCL equations meet a strict scaled backward-error limit of `3e-7`.
This is substantially stricter than the sparse linear-solve safety limit. It
also handles high-gain op-amp nodes whose float-sized operating-point jitter
previously prevented a voltage-delta-only test from terminating: silence and
quiet guitar input could otherwise consume all 40 Newton iterations despite
already satisfying the actual circuit equations. Every sample still receives at
least one complete sparse MNA solve.

Diodes with series resistance retain the same implicit junction equation and
convergence tolerance. Their previous converged junction voltage is reused only
as the next Newton starting point, avoiding repeated cold-start exponential
solves in the TS808 and DS-1 Ebers-Moll transistor subcircuits.

Every accepted sparse result is checked against the original MNA equations. The
expensive sparse backward-error pass runs when Newton accepts the sample or
exhausts its iteration budget, instead of once for every intermediate iterate.
Unsafe pivots or residuals still fall back to the dense partial-pivot oracle.

No diode, transistor, dynamic op-amp, capacitor, potentiometer, oversampling
factor, or externally visible circuit topology is removed by these changes.

The new active-device fidelity layer is built on this split. BJT/JFET/MOSFET parasitic capacitances are ordinary trapezoidal companion elements, so their conductance terms live in the cached matrix while only history terms enter the per-sample RHS. The rail/slew/current-limited dynamic op-amp uses the nonlinear Newton path. See `ACTIVE_DEVICE_FIDELITY.md` for the modelling details and boundaries.

## Realtime boundary

All vectors and factorization workspaces are sized in `prepare()`. The steady audio path intentionally performs no topology changes and no intentional heap allocation. Matrix-affecting control edits use the existing block-boundary update path and trigger a cache rebuild rather than a topology rebuild.

## Deterministic validation

`MnaCircuitEngine::PerformanceStats` exposes structural counters rather than relying on fragile CI wall-clock thresholds:

- samples
- static cache rebuilds
- sample RHS assemblies
- nonlinear assemblies
- full factorizations
- cached linear solves
- general dense solves
- sparse Newton solves and dense fallbacks
- accepted nonlinear residual convergences

`MnaAccelerationTests` verifies that audio-rate voltage-source changes do not rebuild or refactorize the matrix, linear circuits reuse cached LU solves, matrix-affecting edits rebuild/refactorize exactly once, and nonlinear circuits reuse the cached linear base while restamping Newton-dependent terms.

`ActiveDeviceFidelityTests` verifies the new op-amp rail/slew/current-limit and transistor-parasitic paths. `CircuitNetlistTests` verifies ordinary BJT/JFET/MOSFET definitions compile with those parasitic networks enabled.

The complete pre-existing circuit regression suite remains the numerical compatibility gate.

## Local benchmark

An optional microbenchmark is available without making wall-clock performance a CI pass/fail condition:

```sh
cmake -S . -B build-bench -G Ninja -DCMAKE_BUILD_TYPE=Release -DGUITARDSP_BUILD_BENCHMARKS=ON
cmake --build build-bench --target GuitarDSPMnaBenchmark
./build-bench/GuitarDSPMnaBenchmark
```

It reports samples/second and structural counters for a 32-stage linear RC ladder and an 8-stage nonlinear diode ladder. Compare results only on the same machine, compiler, build type and CPU power policy.

## Next acceleration stages

1. Compile direct nonlinear stamp destinations so device models do not perform
   repeated row/column address arithmetic.
2. Partition circuits so only nonlinear islands iterate; solve purely linear
   regions with cached factorizations or an exact Schur complement.
3. Improve convergence of the larger DS-1 booster/op-amp circuit without
   weakening its physical component model.
4. Benchmark representative pedal, tube-preamp and power-amp netlists rather
   than synthetic ladders only.

The design rule is unchanged: every optimization must preserve the schematic-facing netlist contract and remain regression-compatible with the dense correctness model.
