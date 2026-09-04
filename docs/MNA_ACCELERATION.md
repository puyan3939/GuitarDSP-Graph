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

- a linear-first Markowitz ordering of the matched MNA rows and columns;
- the original sparse matrix entries;
- the exact structural LU fill pattern;
- every diagonal and lower-factor location;
- every elimination target address;
- the rows needed for scaled backward-error validation;
- an exact, reusable factorization of the invariant linear prefix.

The ordering never removes an unknown or changes a circuit equation. For the
complete 48-unknown TS808, 27 linear unknowns are eliminated once whenever the
static conductance matrix changes. Newton iterations factor only the remaining
21-unknown nonlinear Schur boundary, then reconstruct all 48 schematic-facing
voltages and branch currents. This deliberately trades a small amount of
symbolic fill for a reusable linear prefix: the factor has 193 entries instead
of 323 in schematic order, and only 15 of its 71 elimination updates remain in
the per-Newton suffix. The 57-unknown DS-1 similarly caches 29 linear unknowns,
leaving a 28-unknown nonlinear boundary and a 238-entry factor instead of 453.
The same split covers the tube amp stages added afterward: the 14-unknown
component-level preamp caches 11 linear unknowns, and the 18-unknown power amp
caches 13; `FullAmpCircuit` cascades those two prepared engines independently,
so it needs no separate partitioning of its own. `MnaAccelerationTests.cpp`,
`MnaSparseNonlinearTests.cpp`, `TS808CircuitTests.cpp`, `DS1CircuitTests.cpp`,
`PreampCircuitTests.cpp` and `PowerAmpCircuitTests.cpp` all assert on
`cachedLinearUnknowns()`/`sparseNonlinearCachedLinearUnknowns()` so a future
change cannot silently regress the split back to reiterating the whole matrix.

The invariant-prefix factor and the static Schur complement are refreshed when
potentiometers or other conductance-affecting controls change. Reactive/source
history is forward-substituted once per audio sample, while nonlinear equivalent
sources are still updated on every Newton iteration. All original capacitor,
transistor, diode and op-amp equations remain active.

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
solves in the TS808 and DS-1 Ebers-Moll transistor subcircuits. The prepared
sparse path caches each device's inverse thermal voltage; clamped exponential
endpoints use their identical precomputed float values. Dense-reference mode
keeps its original division path.

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

It reports samples/second and structural counters for a 32-stage linear RC ladder and an 8-stage nonlinear diode ladder. Compare results only on the same machine, compiler, build type and CPU power policy. As of issue #85 it also reports the component-level preamp/power-amp stages and the `FullAmpCircuit` preamp->power-amp cascade (`runFullAmp`), reporting each stage's own `MnaCircuitEngine` counters since the cascade is two independent engines rather than one combined solve.

## Next acceleration stages

1. Compile direct nonlinear stamp destinations so device models do not perform
   repeated row/column address arithmetic.
2. ~~Partition circuits so only nonlinear islands iterate; solve purely linear
   regions with cached factorizations or an exact Schur complement.~~ **Done.**
   This was already implemented as part of the `FixedPatternSparseSolver`
   symbolic ordering above (`linearPrefix_`/`cachedLinearValues_` in
   `FixedPatternSparseSolver.h`, wired through
   `MnaCircuitEngineCore::processSample()`'s Newton loop): every Newton
   iteration reuses the cached invariant linear-prefix factorization and only
   factors/solves the mutable nonlinear Schur boundary. See the "Nonlinear
   circuits" section above and the `cachedLinearUnknowns()` regression
   assertions in `MnaAccelerationTests.cpp`, `MnaSparseNonlinearTests.cpp`,
   `TS808CircuitTests.cpp`, `DS1CircuitTests.cpp`, `PreampCircuitTests.cpp`
   and `PowerAmpCircuitTests.cpp` (issue #54).
3. Improve convergence of the larger DS-1 booster/op-amp circuit without
   weakening its physical component model.
4. ~~Benchmark representative pedal, tube-preamp and power-amp netlists rather
   than synthetic ladders only.~~ **Done.** `bench/MnaBenchmark.cpp` benchmarks
   TS808/DS-1 (pedals) and, as of issue #54, the component-level preamp and
   power-amp stages as well.
5. ~~Replace the triode/pentode nonlinear stamps' central-difference Jacobian
   with an analytic one.~~ **Done (issue #85).** `stampTriode`/`stampPentode`
   in `MnaCircuitEngineCore.h` used two-sided central differences to build
   gm/gp(/gscreen) each Newton iteration -- 3 evaluations of
   `TriodeModel::plateCurrent` per triode stamp, 12 evaluations of
   `PentodeModel::plateCurrent`/`screenCurrent` per pentode stamp, each of
   which recomputes the same `exp`/`log1p`/`pow`(/`atan`) chain from scratch.
   `TriodeModel::plateCurrentJacobian` and `PentodeModel::currentsAndJacobian`
   (`include/guitardsp/hq/Components.h`) instead derive the exact partials in
   closed form by chain rule through that same softplus/power-law/atan
   structure, reusing the `exp`/`log1p`/`pow` calls the current value itself
   already needs -- 1 model evaluation per triode stamp, 2 per pentode stamp.
   Current values are bit-for-bit identical to before (same `plateCurrent`/
   `screenCurrent` functions, now also called by the new methods); only the
   Jacobian used to build each Newton step changes, which affects the path to
   convergence, not the converged operating point (see "Nonlinear solver
   design" in `CLAUDE.md` for why the backtracking line search's
   backward-error convergence test doesn't depend on which Jacobian
   approximation produced a candidate step). Regression suite (`ctest`, all
   50 tests including `PowerAmpCircuitTests`, `FullAmpCircuitTests`,
   `ThdSweepTests`, `NetlistParityTests`) is unaffected. Measured on this
   repo's CI-equivalent build (Release, `GUITARDSP_BUILD_BENCHMARKS=ON`;
   `bench/MnaBenchmark.cpp`'s `runPreamp`/`runPowerAmp`/`runFullAmp`,
   3-run average, same machine before/after):

   | Stage | Before (central diff) | After (analytic) | Speedup |
   |---|---|---|---|
   | Preamp (12AX7 triode) | 4.35% est. CPU | 2.88% est. CPU | ~1.51x |
   | PowerAmp (EL34 pentode) | 7.72% est. CPU | 3.53% est. CPU | ~2.18x |
   | FullAmp (cascade) | 12.37% est. CPU | 6.46% est. CPU | ~1.91x |

   ("est. CPU" is `MnaBenchmark`'s `estimated_cpu_percent` -- realtime block
   processing time as a fraction of wall-clock audio time, single core,
   independent of the reporter's actual core count.) This beat the
   conservative pre-implementation estimate (1.5-1.8x PowerAmp, 1.3-1.5x
   FullAmp based on transcendental-call counting) since PowerAmp's pentode
   stamp went from 12 model evaluations to 2, larger than the estimate's
   call-counting assumed.
6. SIMD (e.g. AVX) intra-core parallelism across nonlinear device evaluations
   was considered as an alternative to item 5 and set aside for now: a
   typical stamped circuit has only one triode/pentode per Newton iteration
   (no SIMD lanes to fill across devices), and the fixed-pattern sparse
   factorization's column addressing isn't a simple strided/vectorizable
   access pattern as currently structured. Worth revisiting if matrix
   factorization/back-substitution becomes the dominant cost after item 5.
7. A simplified/lower-quality approximate mode for `FullAmpCircuit` at low
   quality settings was also considered and set aside: profiling for issue
   #85 found the cascade itself carries no measurable overhead beyond
   "preamp cost + power-amp cost" (independent single-engine costs sum to
   within ~0.1% of the measured combined cost), so there is no cascade-specific
   inefficiency for an approximate mode to remove.

The design rule is unchanged: every optimization must preserve the schematic-facing netlist contract and remain regression-compatible with the dense correctness model.
