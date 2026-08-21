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

For diode/BJT/JFET/MOSFET/triode circuits, each Newton iteration begins from the cached linear base matrix and only adds the nonlinear Jacobian/equivalent-source terms before the dense reference solve. This removes repeated stamping of the passive and controlled-source network from the Newton loop while preserving the existing nonlinear equations and partial-pivot solver.

The nonlinear solve is still dense `O(n^3)` per Newton iteration. Fixed-pattern/sparse work comes next.

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

`MnaAccelerationTests` verifies that audio-rate voltage-source changes do not rebuild or refactorize the matrix, linear circuits reuse cached LU solves, matrix-affecting edits rebuild/refactorize exactly once, and nonlinear circuits reuse the cached linear base while restamping Newton-dependent terms.

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

1. Preassemble the complete per-sample RHS once and reuse it across Newton iterations.
2. Compile fixed matrix write locations so nonlinear stamps avoid repeated row/column address arithmetic.
3. Separate symbolic sparsity analysis from numeric factorization.
4. Add a fixed-pattern sparse LU backend for larger circuit islands while retaining the dense solver as a correctness oracle.
5. Partition circuits so only nonlinear islands iterate; solve purely linear regions with cached factorizations.
6. Add bounded Newton continuation/voltage limiting so acceleration does not trade away difficult-circuit convergence.
7. Benchmark representative pedal, tube-preamp and power-amp netlists rather than synthetic ladders only.

The design rule is unchanged: every optimization must preserve the schematic-facing netlist contract and remain regression-compatible with the dense correctness model.
