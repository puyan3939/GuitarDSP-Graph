# Golden reference files

`tests/golden/` pins the current (v1) output of the component-level MNA
circuits as a fixed, file-based baseline. It exists so every future
optimization, refactor, or v2 implementation has something concrete to be
judged against, instead of against "whatever the last build happened to
produce" -- which moves every time a build flag changes. This document is
the authoritative policy for that directory; see issue #88 for the original
request.

## Update policy

**Golden files under `tests/golden/*.txt` and `tests/golden/MANIFEST.json`
must not be regenerated or edited outside of the issue that establishes or
explicitly revises them.** If a change to a circuit legitimately requires
new golden output (a deliberate behavior change, not drift), open an issue
that:

1. States the reason the golden output must change.
2. Shows the diff in output (ideally via `tools/parity_check`) and explains
   why it's expected.
3. Gets explicit approval before the regenerated files are committed.

`tests/golden/params/*.json` (the knob settings the golden files were
generated from) follow the same rule -- changing a params file invalidates
every golden file generated from it, so it needs the same approval path.

## 1. The GOLDEN CMake preset

Golden files may only be (re)generated using the `golden` preset in
`CMakePresets.json`:

```bash
cmake --preset golden
cmake --build build-golden --target generate_golden
```

The preset fixes `-O2` (not the Release default `-O3`), disables fast-math,
`-march=native` (`GUITARDSP_NATIVE_CPU`) and LTO/IPO
(`CMAKE_INTERPROCEDURAL_OPTIMIZATION`), so the build that produces the
files is reproducible across machines and compiler versions that share the
same toolchain family, rather than silently depending on whatever
optimization flags happen to be in effect.

`generate_golden` builds and runs `tools/golden_gen` (only built when
`GUITARDSP_BUILD_GOLDEN_TOOLS=ON`, which the `golden` preset sets) against
every circuit/signal/param-variant combination and writes both the `.txt`
files and `MANIFEST.json`.

## 2. Input signals

Every input signal is generated deterministically from a closed-form
expression in `tests/golden/GoldenSignals.h` -- never read from a committed
audio file -- at `fs = 48000 Hz`, computed in `float64` and rounded to
`float32`:

| name      | description                                          |
| --------- | ----------------------------------------------------- |
| `sine20`  | 1 kHz sine, -20 dBFS, 0.5 s                           |
| `sine3`   | 1 kHz sine, -3 dBFS, 0.5 s (clipping region)          |
| `sweep`   | 20 Hz -> 20 kHz exponential sweep, -12 dBFS, 2 s      |
| `impulse` | unit impulse followed by silence, 0.5 s               |
| `silence` | silence, 0.5 s (fixed point for e.g. DS-1 noise floor) |

## 3. Circuits and parameters

TS808, DS-1, Preamp, PowerAmp, FullAmp, Compressor. Knob settings live in
`tests/golden/params/<circuit>.json`, read by both `tools/golden_gen` and
`tests/GoldenReferenceTests.cpp` (no magic numbers in test code). Each file
also records the Newton iteration/tolerance literals each circuit's
`processSample()` hardcodes -- those aren't runtime-configurable today, so
this is documentation of what a golden file was generated under, not a
control knob.

TS808/DS-1/Preamp/FullAmp expose two variants, `mid` (every knob at 0.5,
i.e. "12 o'clock") and `full` (every knob at 1.0). PowerAmpCircuit and
CompressorCircuit expose no `setControls()`/knob API today (see their
headers), so they only have a `default` variant -- there is no `mid`/`full`
pair to generate for those two.

## 4. File format

`tests/golden/<circuit>_<signal>_<variant>.txt`: one sample per line, the
sample's IEEE-754 float32 bit pattern as 8 lowercase hex digits (`%08x`),
via `tests/golden/HexFloat.h`. Hex avoids the golden file itself drifting
under decimal-text rounding.

`tools/golden_gen` can also write 32-bit float WAV files for listening
(`--wav-dir <dir>`). **WAV files are never committed** -- golden comparison
is text-only; the WAV output is purely a local listening aid.

## 5. `MANIFEST.json`

Records the commit hash the files were generated from, compiler identity,
the CMake preset name, the sample rate, and an FNV-1a64 hash of each params
JSON file (so an accidental params edit without regeneration is
detectable).

## 6. `tools/parity_check`

```bash
parity_check <golden.txt> <candidate.txt> [--tolerance 1e-6]
```

Reports sample count, max absolute error, RMS error, **the index of the
first sample that exceeds tolerance** (the most useful number for finding
exactly where a regression starts), and whether the two files are bit-for-
bit identical. Exits non-zero if the candidate falls outside tolerance.
Only built when `GUITARDSP_BUILD_GOLDEN_TOOLS=ON`.

## 7. `tests/GoldenReferenceTests.cpp` (the `golden_reference` ctest)

Runs every circuit/signal/variant combination against the committed golden
files as part of the normal `ctest` suite, at a loose absolute tolerance
(`2e-3`), **not bit-exact**. That's a deliberate compromise: the default
`ctest` build is not the `GOLDEN` preset, and because these are iterative
nonlinear (Newton) solves with per-sample state, a difference from a
different optimization level can compound across a long signal instead of
staying at LSB scale. This test is meant to catch a broken or badly drifted
circuit in ordinary CI, not to enforce bit-exact reproducibility -- that
guarantee only holds when comparing two `GOLDEN`-preset builds via
`tools/parity_check`.

For TS808 and DS-1, this test also renders the JSON netlist equivalent
(`NetlistLoader.h` / `data/circuits/*.json`) against the same golden file.
Because both the hand-written class and the netlist must each independently
agree with the fixed reference, they transitively agree with each other --
this is what replaces the previous "hand-written vs. netlist, whatever this
build produces" bit-precision check for those two circuits.
`tests/NetlistParityTests.cpp` itself is unchanged: it checks a different,
still-useful property (hand-written vs. netlist cross-representation
agreement at the current build's output, across a wider parameter sweep and
including PowerAmp/Compressor), and is not superseded for those circuits.

## Known caveat: TS808 `full` variant

The five `ts808_*_full.txt` golden files (drive = tone = level = 1.0)
contain very large-magnitude samples (RMS on the order of 10^3-10^8, e.g.
the sweep file), well beyond a plausible clipped-diode output. This looks
like a genuine Newton-solver divergence at that specific extreme parameter
corner rather than intended circuit behavior -- see the issue #88
implementation report for details, and issue #91 for confirmation that the
same divergence still reproduces (with different exact numbers) after
`MnaCircuitEngine::solveDcOperatingPoint()`-based DC initialization, so it
is a runtime Newton-solve property, not a `prepare()`-time artifact. It's
captured as-is because that's the point of a golden reference (freeze
current behavior, bugs included, so a future fix is a visible, intentional
diff against this file); it is not fixed by this change. These five cases
are listed in `tests/golden/MANIFEST.json`'s `knownBad` array, which
`golden_reference` reads to still report (but not fail on) their actual max
error -- see `tests/GoldenReferenceTests.cpp`'s `loadKnownBad()`. Treat
those five files as unusually solver-sensitive when comparing against them.
