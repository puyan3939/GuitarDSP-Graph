# Realtime audio host: component-level guitar and parallel octave/bass rigs

Phase 5 opens live hardware playback and selectable parallel graph routing while
keeping the component-level pedal model and established realtime constraints.

## Signal path

The standalone host prepares this graph off the audio thread:

```text
Audio interface input
    -> component-level TS808 / component-level DS-1 / bypass
    -> Reference Amp / British Plexi family / American Clean family
    -> SpeakerDynamicsNode
    -> PartitionedCabNode
    -> audio interface output
```

Input and output trims live in the JUCE-independent `RealtimeAudioEngine` so they can be changed atomically without rebuilding the graph. Mono guitar input is duplicated coherently to both processing channels when a stereo output device is active.

The Routing + Bass page can also prepare two real branched graphs:

```text
                               -> pedal -> guitar amp -> guitar cab -> guitar level --
audio input -> split/crossover                                                     -> merge -> output
                               -> octave -1 -> bass amp -> bass cab -> bass level ----
```

`Parallel octave + bass amp` sends the full input to both branches. `Crossover
octave + bass amp` sends the complementary high band to the guitar path and the
low band to the octave/bass path. The crossover frequency, branch levels, octave
mix/level and dedicated bass drive/tone/output are independently adjustable. The
octave stage is a **monophonic** Schmitt/PLL divider, not a polyphonic POG-style
pitch shifter. Octave and bass cabinet stages can each be omitted structurally.
Graph PDC aligns the cabinet and oversampling delays before the branch merge.

## Realtime boundaries

`RealtimeAudioEngine::process()` performs no graph preparation, file I/O, topology changes, IR conversion, or graph destruction. Callback buffers are allocated in `configure()`. Structural/quality/IR changes create a new `PreparedGraph` on the control thread, submit it through `RealtimeGraphHost`, and the audio callback adopts it only at a block boundary. Retired graphs are destroyed by the UI timer through `collectRetired()`.

Realtime parameter edits can target either a node category or an exact node type.
The standalone host uses exact types for guitar and bass amplifiers so changing
one branch never rewrites another amplifier's controls. Both serial and parallel
graphs have first-callback zero-allocation regression coverage.

The emergency output ceiling defaults to 0.98 linear and only clamps samples that exceed that ceiling. It is a bring-up safety net, not a tone-shaping limiter. The standalone app starts with -12 dB output trim.

## Cabinet IR policy

A measured cabinet IR is the fidelity path. The app accepts WAV/AIFF IR files for
both the guitar cabinet and the dedicated bass cabinet (separate `LOAD MEASURED
IR` / `LOAD MEASURED BASS IR` buttons and file choosers, each keeping its own
loaded IR independently) and converts them to the active device sample rate
offline using the windowed-sinc resampler in `ReferenceCabinetIR.h` before graph
preparation. `Match measured IR loudness` is enabled by default and applies to
both cabinets: it removes inaudible DC, measures the actual convolution
frequency response, and applies one broadband gain to target roughly -1 dB
through the midrange while bounding the highest response to +4 dB. It does not
EQ, flatten, compress, or limit the measured cabinet. The control can be
disabled when the original capture gain is required. Non-finite IR samples are
removed before the graph reaches the realtime callback.

If no measured IR is supplied for the guitar cabinet, the app uses
`makeReferenceCabinetImpulse()`. The synthetic reference is designed as a stable
high-pass/body/mid/presence/low-pass filter cascade with short early
reflections. Its response is normalized by midband convolution gain, not by the
peak amplitude of a single impulse sample. At 44.1, 48, and 96 kHz the midrange
is approximately -0.75 dB and the broad cabinet-body peak remains around +2.1
dB. It is **not measured data and must not be used as a cabinet-model accuracy
reference**.

The dedicated bass branch falls back the same way: if no measured IR has been
loaded for it, it uses a separately voiced synthetic fallback from
`makeReferenceBassCabinetImpulse()` (`LiveRigSettings::bassCabinetImpulse` empty
means fallback, same as `cabinetImpulse` for the guitar cabinet). Its frequency
response is bounded during offline preparation so a resonant impulse cannot
silently add tens of decibels of convolution gain. It is also **not measured**.

The Speaker + Cabinet inspector exposes IR Mix, adjustable 12 dB/octave low cut
and high cut, voice-coil compression, excursion, low-frequency speaker resonance,
cabinet output level, external IR selection, and measured-IR level matching.
The low/high filters shape only the wet cabinet path. IR Mix combines that path
with the post-speaker/pre-IR signal through a matching dry delay; 0%, 50%, and
100% all preserve the same reported cabinet latency. Low cut, high cut, speaker,
and mix edits are atomic realtime parameters and never rebuild the graph.

The standalone interface presents the signal chain as a vertical selector on
the left and opens the selected block's complete controls in the inspector on
the right. Guitar amplifier gain/EQ/output and the existing EL34/6L6GC/KT88
power-tube, reference/British/American tone-stack, cathode-follower/plate-driver,
and negative-feedback controls are together in the amplifier inspector.
Family-specific British/American amps intentionally keep their fixed internal
family selections. Audio-device setup is collapsed until `AUDIO SETTINGS` is
selected, leaving the main editing surface available for actual tone controls.

## Quality modes

Nonlinear drive and amp nodes follow the existing quality policy:

- Eco: 2x
- Live: 4x
- High: 8x
- Studio: 16x

The graph recomputes latency after node preparation, so the selected oversampling quality and the cabinet partition latency are included in PDC.

## Linux build

The normal core build remains JUCE-independent.

To build the standalone app with a local JUCE checkout:

```bash
cmake -S . -B build-app -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGUITARDSP_BUILD_AUDIO_APP=ON \
  -DGUITARDSP_JUCE_PATH=/path/to/JUCE
cmake --build build-app --target GuitarDSPApp --parallel
```

Or let CMake fetch the CI-pinned JUCE release:

```bash
cmake -S . -B build-app -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGUITARDSP_BUILD_AUDIO_APP=ON \
  -DGUITARDSP_FETCH_JUCE=ON
cmake --build build-app --target GuitarDSPApp --parallel
```

For an executable that will run on the same machine where it is compiled, add
`-DGUITARDSP_NATIVE_CPU=ON` to either configure command. GCC/Clang then use
`-march=native` for the DSP core and standalone host. Keep this option disabled
for portable binaries built on a different CPU. It does not enable fast-math or
change finite-sample safety checks.

Typical Debian/Ubuntu development packages for the Linux standalone build are:

```bash
sudo apt install -y \
  git cmake ninja-build g++ \
  libasound2-dev libjack-jackd2-dev \
  libx11-dev libxext-dev libxinerama-dev libxrandr-dev \
  libxcursor-dev libxcomposite-dev libfreetype6-dev libfontconfig1-dev
```

The executable is generated in the JUCE product output directory under `build-app`; CMake prints the exact path during the build.

## First-listen procedure

The Phase 5 host starts muted in Safe dry monitor with -12 dB output trim. Select
the interface, inspect both physical input meters, confirm the automatically
selected guitar jack, and then unmute at low monitor volume. Disable Safe dry
monitor after direct monitoring is verified. Start in `Eco` quality to confirm
stable streaming, especially on older CPUs. Move to `Live`, `High`, or `Studio`
only while watching callback peak, deadline misses and driver XRUNs on the
actual machine. Enable the parallel bass branch after the serial guitar path is
stable because its second cabinet adds real, measurable CPU work.

Load a measured cabinet IR before making cabinet-fidelity judgments. The synthetic reference fallback exists only so the full signal path can be tested immediately.

See `HARDWARE_BRINGUP.md` for input-routing policy, saved device state, realtime
CPU/XRUN telemetry, latency accounting and atomic live pedal/amplifier controls.

## What is deliberately not done yet

The host does not mutate circuit topology from the audio thread and does not
perform live in-place IR replacement. The main pedal and amplifier tone controls
use atomic parameters updated at a bounded UI/control rate, but arbitrary
component-by-component Circuit Mode editing remains future work.
