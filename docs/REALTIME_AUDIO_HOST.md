# Realtime audio host: pedal -> amp -> speaker/cab

Phase 4 opens the first complete live audio path while keeping the DSP graph and realtime constraints established in the earlier phases.

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

## Realtime boundaries

`RealtimeAudioEngine::process()` performs no graph preparation, file I/O, topology changes, IR conversion, or graph destruction. Callback buffers are allocated in `configure()`. Structural/quality/IR changes create a new `PreparedGraph` on the control thread, submit it through `RealtimeGraphHost`, and the audio callback adopts it only at a block boundary. Retired graphs are destroyed by the UI timer through `collectRetired()`.

The emergency output ceiling defaults to 0.98 linear and only clamps samples that exceed that ceiling. It is a bring-up safety net, not a tone-shaping limiter. The standalone app starts with -12 dB output trim.

## Cabinet IR policy

A measured cabinet IR is the fidelity path. The app accepts WAV/AIFF IR files and converts them to the active device sample rate offline using the windowed-sinc resampler in `ReferenceCabinetIR.h` before graph preparation.

If no measured IR is supplied, the app uses `makeReferenceCabinetImpulse()`. This is a deterministic synthetic band-limited cabinet-like response intended only to verify that speaker dynamics + partitioned convolution are live. It is **not measured data and must not be used as a cabinet-model accuracy reference**.

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

For hardware bring-up, start with output trim at -12 dB or lower, mute downstream amplification while selecting the audio device, select one input and the intended stereo/mono outputs, then unmute at low monitor volume. Start in `High` or `Live` quality to confirm stable streaming, then move to `Studio` and watch callback/dropout behaviour on the actual machine.

Load a measured cabinet IR before making cabinet-fidelity judgments. The synthetic reference fallback exists only so the full signal path can be tested immediately.

## What is deliberately not done yet

The first host does not mutate circuit topology from the audio thread and does not perform live in-place IR replacement. It also does not yet expose every pedal and amp component parameter in the GUI. Those controls should be added through bounded control-rate parameter/update paths after real-device CPU and dropout measurements are collected.
