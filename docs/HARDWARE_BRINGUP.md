# Hardware bring-up and realtime observability

Phase 5 turns the first JUCE audio host into an instrument that can be safely
connected to a physical audio interface and diagnosed without guessing whether
silence, distortion or a dropout originates at the jack, the DSP graph or the
driver.

## Startup boundary

The standalone application starts with:

- output muted;
- output trim at -12 dB;
- an explicitly selected safe dry monitor graph;
- two requested input channels and two requested output channels;
- previously saved device/sample-rate/buffer/channel settings, when available;
- `*WAVIO*` as the preferred default device when no saved device exists.

JUCE can open fewer channels than requested when the interface does not provide
two. No particular physical WAVIO device is claimed to have been exercised by CI.

Safe dry monitor is a real preallocated graph. It bypasses the component pedal,
amplifier and cabinet while retaining the normal input routing, trims, output
safety boundary and realtime telemetry. Disable it only after the physical input
and output meters behave as expected.

## Physical input routing

Four explicit policies are available:

1. **Auto mono:** choose the stronger valid physical input, apply switching
   hysteresis, process one component-level guitar signal, and copy its result to
   both physical outputs.
2. **Input 1:** process physical input 1 once and copy it to both outputs.
3. **Input 2:** process physical input 2 once and copy it to both outputs.
4. **Stereo:** preserve independent left/right inputs and independent circuit
   state for both processing channels.

Changing between mono and independent stereo synchronizes with the current audio
callback before preparing a graph with the new processing-channel count. Stereo
hardware outputs alone no longer cause a mono guitar pedal, amplifier and cabinet
to be solved twice.

An explicitly requested input that is unavailable produces silence rather than
silently substituting a different jack. Automatic mode keeps its previous
selection while the signal is below the detection floor, which avoids arbitrary
left/right flapping between notes.

This fixes the important WAVIO-style failure mode where input 1 is silent and the
guitar is connected only to input 2: the right physical input becomes the mono
source, rather than being overwritten by a silent left channel.

## Signal safety

The bridge records separate physical input peaks before routing and trim. ADC
full-scale samples, output safety-ceiling events, and non-finite input/output
samples use independent cumulative counters.

NaN/Inf samples are replaced by silence before they enter the graph or leave the
output boundary. The fixed emergency output ceiling remains a final hardware
protection measure, not a desired tone-shaping stage.

## CPU, XRUN and latency measurements

Each prepared callback records its elapsed monotonic-clock time and compares that
duration with the actual callback budget:

`callback budget = numSamples / sampleRate`

The monitor publishes a smoothed callback load, peak callback load, peak duration
and the number of callbacks that missed their deadline. Timing calculations are
also covered by deterministic tests that inject elapsed durations instead of
depending on a machine's wall-clock speed.

JUCE's independent device-manager CPU estimate and native/estimated XRUN count
are shown alongside the core callback statistics. They are complementary:

- an internal deadline miss means the measured DSP callback exceeded its budget;
- a device XRUN means the selected driver reported or estimated an under/overrun;
- neither value, by itself, proves which plugin, driver or competing process was
  responsible.

Measure callback load both while playing and while the guitar is quiet. The
component TS808's former voltage-delta-only Newton criterion was worst near its
quiescent operating point, so a loud-sine benchmark could pass while actual
low-level input produced repeated 40-iteration cycles and driver XRUNs. The
current solver also tests the freshly restamped nonlinear circuit residual;
quiet-input regression coverage therefore belongs alongside driven-waveform
tests.

The app reports buffer duration, the driver's reported input/output latency, DSP
graph latency and their reported I/O + DSP total separately. It does not add the
buffer a second time because drivers differ in whether their reported I/O latency
already includes buffering.

## Live tone controls

The JUCE host exposes pedal Drive/Tone/Level and amplifier
Gain/Bass/Mid/Treble/Master/Presence. UI events are coalesced by a 20 Hz message
thread timer and written to existing node atomic parameters. These edits do not
rebuild the graph, rerun nonlinear operating-point continuation, or reset
capacitor/tube history.

Structural edits such as pedal/amp model, quality mode, cabinet selection and
safe-dry switching still prepare a new graph on the control thread and adopt it
at a block boundary. Pending and active graphs both receive live parameter edits,
so a simultaneous prepared-graph swap does not discard a knob movement.

## First real-device procedure

1. Connect the guitar buffer and interface with downstream amplification low.
2. Open the app and choose the intended device/sample rate/buffer in the device
   selector. The app restores that choice on later launches.
3. Keep **Safe dry monitor** selected. Play and inspect both physical input meters.
4. Choose Auto mono, Input 1 or Input 2 until the selected source and both output
   meters are coherent.
5. Unmute at the default -12 dB output trim and verify clean direct monitoring.
6. Disable Safe dry monitor, select the component pedal/amp, and load a measured
   cabinet IR if cabinet-fidelity evaluation matters.
7. Compare Live (4x), High (8x) and Studio (16x) on the actual machine while
   watching callback peak, deadline misses, driver CPU and XRUNs.
8. Increase the device buffer if necessary before changing the agreed audio
   quality target. The synthetic cabinet fallback is only a smoke-test response.

## Current boundaries

CI verifies routing, finite-sample protection, deadline accounting, zero callback
allocations, live parameter changes, the core suite and the JUCE Linux standalone
build. CI cannot verify a physical WAVIO's driver behaviour, analog gain,
round-trip latency or subjective sound; those require the user's real interface.
