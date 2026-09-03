#include "guitardsp/app/RealtimeAudioEngine.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <new>

namespace {
std::atomic<bool> trackAllocations{false};
std::atomic<std::size_t> allocationCount{0};
}

void* operator new(std::size_t size) {
    if (trackAllocations.load(std::memory_order_relaxed))
        allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* pointer = std::malloc(size)) return pointer;
    throw std::bad_alloc{};
}

void* operator new[](std::size_t size) {
    if (trackAllocations.load(std::memory_order_relaxed))
        allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* pointer = std::malloc(size)) return pointer;
    throw std::bad_alloc{};
}

void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}
}

int main() {
    using namespace guitardsp;
    bool ok = true;

    app::LiveRigSettings settings;
    settings.quality = graph::ProcessingQuality::eco;
    settings.pedal = app::PedalModel::bypass;
    settings.ampEnabled = false;
    settings.cabinetEnabled = false;

    app::RealtimeAudioEngine engine;
    ok &= require(engine.configure(48000.0, 64, 2, settings),
                  "realtime audio bridge allocation fixture configures");
    engine.setInputTrimDb(0.0f);
    engine.setOutputTrimDb(0.0f);
    engine.setSafetyCeiling(0.98f);

    float input[128]{};
    float left[128]{};
    float right[128]{};
    for (int i = 0; i < 128; ++i)
        input[i] = 0.1f * std::sin(0.07f * static_cast<float>(i));
    const float* inputs[]{input};
    float* outputs[]{left, right};

    allocationCount.store(0, std::memory_order_relaxed);
    trackAllocations.store(true, std::memory_order_release);
    engine.process(inputs, 1, outputs, 2, 128);
    trackAllocations.store(false, std::memory_order_release);

    ok &= require(allocationCount.load(std::memory_order_relaxed) == 0,
                  "first realtime audio bridge callback performs zero heap allocations");

    float difference = 0.0f;
    float energy = 0.0f;
    for (int i = 0; i < 128; ++i) {
        difference += std::abs(left[i] - right[i]);
        energy += left[i] * left[i];
    }
    ok &= require(difference < 1.0e-6f && energy > 1.0e-5f,
                  "chunked mono-to-stereo callback remains coherent");

    engine.setMuted(true);
    allocationCount.store(0, std::memory_order_relaxed);
    trackAllocations.store(true, std::memory_order_release);
    engine.process(inputs, 1, outputs, 2, 128);
    trackAllocations.store(false, std::memory_order_release);

    float mutedPeak = 0.0f;
    for (float sample : left) mutedPeak = std::max(mutedPeak, std::abs(sample));
    ok &= require(allocationCount.load(std::memory_order_relaxed) == 0 && mutedPeak == 0.0f,
                  "mute path remains allocation-free and silent");

    app::LiveRigSettings parallelSettings;
    parallelSettings.quality = graph::ProcessingQuality::eco;
    parallelSettings.pedal = app::PedalModel::bypass;
    parallelSettings.ampEnabled = false;
    parallelSettings.cabinetEnabled = true;
    parallelSettings.signalRouting = app::SignalRouting::parallelOctaveBass;
    app::RealtimeAudioEngine parallelEngine;
    ok &= require(parallelEngine.configure(48000.0, 64, 1, parallelSettings),
                  "parallel octave/bass/cabinet allocation fixture configures");
    parallelEngine.setOutputTrimDb(0.0f);

    allocationCount.store(0, std::memory_order_relaxed);
    trackAllocations.store(true, std::memory_order_release);
    parallelEngine.process(inputs, 1, outputs, 2, 128);
    trackAllocations.store(false, std::memory_order_release);
    ok &= require(allocationCount.load(std::memory_order_relaxed) == 0,
                  "first parallel octave and dual-cab callback performs zero heap allocations");

    ok &= require(parallelEngine.setNodeTypeParameter(
                      "Speaker Dynamics + Partitioned Cab", 5, 155.0f)
                      && parallelEngine.setNodeTypeParameter(
                          "Speaker Dynamics + Partitioned Cab", 6, 4600.0f),
                  "live guitar cabinet low/high cuts are independently addressable");
    allocationCount.store(0, std::memory_order_relaxed);
    trackAllocations.store(true, std::memory_order_release);
    parallelEngine.process(inputs, 1, outputs, 2, 128);
    trackAllocations.store(false, std::memory_order_release);
    ok &= require(allocationCount.load(std::memory_order_relaxed) == 0,
                  "changing live cabinet filter coefficients allocates no audio-thread memory");

    app::RealtimeAudioEngine testSignalEngine;
    ok &= require(testSignalEngine.configure(48000.0, 64, 1, settings),
                  "test-signal allocation fixture configures");
    testSignalEngine.setInputRoutingMode(app::InputRoutingMode::testSignal);
    testSignalEngine.setTestSignalFrequencyHz(1000.0f);
    testSignalEngine.setTestSignalAmplitude(0.5f);
    testSignalEngine.setMuted(false);

    float tap[128]{};
    float outputTap[128]{};
    allocationCount.store(0, std::memory_order_relaxed);
    trackAllocations.store(true, std::memory_order_release);
    // Physical input is null throughout: the whole point of testSignal mode
    // is to feed the graph without any physical input present.
    testSignalEngine.process(nullptr, 0, outputs, 2, 128,
                             app::MonitorTapPoint::physicalInput, tap,
                             app::MonitorTapPoint::physicalOutput, outputTap);
    trackAllocations.store(false, std::memory_order_release);
    ok &= require(allocationCount.load(std::memory_order_relaxed) == 0,
                  "test-signal oscillator callback performs zero heap allocations");

    float tapPeak = 0.0f;
    bool tapFinite = true;
    for (float sample : tap) {
        tapPeak = std::max(tapPeak, std::abs(sample));
        tapFinite &= std::isfinite(sample);
    }
    ok &= require(tapFinite && tapPeak > 0.0f && tapPeak <= 0.5f + 1.0e-6f,
                  "test-signal tap carries a finite sine bounded by the configured amplitude");

    float outputEnergy = 0.0f;
    for (float sample : left) outputEnergy += sample * sample;
    ok &= require(outputEnergy == 0.0f,
                  "test-signal mode always forces the physical output silent, "
                  "even with setMuted(false)");

    float outputTapPeak = 0.0f;
    bool outputTapFinite = true;
    for (float sample : outputTap) {
        outputTapPeak = std::max(outputTapPeak, std::abs(sample));
        outputTapFinite &= std::isfinite(sample);
    }
    ok &= require(outputTapFinite && outputTapPeak > 0.0f,
                  "test-signal output tap still carries the real post-DSP signal "
                  "while the physical output is forced silent");

    ok &= require(testSignalEngine.stats().inputRoutingMode == app::InputRoutingMode::testSignal,
                  "stats report the active test-signal routing mode");

    {
        // Issue #76 change 2: a SIGNAL CHAIN monitor tap (e.g. "GUITAR
        // AMPLIFIER out"/"SPEAKER+CABINET out") should carry the live
        // internal node signal, stay allocation-free across a device
        // callback split into several sub-blocks (numSamples >
        // maximumBlockSize), and fall back to silence rather than garbage
        // when the requested stage doesn't exist in the current rig.
        app::LiveRigSettings full;
        full.quality = graph::ProcessingQuality::eco;
        full.pedal = app::PedalModel::ts808Circuit;
        full.amp = app::AmpModel::reference;
        full.ampEnabled = true;
        full.cabinetEnabled = true;
        full.cabinetPartitionSize = 16;

        app::RealtimeAudioEngine fullEngine;
        ok &= require(fullEngine.configure(48000.0, 32, 1, full),
                      "pedal+amp+cabinet allocation fixture configures with a small block size");
        fullEngine.setMuted(false);
        fullEngine.setOutputTrimDb(0.0f);

        // Reference amp is a physically-modelled tube stage that needs a
        // real self-bias settling period (see LiveRigTests.cpp's "right-jack
        // guitar stays audible after tube self-bias settles"), so drive many
        // callbacks and only check the tap contents from the last one.
        // numSamples (96) > maximumBlockSize (32) on every callback still
        // forces sub-block splitting throughout.
        constexpr int callbackSamples = 96;
        constexpr int callbacks = 200;
        float driveInput[callbackSamples]{};
        float driveLeft[callbackSamples]{};
        const float* driveInputs[]{driveInput};
        float* driveOutputs[]{driveLeft};
        float ampTap[callbackSamples]{};
        float cabinetTap[callbackSamples]{};

        for (int callback = 0; callback < callbacks; ++callback) {
            for (int i = 0; i < callbackSamples; ++i) {
                const int sample = callback * callbackSamples + i;
                driveInput[i] = 0.3f * std::sin(0.2f * static_cast<float>(sample));
            }
            if (callback == 0) {
                allocationCount.store(0, std::memory_order_relaxed);
                trackAllocations.store(true, std::memory_order_release);
            }
            fullEngine.process(driveInputs, 1, driveOutputs, 1, callbackSamples,
                               app::MonitorTapPoint::ampOutput, ampTap,
                               app::MonitorTapPoint::cabinetOutput, cabinetTap);
            if (callback == 0) trackAllocations.store(false, std::memory_order_release);
        }
        ok &= require(allocationCount.load(std::memory_order_relaxed) == 0,
                      "SIGNAL CHAIN node taps stay allocation-free across a sub-block-split callback");

        bool ampTapFinite = true, cabinetTapFinite = true;
        float ampTapEnergy = 0.0f, cabinetTapEnergy = 0.0f;
        for (float sample : ampTap) {
            ampTapFinite &= std::isfinite(sample);
            ampTapEnergy += sample * sample;
        }
        for (float sample : cabinetTap) {
            cabinetTapFinite &= std::isfinite(sample);
            cabinetTapEnergy += sample * sample;
        }
        ok &= require(ampTapFinite && ampTapEnergy > 1.0e-8f,
                      "GUITAR AMPLIFIER monitor tap carries a finite, audible internal signal");
        ok &= require(cabinetTapFinite && cabinetTapEnergy > 1.0e-8f,
                      "SPEAKER+CABINET monitor tap carries a finite, audible internal signal");

        // Bypassed pedal: drive.ts808_circuit_hq/drive.ds1_circuit_hq are both
        // absent from the compiled graph, so resolveMonitorNodeTap() has
        // nothing to find and must write silence, not stale/garbage memory.
        app::LiveRigSettings bypassPedal = full;
        bypassPedal.pedal = app::PedalModel::bypass;
        app::RealtimeAudioEngine bypassEngine;
        ok &= require(bypassEngine.configure(48000.0, 32, 1, bypassPedal),
                      "bypassed-pedal fixture configures");
        bypassEngine.setMuted(false);
        float pedalTap[callbackSamples];
        std::fill(std::begin(pedalTap), std::end(pedalTap), 1.0f); // poison, must be overwritten with 0.
        float scratchOutput[callbackSamples]{};
        float* scratchOutputs[]{scratchOutput};
        bypassEngine.process(driveInputs, 1, scratchOutputs, 1, callbackSamples,
                             app::MonitorTapPoint::pedalOutput, pedalTap,
                             app::MonitorTapPoint::physicalOutput, nullptr);
        bool pedalTapSilent = true;
        for (float sample : pedalTap) pedalTapSilent &= (sample == 0.0f);
        ok &= require(pedalTapSilent,
                      "a monitor tap pointed at a stage absent from the rig reads back silence");
    }

    {
        // Parallel/crossover branches (issue #76 change 2 follow-up: octave,
        // bass amp and bass cabinet must be tappable too, since that's the
        // whole point of confirming a MIYAVI-style split's bass response).
        app::LiveRigSettings parallel;
        parallel.quality = graph::ProcessingQuality::eco;
        parallel.pedal = app::PedalModel::bypass;
        parallel.ampEnabled = false;
        parallel.cabinetEnabled = false;
        parallel.signalRouting = app::SignalRouting::parallelOctaveBass;
        parallel.octaveEnabled = true;
        parallel.bassCabinetEnabled = true;
        parallel.guitarBranchLevel = 0.0f;
        parallel.bassBranchLevel = 1.0f;
        parallel.bassLevel = 1.0f;

        app::RealtimeAudioEngine parallelTapEngine;
        ok &= require(parallelTapEngine.configure(48000.0, 64, 1, parallel),
                      "parallel octave/bass-amp/bass-cab tap fixture configures");
        parallelTapEngine.setMuted(false);

        // Bass amp is also a self-biasing tube model (see the settling note
        // above), so again drive many callbacks and only check the last one.
        constexpr int samples = 128;
        constexpr int bassCallbacks = 200;
        float bassInput[samples]{};
        float bassLeft[samples]{};
        const float* bassInputs[]{bassInput};
        float* bassOutputs[]{bassLeft};
        float octaveTap[samples]{};
        float bassCabTap[samples]{};

        for (int callback = 0; callback < bassCallbacks; ++callback) {
            for (int i = 0; i < samples; ++i) {
                const int sample = callback * samples + i;
                bassInput[i] = 0.2f * std::sin(0.05f * static_cast<float>(sample));
            }
            parallelTapEngine.process(bassInputs, 1, bassOutputs, 1, samples,
                                      app::MonitorTapPoint::octaveOutput, octaveTap,
                                      app::MonitorTapPoint::bassCabinetOutput, bassCabTap);
        }

        bool octaveFinite = true, bassCabFinite = true;
        float octaveEnergy = 0.0f, bassCabEnergy = 0.0f;
        for (float sample : octaveTap) { octaveFinite &= std::isfinite(sample); octaveEnergy += sample * sample; }
        for (float sample : bassCabTap) { bassCabFinite &= std::isfinite(sample); bassCabEnergy += sample * sample; }
        ok &= require(octaveFinite && octaveEnergy > 1.0e-8f,
                      "octave-branch monitor tap carries a finite, audible signal");
        ok &= require(bassCabFinite && bassCabEnergy > 1.0e-8f,
                      "bass-cabinet-branch monitor tap carries a finite, audible signal");
    }

    return ok ? 0 : 1;
}
