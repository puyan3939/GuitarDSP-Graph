#include "guitardsp/app/RealtimeAudioEngine.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
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

    return ok ? 0 : 1;
}
