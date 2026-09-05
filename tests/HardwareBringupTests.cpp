#include "guitardsp/app/RealtimeAudioEngine.h"
#include "guitardsp/app/RealtimePerformanceMonitor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>

namespace {

bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

float peak(const std::array<float, 64>& samples) {
    float result = 0.0f;
    for (const float sample : samples)
        result = std::max(result, std::abs(sample));
    return result;
}

} // namespace

int main() {
    using namespace guitardsp;
    bool ok = true;

    {
        app::RealtimePerformanceMonitor monitor;
        monitor.prepare(48000.0);
        monitor.recordCallback(48, 500000);  // 0.50 ms / 1.00 ms
        auto timing = monitor.snapshot();
        ok &= require(timing.callbacks == 1 && timing.latestBudgetNanoseconds == 1000000
                          && std::abs(timing.averageLoad - 0.5f) < 1.0e-6f,
                      "callback monitor computes deterministic 50 percent realtime load");

        monitor.recordCallback(48, 1250000); // 1.25 ms / 1.00 ms
        timing = monitor.snapshot();
        ok &= require(timing.callbacks == 2 && timing.deadlineMisses == 1
                          && timing.peakDurationNanoseconds == 1250000
                          && std::abs(timing.peakLoad - 1.25f) < 1.0e-6f,
                      "callback monitor counts real deadline misses and retains peak load");
        monitor.reset();
        ok &= require(monitor.snapshot().callbacks == 0
                          && monitor.snapshot().deadlineMisses == 0,
                      "performance diagnostics reset without changing the sample rate");

        for (int callback = 0; callback < 99; ++callback)
            monitor.recordCallback(48, 400000);
        monitor.recordCallback(48, 1500000);
        timing = monitor.snapshot();
        ok &= require(timing.callbacks == 100 && timing.deadlineMisses == 1
                          && std::abs(timing.percentile95Load - 0.40f) < 0.011f
                          && std::abs(timing.percentile99Load - 0.40f) < 0.011f
                          && std::abs(timing.peakLoad - 1.50f) < 1.0e-6f,
                      "fixed allocation-free timing histogram separates p99 from a startup spike");
        monitor.reset();
        timing = monitor.snapshot();
        ok &= require(timing.callbacks == 0 && timing.percentile95Load == 0.0f
                          && timing.percentile99Load == 0.0f,
                      "performance reset clears both realtime percentile histograms");
    }

    {
        app::RealtimePerformanceMonitor monitor;
        monitor.prepare(48000.0);

        auto timing = monitor.snapshot();
        ok &= require(!timing.cpuTimeAvailable && timing.cpuAverageLoad == 0.0f
                          && timing.cpuPeakLoad == 0.0f,
                      "cpu-time load defaults to unavailable/zero before any measurement");

        // 0.25 ms / 1.00 ms budget: exercises the CPU-time-supplied overload
        // alongside the existing wall-clock-only one above without changing
        // wall-clock computation.
        monitor.recordCallback(48, 500000, std::optional<std::uint64_t>(250000));
        timing = monitor.snapshot();
        ok &= require(timing.cpuTimeAvailable
                          && std::abs(timing.cpuAverageLoad - 0.25f) < 1.0e-6f
                          && std::abs(timing.averageLoad - 0.5f) < 1.0e-6f,
                      "cpu-time load tracks separately from wall-clock load when supplied");

        monitor.recordCallback(48, 1250000, std::optional<std::uint64_t>(1000000));
        timing = monitor.snapshot();
        ok &= require(std::abs(timing.cpuPeakLoad - 1.0f) < 1.0e-6f,
                      "cpu-time load retains its own peak independent of wall-clock peak");

        monitor.reset();
        timing = monitor.snapshot();
        ok &= require(!timing.cpuTimeAvailable && timing.cpuAverageLoad == 0.0f
                          && timing.cpuPeakLoad == 0.0f,
                      "performance reset clears cpu-time load alongside wall-clock load");
    }

    app::LiveRigSettings settings;
    settings.quality = graph::ProcessingQuality::eco;
    settings.pedal = app::PedalModel::bypass;
    settings.ampEnabled = false;
    settings.cabinetEnabled = false;

    app::RealtimeAudioEngine engine;
    ok &= require(engine.configure(48000.0, 64, 2, settings),
                  "hardware bring-up dry monitor configures");
    engine.setInputTrimDb(0.0f);
    engine.setOutputTrimDb(0.0f);
    engine.setSafetyCeiling(0.98f);

    std::array<float, 64> physicalLeft{};
    std::array<float, 64> physicalRight{};
    std::array<float, 64> outputLeft{};
    std::array<float, 64> outputRight{};
    for (std::size_t i = 0; i < physicalRight.size(); ++i)
        physicalRight[i] = 0.20f * std::sin(0.11f * static_cast<float>(i));

    const float* stereoInputs[]{physicalLeft.data(), physicalRight.data()};
    float* stereoOutputs[]{outputLeft.data(), outputRight.data()};

    {
        app::RealtimeAudioEngine monoEngine;
        ok &= require(monoEngine.configure(48000.0, 64, 1, settings),
                      "one-channel guitar rig prepares independently of stereo outputs");
        monoEngine.setInputRoutingMode(app::InputRoutingMode::input2);
        monoEngine.setInputTrimDb(0.0f);
        monoEngine.setOutputTrimDb(0.0f);
        monoEngine.process(stereoInputs, 2, stereoOutputs, 2, 64);
        float maximumDifference = 0.0f;
        for (std::size_t i = 0; i < outputLeft.size(); ++i)
            maximumDifference = std::max(maximumDifference,
                std::abs(outputLeft[i] - outputRight[i]));
        ok &= require(monoEngine.processingChannels() == 1
                          && monoEngine.stats().selectedInputChannel == 1
                          && maximumDifference < 1.0e-7f
                          && peak(outputLeft) > 0.15f,
                      "right-jack guitar is processed once and copied to both outputs");
    }

    engine.setInputRoutingMode(app::InputRoutingMode::autoMono);
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    float stereoDifference = 0.0f;
    for (std::size_t i = 0; i < outputLeft.size(); ++i)
        stereoDifference += std::abs(outputLeft[i] - outputRight[i]);
    auto stats = engine.stats();
    ok &= require(stats.selectedInputChannel == 1 && stats.physicalInputPeaks[0] == 0.0f
                      && stats.physicalInputPeaks[1] > 0.15f
                      && stereoDifference < 1.0e-6f && peak(outputLeft) > 0.15f,
                  "silent left / active WAVIO right input auto-routes coherently to both outputs");

    for (std::size_t i = 0; i < physicalLeft.size(); ++i) {
        physicalLeft[i] = 0.10f * std::sin(0.09f * static_cast<float>(i));
        physicalRight[i] = 0.12f * std::sin(0.09f * static_cast<float>(i));
    }
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    ok &= require(engine.stats().selectedInputChannel == 1,
                  "automatic mono routing uses hysteresis instead of flapping between inputs");

    engine.setInputRoutingMode(app::InputRoutingMode::input1);
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    ok &= require(engine.stats().selectedInputChannel == 0
                      && std::abs(outputLeft[20] - physicalLeft[20]) < 1.0e-6f
                      && std::abs(outputRight[20] - physicalLeft[20]) < 1.0e-6f,
                  "explicit Input 1 duplicates the selected physical input");

    engine.setInputRoutingMode(app::InputRoutingMode::input2);
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    ok &= require(engine.stats().selectedInputChannel == 1
                      && std::abs(outputLeft[20] - physicalRight[20]) < 1.0e-6f,
                  "explicit Input 2 preserves a right-jack guitar signal");

    const float* singleInput[]{physicalLeft.data()};
    engine.process(singleInput, 1, stereoOutputs, 2, 64);
    ok &= require(engine.stats().selectedInputChannel == -1 && peak(outputLeft) == 0.0f,
                  "unavailable explicit Input 2 fails silently instead of selecting the wrong jack");

    engine.setInputRoutingMode(app::InputRoutingMode::stereo);
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    ok &= require(std::abs(outputLeft[20] - physicalLeft[20]) < 1.0e-6f
                      && std::abs(outputRight[20] - physicalRight[20]) < 1.0e-6f,
                  "stereo routing preserves independent physical input channels");

    engine.setInputRoutingMode(app::InputRoutingMode::input1);
    physicalLeft.fill(0.05f);
    physicalLeft[5] = std::numeric_limits<float>::quiet_NaN();
    physicalLeft[6] = std::numeric_limits<float>::infinity();
    physicalLeft[7] = 1.0f;
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    bool finite = true;
    for (const float sample : outputLeft) finite &= std::isfinite(sample);
    stats = engine.stats();
    ok &= require(finite && outputLeft[5] == 0.0f && outputLeft[6] == 0.0f
                      && stats.nonFiniteInputSamples >= 2 && stats.inputClippedSamples >= 1
                      && stats.clippedSamples >= 1 && outputLeft[7] <= 0.98f,
                  "nonfinite ADC data and full-scale clips cannot escape the output safety boundary");

    physicalLeft.fill(0.20f);
    physicalRight.fill(0.0f);
    const auto generationBeforeEdit = engine.graphGeneration();
    ok &= require(engine.setNodeParameter(graph::NodeCategory::utility, 0, 0.5f),
                  "live control edit reaches the prepared active graph");
    engine.process(stereoInputs, 2, stereoOutputs, 2, 64);
    ok &= require(std::abs(outputLeft[20] - 0.10f) < 1.0e-6f
                      && engine.graphGeneration() == generationBeforeEdit,
                  "atomic live parameter edits change audio without graph rebuild or state reset");

    stats = engine.stats();
    ok &= require(stats.performance.callbacks == stats.callbacks
                      && stats.performance.latestBudgetNanoseconds > 0,
                  "every processed device callback publishes allocation-free timing telemetry");
    engine.resetDiagnostics();
    stats = engine.stats();
    ok &= require(stats.callbacks == 0 && stats.clippedSamples == 0
                      && stats.inputClippedSamples == 0 && stats.performance.callbacks == 0,
                  "hardware bring-up counters reset together for a clean measurement run");

    return ok ? 0 : 1;
}
