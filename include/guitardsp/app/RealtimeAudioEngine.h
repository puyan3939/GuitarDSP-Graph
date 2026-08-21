#pragma once

#include "LiveRig.h"
#include "RealtimePerformanceMonitor.h"
#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/graph/RealtimeGraphHost.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace guitardsp::app {

enum class InputRoutingMode : std::uint8_t {
    autoMono,
    input1,
    input2,
    stereo
};

struct RealtimeAudioStats {
    std::uint64_t callbacks = 0;
    std::uint64_t clippedSamples = 0;
    float inputPeak = 0.0f;
    float outputPeak = 0.0f;
    int graphLatencySamples = 0;
    std::array<float, 2> physicalInputPeaks{};
    std::uint64_t inputClippedSamples = 0;
    std::uint64_t nonFiniteInputSamples = 0;
    std::uint64_t nonFiniteOutputSamples = 0;
    int selectedInputChannel = -1;
    InputRoutingMode inputRoutingMode = InputRoutingMode::autoMono;
    RealtimePerformanceSnapshot performance{};
};

// JUCE-independent realtime bridge. Device callbacks hand raw channel pointers to
// this class; all graph buffers are preallocated in configure(), and process() does
// no allocation, file I/O, topology mutation, or graph deletion.
class RealtimeAudioEngine {
public:
    bool configure(double sampleRate,
                   int maximumBlockSize,
                   int processingChannels,
                   const LiveRigSettings& settings);

    bool rebuildRig(const LiveRigSettings& settings);

    void process(const float* const* inputChannels,
                 int numInputChannels,
                 float* const* outputChannels,
                 int numOutputChannels,
                 int numSamples) noexcept;

    void setInputTrimDb(float db) noexcept;
    void setOutputTrimDb(float db) noexcept;
    void setMuted(bool muted) noexcept { muted_.store(muted, std::memory_order_release); }
    void setSafetyCeiling(float linear) noexcept;
    void setInputRoutingMode(InputRoutingMode mode) noexcept {
        inputRoutingMode_.store(mode, std::memory_order_release);
    }
    bool setNodeParameter(graph::NodeCategory category,
                          std::size_t parameterIndex,
                          float value) noexcept;
    void resetDiagnostics() noexcept;

    [[nodiscard]] double sampleRate() const noexcept { return sampleRate_; }
    [[nodiscard]] int maximumBlockSize() const noexcept { return maximumBlockSize_; }
    [[nodiscard]] int processingChannels() const noexcept { return processingChannels_; }
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] RealtimeAudioStats stats() const noexcept;
    [[nodiscard]] std::uint64_t graphGeneration() const noexcept { return host_.generation(); }

    // Message/control thread only. Destruction is intentionally kept out of the
    // device callback; callers should drain retired prepared graphs from a UI timer.
    std::size_t collectRetired() noexcept { return host_.collectRetired(); }

private:
    static float dbToLinear(float db) noexcept;

    graph::RealtimeGraphHost host_;
    graph::AudioBuffer inputBlock_;
    graph::AudioBuffer outputBlock_;
    double sampleRate_ = 0.0;
    int maximumBlockSize_ = 0;
    int processingChannels_ = 0;
    bool configured_ = false;

    std::atomic<float> inputGain_{1.0f};
    std::atomic<float> outputGain_{0.25118864f}; // -12 dB safe first-listen default
    std::atomic<float> safetyCeiling_{0.98f};
    std::atomic<bool> muted_{false};
    std::atomic<InputRoutingMode> inputRoutingMode_{InputRoutingMode::autoMono};

    std::atomic<std::uint64_t> callbacks_{0};
    std::atomic<std::uint64_t> clippedSamples_{0};
    std::atomic<std::uint64_t> inputClippedSamples_{0};
    std::atomic<std::uint64_t> nonFiniteInputSamples_{0};
    std::atomic<std::uint64_t> nonFiniteOutputSamples_{0};
    std::atomic<float> inputPeak_{0.0f};
    std::atomic<float> outputPeak_{0.0f};
    std::atomic<float> physicalInputPeak1_{0.0f};
    std::atomic<float> physicalInputPeak2_{0.0f};
    std::atomic<int> selectedInputChannel_{-1};
    std::atomic<int> graphLatencySamples_{0};
    int autoSelectedInputChannel_ = -1;
    RealtimePerformanceMonitor performance_;
};

} // namespace guitardsp::app
