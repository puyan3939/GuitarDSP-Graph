#include "guitardsp/app/RealtimeAudioEngine.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace guitardsp::app {

float RealtimeAudioEngine::dbToLinear(float db) noexcept {
    return std::pow(10.0f, db / 20.0f);
}

bool RealtimeAudioEngine::configure(double sampleRate,
                                    int maximumBlockSize,
                                    int processingChannels,
                                    const LiveRigSettings& settings) {
    if (sampleRate <= 0.0 || maximumBlockSize <= 0 || processingChannels <= 0) return false;

    auto prepared = prepareLiveRig(settings, sampleRate, maximumBlockSize, processingChannels);
    if (!prepared) return false;

    sampleRate_ = sampleRate;
    maximumBlockSize_ = maximumBlockSize;
    processingChannels_ = processingChannels;
    inputBlock_.resize(processingChannels_, maximumBlockSize_);
    outputBlock_.resize(processingChannels_, maximumBlockSize_);
    graphLatencySamples_.store(prepared->runtime.totalLatencySamples(), std::memory_order_release);
    host_.submit(std::move(prepared));
    autoSelectedInputChannel_ = -1;
    performance_.prepare(sampleRate_);
    configured_ = true;
    return true;
}

bool RealtimeAudioEngine::rebuildRig(const LiveRigSettings& settings) {
    if (!configured_) return false;
    auto prepared = prepareLiveRig(settings, sampleRate_, maximumBlockSize_, processingChannels_);
    if (!prepared) return false;
    graphLatencySamples_.store(prepared->runtime.totalLatencySamples(), std::memory_order_release);
    host_.submit(std::move(prepared));
    return true;
}

void RealtimeAudioEngine::setInputTrimDb(float db) noexcept {
    inputGain_.store(dbToLinear(std::clamp(db, -36.0f, 18.0f)), std::memory_order_release);
}

void RealtimeAudioEngine::setOutputTrimDb(float db) noexcept {
    outputGain_.store(dbToLinear(std::clamp(db, -60.0f, 6.0f)), std::memory_order_release);
}

void RealtimeAudioEngine::setSafetyCeiling(float linear) noexcept {
    safetyCeiling_.store(std::clamp(linear, 0.1f, 1.0f), std::memory_order_release);
}

bool RealtimeAudioEngine::setNodeParameter(graph::NodeCategory category,
                                           std::size_t parameterIndex,
                                           float value) noexcept {
    return configured_ && host_.setCategoryParameter(category, parameterIndex, value);
}

void RealtimeAudioEngine::resetDiagnostics() noexcept {
    callbacks_.store(0, std::memory_order_relaxed);
    clippedSamples_.store(0, std::memory_order_relaxed);
    inputClippedSamples_.store(0, std::memory_order_relaxed);
    nonFiniteInputSamples_.store(0, std::memory_order_relaxed);
    nonFiniteOutputSamples_.store(0, std::memory_order_relaxed);
    inputPeak_.store(0.0f, std::memory_order_relaxed);
    outputPeak_.store(0.0f, std::memory_order_relaxed);
    physicalInputPeak1_.store(0.0f, std::memory_order_relaxed);
    physicalInputPeak2_.store(0.0f, std::memory_order_relaxed);
    performance_.reset();
}

RealtimeAudioStats RealtimeAudioEngine::stats() const noexcept {
    return {
        callbacks_.load(std::memory_order_relaxed),
        clippedSamples_.load(std::memory_order_relaxed),
        inputPeak_.load(std::memory_order_relaxed),
        outputPeak_.load(std::memory_order_relaxed),
        graphLatencySamples_.load(std::memory_order_acquire),
        {physicalInputPeak1_.load(std::memory_order_relaxed),
         physicalInputPeak2_.load(std::memory_order_relaxed)},
        inputClippedSamples_.load(std::memory_order_relaxed),
        nonFiniteInputSamples_.load(std::memory_order_relaxed),
        nonFiniteOutputSamples_.load(std::memory_order_relaxed),
        selectedInputChannel_.load(std::memory_order_relaxed),
        inputRoutingMode_.load(std::memory_order_acquire),
        performance_.snapshot()
    };
}

void RealtimeAudioEngine::process(const float* const* inputChannels,
                                  int numInputChannels,
                                  float* const* outputChannels,
                                  int numOutputChannels,
                                  int numSamples) noexcept {
    if (numSamples <= 0) return;

    for (int ch = 0; ch < numOutputChannels; ++ch) {
        if (outputChannels != nullptr && outputChannels[ch] != nullptr)
            std::fill(outputChannels[ch], outputChannels[ch] + numSamples, 0.0f);
    }

    if (!configured_ || outputChannels == nullptr || numOutputChannels <= 0) return;

    const auto callbackStarted = std::chrono::steady_clock::now();
    const float inputGain = inputGain_.load(std::memory_order_acquire);
    const float outputGain = outputGain_.load(std::memory_order_acquire);
    const float ceiling = safetyCeiling_.load(std::memory_order_acquire);
    const bool muted = muted_.load(std::memory_order_acquire);
    const auto routingMode = inputRoutingMode_.load(std::memory_order_acquire);

    std::array<float, 2> physicalPeaks{};
    std::uint64_t clippedInput = 0;
    std::uint64_t nonFiniteInput = 0;
    const int physicalChannels = inputChannels == nullptr
        ? 0 : std::min(std::max(numInputChannels, 0), 2);
    for (int ch = 0; ch < physicalChannels; ++ch) {
        const float* source = inputChannels[ch];
        if (source == nullptr) continue;
        for (int i = 0; i < numSamples; ++i) {
            const float sample = source[i];
            if (!std::isfinite(sample)) {
                ++nonFiniteInput;
                continue;
            }
            const float magnitude = std::abs(sample);
            physicalPeaks[static_cast<std::size_t>(ch)] =
                std::max(physicalPeaks[static_cast<std::size_t>(ch)], magnitude);
            if (magnitude >= 0.999f) ++clippedInput;
        }
    }

    auto channelAvailable = [&](int channel) noexcept {
        return channel >= 0 && channel < physicalChannels
            && inputChannels != nullptr && inputChannels[channel] != nullptr;
    };

    int selectedChannel = -1;
    if (routingMode == InputRoutingMode::input1) {
        selectedChannel = channelAvailable(0) ? 0 : -1;
    } else if (routingMode == InputRoutingMode::input2) {
        selectedChannel = channelAvailable(1) ? 1 : -1;
    } else if (routingMode == InputRoutingMode::autoMono) {
        constexpr float minimumDetectionPeak = 1.0e-4f;
        constexpr float switchRatio = 1.6f;

        if (!channelAvailable(autoSelectedInputChannel_))
            autoSelectedInputChannel_ = channelAvailable(0) ? 0
                                        : channelAvailable(1) ? 1 : -1;

        if (channelAvailable(0) && channelAvailable(1)) {
            const int strongest = physicalPeaks[1] > physicalPeaks[0] ? 1 : 0;
            const int current = autoSelectedInputChannel_;
            if (strongest != current
                && physicalPeaks[static_cast<std::size_t>(strongest)] > minimumDetectionPeak
                && physicalPeaks[static_cast<std::size_t>(strongest)]
                    > physicalPeaks[static_cast<std::size_t>(current)] * switchRatio) {
                autoSelectedInputChannel_ = strongest;
            }
        }
        selectedChannel = autoSelectedInputChannel_;
    } else {
        selectedChannel = channelAvailable(0) ? 0 : channelAvailable(1) ? 1 : -1;
    }

    float callbackInputPeak = 0.0f;
    float callbackOutputPeak = 0.0f;
    std::uint64_t clipped = 0;
    std::uint64_t nonFiniteOutput = 0;

    int offset = 0;
    while (offset < numSamples) {
        const int blockSamples = std::min(maximumBlockSize_, numSamples - offset);
        inputBlock_.clear(blockSamples);
        outputBlock_.clear(blockSamples);

        for (int ch = 0; ch < processingChannels_; ++ch) {
            float* destination = inputBlock_.channel(ch);
            const float* source = nullptr;
            if (routingMode == InputRoutingMode::stereo) {
                const int sourceChannel = physicalChannels == 1
                    ? 0 : std::min(ch, physicalChannels - 1);
                if (channelAvailable(sourceChannel)) source = inputChannels[sourceChannel];
            } else if (channelAvailable(selectedChannel)) {
                source = inputChannels[selectedChannel];
            }
            if (source == nullptr) continue;
            source += offset;
            for (int i = 0; i < blockSamples; ++i) {
                const float sample = std::isfinite(source[i]) ? source[i] * inputGain : 0.0f;
                if (!std::isfinite(sample)) {
                    destination[i] = 0.0f;
                    continue;
                }
                destination[i] = sample;
                callbackInputPeak = std::max(callbackInputPeak, std::abs(sample));
            }
        }

        host_.process(inputBlock_, outputBlock_, blockSamples);

        for (int ch = 0; ch < numOutputChannels; ++ch) {
            float* destination = outputChannels[ch];
            if (destination == nullptr) continue;
            destination += offset;
            const int sourceChannel = std::min(ch, processingChannels_ - 1);
            const float* source = outputBlock_.channel(sourceChannel);
            for (int i = 0; i < blockSamples; ++i) {
                float sample = muted ? 0.0f : source[i] * outputGain;
                if (!std::isfinite(sample)) {
                    sample = 0.0f;
                    ++nonFiniteOutput;
                } else if (sample > ceiling) {
                    sample = ceiling;
                    ++clipped;
                } else if (sample < -ceiling) {
                    sample = -ceiling;
                    ++clipped;
                }
                destination[i] = sample;
                callbackOutputPeak = std::max(callbackOutputPeak, std::abs(sample));
            }
        }

        offset += blockSamples;
    }

    callbacks_.fetch_add(1, std::memory_order_relaxed);
    clippedSamples_.fetch_add(clipped, std::memory_order_relaxed);
    inputClippedSamples_.fetch_add(clippedInput, std::memory_order_relaxed);
    nonFiniteInputSamples_.fetch_add(nonFiniteInput, std::memory_order_relaxed);
    nonFiniteOutputSamples_.fetch_add(nonFiniteOutput, std::memory_order_relaxed);
    inputPeak_.store(callbackInputPeak, std::memory_order_relaxed);
    outputPeak_.store(callbackOutputPeak, std::memory_order_relaxed);
    physicalInputPeak1_.store(physicalPeaks[0], std::memory_order_relaxed);
    physicalInputPeak2_.store(physicalPeaks[1], std::memory_order_relaxed);
    selectedInputChannel_.store(selectedChannel, std::memory_order_relaxed);

    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - callbackStarted).count();
    performance_.recordCallback(numSamples,
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, elapsed)));
}

} // namespace guitardsp::app
