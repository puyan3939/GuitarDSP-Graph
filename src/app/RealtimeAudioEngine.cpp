#include "guitardsp/app/RealtimeAudioEngine.h"

#include <algorithm>
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

RealtimeAudioStats RealtimeAudioEngine::stats() const noexcept {
    return {
        callbacks_.load(std::memory_order_relaxed),
        clippedSamples_.load(std::memory_order_relaxed),
        inputPeak_.load(std::memory_order_relaxed),
        outputPeak_.load(std::memory_order_relaxed),
        graphLatencySamples_.load(std::memory_order_acquire)
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

    const float inputGain = inputGain_.load(std::memory_order_acquire);
    const float outputGain = outputGain_.load(std::memory_order_acquire);
    const float ceiling = safetyCeiling_.load(std::memory_order_acquire);
    const bool muted = muted_.load(std::memory_order_acquire);

    float callbackInputPeak = 0.0f;
    float callbackOutputPeak = 0.0f;
    std::uint64_t clipped = 0;

    int offset = 0;
    while (offset < numSamples) {
        const int blockSamples = std::min(maximumBlockSize_, numSamples - offset);
        inputBlock_.clear(blockSamples);
        outputBlock_.clear(blockSamples);

        for (int ch = 0; ch < processingChannels_; ++ch) {
            float* destination = inputBlock_.channel(ch);
            const float* source = nullptr;
            if (inputChannels != nullptr && numInputChannels > 0) {
                const int sourceChannel = numInputChannels == 1 ? 0 : std::min(ch, numInputChannels - 1);
                source = inputChannels[sourceChannel];
            }
            if (source == nullptr) continue;
            source += offset;
            for (int i = 0; i < blockSamples; ++i) {
                const float sample = source[i] * inputGain;
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
                if (sample > ceiling) {
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
    inputPeak_.store(callbackInputPeak, std::memory_order_relaxed);
    outputPeak_.store(callbackOutputPeak, std::memory_order_relaxed);
}

} // namespace guitardsp::app
