#pragma once

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_gui_basics/juce_gui_basics.h>

#include "guitardsp/hq/FFT.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <vector>

namespace guitardsp::app {

// Message-thread-only spectrum analyser. Samples are handed in via
// pushSamples(), which is expected to be called from MainComponent's
// timerCallback() with data drained from an AudioTapFifo -- the same
// FIFO/UI pattern used for the waveform display. The FFT (windowing,
// guitardsp::hq::Radix2FFT transform, dB conversion) only ever runs here,
// never from the audio callback.
class SpectrumAnalyserComponent final : public juce::Component {
public:
    explicit SpectrumAnalyserComponent(int fftSize = 2048) { setFftSize(fftSize); }

    void setFftSize(int fftSize) {
        fftSize_ = static_cast<int>(guitardsp::hq::nextPowerOfTwo(
            static_cast<std::size_t>(std::max(64, fftSize))));

        window_.assign(static_cast<std::size_t>(fftSize_), 0.0f);
        double windowSum = 0.0;
        for (int i = 0; i < fftSize_; ++i) {
            const float w = 0.5f - 0.5f * std::cos(2.0f * std::numbers::pi_v<float>
                * static_cast<float>(i) / static_cast<float>(fftSize_ - 1));
            window_[static_cast<std::size_t>(i)] = w;
            windowSum += w;
        }
        windowGain_ = static_cast<float>(windowSum / fftSize_);

        accumulator_.assign(static_cast<std::size_t>(fftSize_), 0.0f);
        accumulated_ = 0;
        magnitudesDb_.assign(static_cast<std::size_t>(fftSize_ / 2), kFloorDb);
        peakDb_.assign(static_cast<std::size_t>(fftSize_ / 2), kFloorDb);
    }

    void setSampleRate(double sampleRate) noexcept { sampleRate_ = sampleRate; }

    void setColours(juce::Colour background, juce::Colour trace) {
        background_ = background;
        trace_ = trace;
    }

    // Message thread only -- see class comment. Accumulates samples and
    // runs one FFT frame each time fftSize() new samples have arrived.
    void pushSamples(const float* samples, int numSamples) {
        if (samples == nullptr || numSamples <= 0 || fftSize_ <= 0) return;
        for (int i = 0; i < numSamples; ++i) {
            accumulator_[static_cast<std::size_t>(accumulated_)] = samples[i];
            if (++accumulated_ >= fftSize_) {
                runFft();
                accumulated_ = 0;
            }
        }
    }

    void resetAnalysis() {
        accumulated_ = 0;
        std::fill(magnitudesDb_.begin(), magnitudesDb_.end(), kFloorDb);
        std::fill(peakDb_.begin(), peakDb_.end(), kFloorDb);
        repaint();
    }

    void paint(juce::Graphics& g) override {
        const auto bounds = getLocalBounds().toFloat();
        g.setColour(background_);
        g.fillRect(bounds);
        if (magnitudesDb_.size() < 2 || sampleRate_ <= 0.0 || bounds.getWidth() < 4.0f) return;

        const float minFreq = 20.0f;
        const float maxFreq = std::max(minFreq + 1.0f,
            static_cast<float>(std::min(sampleRate_ * 0.5, 20000.0)));
        const float logMin = std::log10(minFreq);
        const float logMax = std::log10(maxFreq);
        const auto xForFreq = [&](float freq) {
            freq = std::clamp(freq, minFreq, maxFreq);
            return bounds.getX() + bounds.getWidth()
                * (std::log10(freq) - logMin) / (logMax - logMin);
        };
        const auto yForDb = [&](float db) {
            const float t = std::clamp((db - kFloorDb) / (0.0f - kFloorDb), 0.0f, 1.0f);
            return bounds.getBottom() - t * bounds.getHeight();
        };

        juce::Path tracePath;
        juce::Path peakPath;
        bool traceStarted = false;
        bool peakStarted = false;
        const std::size_t numBins = magnitudesDb_.size();
        for (std::size_t bin = 1; bin < numBins; ++bin) {
            const float freq = static_cast<float>(bin) * static_cast<float>(sampleRate_)
                / static_cast<float>(fftSize_);
            if (freq < minFreq || freq > maxFreq) continue;
            const float x = xForFreq(freq);

            const float y = yForDb(magnitudesDb_[bin]);
            if (!traceStarted) { tracePath.startNewSubPath(x, y); traceStarted = true; }
            else tracePath.lineTo(x, y);

            const float py = yForDb(peakDb_[bin]);
            if (!peakStarted) { peakPath.startNewSubPath(x, py); peakStarted = true; }
            else peakPath.lineTo(x, py);
        }

        g.setColour(trace_.withAlpha(0.35f));
        g.strokePath(peakPath, juce::PathStrokeType(1.0f));
        g.setColour(trace_);
        g.strokePath(tracePath, juce::PathStrokeType(1.5f));
    }

private:
    static constexpr float kFloorDb = -100.0f;
    static constexpr float kPeakDecayDbPerFrame = 0.5f;

    void runFft() {
        std::vector<std::complex<float>> spectrum(static_cast<std::size_t>(fftSize_));
        for (int i = 0; i < fftSize_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            spectrum[idx] = accumulator_[idx] * window_[idx];
        }
        guitardsp::hq::Radix2FFT::transform(spectrum, false);

        const float normalise = (static_cast<float>(fftSize_) * 0.5f)
            * std::max(windowGain_, 1.0e-6f);
        const std::size_t numBins = magnitudesDb_.size();
        for (std::size_t bin = 0; bin < numBins; ++bin) {
            const float magnitude = std::abs(spectrum[bin]) / normalise;
            const float db = magnitude > 1.0e-9f
                ? juce::Decibels::gainToDecibels(magnitude, kFloorDb) : kFloorDb;
            magnitudesDb_[bin] = db;
            peakDb_[bin] = std::max(peakDb_[bin] - kPeakDecayDbPerFrame, db);
        }
        repaint();
    }

    double sampleRate_ = 48000.0;
    int fftSize_ = 2048;
    int accumulated_ = 0;
    float windowGain_ = 0.5f;
    std::vector<float> window_;
    std::vector<float> accumulator_;
    std::vector<float> magnitudesDb_;
    std::vector<float> peakDb_;
    juce::Colour background_{juce::Colour::fromRGB(20, 29, 34)};
    juce::Colour trace_{juce::Colours::lightseagreen};
};

} // namespace guitardsp::app
