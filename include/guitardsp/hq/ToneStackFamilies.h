#pragma once

#include "CircuitPrimitives.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

enum class ToneStackFamily { reference = 0, british = 1, american = 2 };

struct ToneStackNominalValues {
    float slopeResistanceOhms = 33000.0f;
    float trebleCapFarads = 470.0e-12f;
    float midCapFarads = 22.0e-9f;
    float bassCapFarads = 22.0e-9f;
    float treblePotOhms = 250000.0f;
    float midPotOhms = 25000.0f;
    float bassPotOhms = 1000000.0f;
};

inline ToneStackNominalValues nominalToneStack(ToneStackFamily family) noexcept {
    switch (family) {
        case ToneStackFamily::british:
            // Circuit-value-informed British/plexi family starting point.
            return {33000.0f, 470.0e-12f, 22.0e-9f, 22.0e-9f,
                    250000.0f, 25000.0f, 1000000.0f};
        case ToneStackFamily::american:
            // Circuit-value-informed American blackface family starting point.
            return {100000.0f, 250.0e-12f, 47.0e-9f, 100.0e-9f,
                    250000.0f, 10000.0f, 250000.0f};
        case ToneStackFamily::reference:
        default:
            return {56000.0f, 330.0e-12f, 33.0e-9f, 47.0e-9f,
                    250000.0f, 25000.0f, 500000.0f};
    }
}

// Interactive passive-stack approximation whose break frequencies and control
// interaction are derived from the selected component family. It intentionally
// preserves a stable realtime contract while a full nodal MNA tone-stack solve
// can later replace the transfer core without changing amp nodes.
class InteractiveToneStack {
public:
    void prepare(double sampleRate, ToneStackFamily family = ToneStackFamily::reference) noexcept {
        sampleRate_ = std::max(1.0, sampleRate);
        family_ = family;
        updateNetwork();
        reset();
    }

    void reset() noexcept {
        low_.reset();
        upper_.reset();
    }

    void setFamily(ToneStackFamily family) noexcept {
        if (family_ == family) return;
        family_ = family;
        updateNetwork();
    }

    void setControls(float bass, float mid, float treble) noexcept {
        bass_ = std::clamp(bass, 0.0f, 1.0f);
        mid_ = std::clamp(mid, 0.0f, 1.0f);
        treble_ = std::clamp(treble, 0.0f, 1.0f);
    }

    [[nodiscard]] ToneStackFamily family() const noexcept { return family_; }
    [[nodiscard]] ToneStackNominalValues nominalValues() const noexcept { return nominalToneStack(family_); }

    float process(float x) noexcept {
        const float low = low_.processLowpass(x);
        const float belowUpper = upper_.processLowpass(x);
        const float midBand = belowUpper - low;
        const float high = x - belowUpper;

        // Passive stacks are lossy. Controls interact rather than acting as three
        // independent active EQ gains. Family-specific coefficients reproduce the
        // broad British mid-forward vs American mid-scooped behavior.
        float bassGain = 0.15f + 1.35f * bass_;
        float midGain = 0.10f + 1.30f * mid_;
        float trebleGain = 0.12f + 1.45f * treble_;

        if (family_ == ToneStackFamily::british) {
            midGain *= 0.82f + 0.28f * treble_;
            bassGain *= 0.88f + 0.18f * (1.0f - mid_);
            trebleGain *= 0.86f + 0.22f * mid_;
            return 0.39f * (bassGain * low + 1.12f * midGain * midBand + trebleGain * high);
        }
        if (family_ == ToneStackFamily::american) {
            const float scoop = 0.48f + 0.52f * mid_;
            midGain *= scoop;
            bassGain *= 0.92f + 0.20f * treble_;
            trebleGain *= 0.96f + 0.18f * bass_;
            return 0.34f * (1.08f * bassGain * low + 0.72f * midGain * midBand + 1.08f * trebleGain * high);
        }
        return 0.37f * (bassGain * low + midGain * midBand + trebleGain * high);
    }

private:
    void updateNetwork() noexcept {
        const auto v = nominalToneStack(family_);
        // Characteristic frequencies from the dominant RC products. Clamp to the
        // audio range because the reduced model collapses the full passive network
        // into two state variables while retaining component-family influence.
        const float lowHz = std::clamp(1.0f / (6.28318530718f * v.bassPotOhms * v.bassCapFarads), 70.0f, 420.0f);
        const float highHz = std::clamp(1.0f / (6.28318530718f * v.slopeResistanceOhms * v.trebleCapFarads), 900.0f, 5200.0f);
        low_.prepare(sampleRate_);
        low_.setLowpass(lowHz);
        upper_.prepare(sampleRate_);
        upper_.setLowpass(highHz);
    }

    double sampleRate_ = 48000.0;
    ToneStackFamily family_ = ToneStackFamily::reference;
    OnePole low_, upper_;
    float bass_ = 0.5f;
    float mid_ = 0.5f;
    float treble_ = 0.5f;
};

} // namespace guitardsp::hq
