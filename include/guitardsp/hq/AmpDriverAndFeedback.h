#pragma once

#include "ADAA.h"
#include "CircuitPrimitives.h"

#include <algorithm>
#include <cmath>

namespace guitardsp::hq {

enum class ToneStackDriverType { reference = 0, cathodeFollower = 1, plateDriven = 2 };
enum class FeedbackVoicing { reference = 0, british = 1, american = 2 };

// Engineering cathode-follower driver. The low source impedance and slight
// asymmetric compression are represented explicitly, while the full triode
// operating point remains a later measured-fit target.
class CathodeFollowerDriver {
public:
    void prepare(double sampleRate) noexcept {
        dc_.prepare(sampleRate);
        dc_.setLowpass(5.0f);
        reset();
    }
    void reset() noexcept { dc_.reset(); nonlinearity_.reset(); }
    void setDrive(float drive) noexcept { drive_ = std::clamp(drive, 0.25f, 4.0f); }
    float process(float x) noexcept {
        const float centered = x - dc_.processLowpass(x);
        const float shifted = drive_ * centered + 0.035f;
        return 0.92f * (nonlinearity_.process(shifted) - 0.034985f);
    }
private:
    OnePole dc_;
    ADAATanh nonlinearity_;
    float drive_ = 1.0f;
};

// Higher-source-impedance plate-driving approximation. The extra HF loading is
// intentional and lets the exact FMV stack see a meaningfully different source
// condition than the cathode-follower path.
class PlateToneStackDriver {
public:
    void prepare(double sampleRate) noexcept {
        sourceLowpass_.prepare(sampleRate);
        sourceLowpass_.setLowpass(11500.0f);
        dc_.prepare(sampleRate);
        dc_.setLowpass(4.0f);
        reset();
    }
    void reset() noexcept { sourceLowpass_.reset(); dc_.reset(); nonlinearity_.reset(); }
    void setDrive(float drive) noexcept { drive_ = std::clamp(drive, 0.25f, 4.0f); }
    float process(float x) noexcept {
        const float centered = x - dc_.processLowpass(x);
        const float loaded = sourceLowpass_.processLowpass(centered);
        return 0.78f * nonlinearity_.process(drive_ * loaded);
    }
private:
    OnePole sourceLowpass_, dc_;
    ADAATanh nonlinearity_;
    float drive_ = 1.0f;
};

// Topology-aware feedback loop. The amount control remains continuous, while
// bandwidth/presence behavior differs between British and American families.
class VoicedNegativeFeedbackLoop {
public:
    void prepare(double sampleRate) noexcept {
        feedback_.prepare(sampleRate);
        presence_.prepare(sampleRate);
        setVoicing(FeedbackVoicing::reference);
        reset();
    }
    void reset() noexcept { feedback_.reset(); presence_.reset(); feedbackState_ = 0.0f; }
    void setAmount(float amount) noexcept { amount_ = std::clamp(amount, 0.0f, 0.9f); }
    void setPresence(float presence) noexcept { presenceAmount_ = std::clamp(presence, 0.0f, 1.0f); }
    void setVoicing(FeedbackVoicing voicing) noexcept {
        voicing_ = voicing;
        float feedbackHz = 7000.0f;
        float presenceHz = 3200.0f;
        if (voicing == FeedbackVoicing::british) {
            feedbackHz = 6200.0f;
            presenceHz = 3600.0f;
            familyScale_ = 0.88f;
        } else if (voicing == FeedbackVoicing::american) {
            feedbackHz = 8500.0f;
            presenceHz = 2800.0f;
            familyScale_ = 1.08f;
        } else {
            familyScale_ = 1.0f;
        }
        feedback_.setLowpass(feedbackHz);
        presence_.setLowpass(presenceHz);
    }
    float drive(float input) noexcept {
        const float feedback = feedbackState_ * amount_ * familyScale_;
        const float presenceLift = presenceAmount_ * 0.34f * (feedbackState_ - presenceState_);
        return input - feedback + presenceLift;
    }
    void observe(float output) noexcept {
        feedbackState_ = feedback_.processLowpass(output);
        presenceState_ = presence_.processLowpass(feedbackState_);
    }
private:
    OnePole feedback_, presence_;
    FeedbackVoicing voicing_ = FeedbackVoicing::reference;
    float feedbackState_ = 0.0f;
    float presenceState_ = 0.0f;
    float amount_ = 0.32f;
    float presenceAmount_ = 0.5f;
    float familyScale_ = 1.0f;
};

} // namespace guitardsp::hq
