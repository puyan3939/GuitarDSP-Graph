#include "guitardsp/hq/YehSmithToneStack.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

std::vector<float> render(hq::ToneStackFamily family, float bass, float mid, float treble,
                          double sampleRate, double hz, int samples) {
    hq::YehSmithToneStack stack;
    stack.prepare(sampleRate, family);
    stack.setControls(bass, mid, treble);
    std::vector<float> out(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const float x = 0.25f * static_cast<float>(std::sin(2.0 * std::numbers::pi * hz * static_cast<double>(i) / sampleRate));
        out[static_cast<std::size_t>(i)] = stack.process(x);
    }
    return out;
}

float rmsDifference(const std::vector<float>& a, const std::vector<float>& b, int start) {
    double sum = 0.0;
    int n = 0;
    for (int i = start; i < static_cast<int>(std::min(a.size(), b.size())); ++i) {
        const double d = static_cast<double>(a[static_cast<std::size_t>(i)]) - static_cast<double>(b[static_cast<std::size_t>(i)]);
        sum += d*d;
        ++n;
    }
    return n > 0 ? static_cast<float>(std::sqrt(sum / static_cast<double>(n))) : 0.0f;
}
}

int main() {
    bool ok = true;
    constexpr double sr = 48000.0;
    constexpr int samples = 8192;

    hq::YehSmithToneStack stack;
    stack.prepare(sr, hq::ToneStackFamily::reference);
    stack.setControls(0.5f, 0.5f, 0.5f);
    float peak = 0.0f;
    bool finite = true;
    for (int i = 0; i < samples; ++i) {
        const float x = (i == 0) ? 1.0f : 0.0f;
        const float y = stack.process(x);
        finite &= std::isfinite(y);
        peak = std::max(peak, std::abs(y));
    }
    ok &= require(finite && peak < 2.0f, "Yeh-Smith tone stack impulse is stable and bounded");

    const auto british = render(hq::ToneStackFamily::british, 0.5f, 0.5f, 0.5f, sr, 1000.0, samples);
    const auto american = render(hq::ToneStackFamily::american, 0.5f, 0.5f, 0.5f, sr, 1000.0, samples);
    ok &= require(rmsDifference(british, american, samples / 2) > 1.0e-5f,
                  "British and American exact FMV component families differ");

    const auto midLow = render(hq::ToneStackFamily::reference, 0.5f, 0.1f, 0.5f, sr, 700.0, samples);
    const auto midHigh = render(hq::ToneStackFamily::reference, 0.5f, 0.9f, 0.5f, sr, 700.0, samples);
    ok &= require(rmsDifference(midLow, midHigh, samples / 2) > 1.0e-5f,
                  "FMV middle control changes response");

    const auto trebleLow = render(hq::ToneStackFamily::reference, 0.5f, 0.5f, 0.05f, sr, 5000.0, samples);
    const auto trebleHigh = render(hq::ToneStackFamily::reference, 0.5f, 0.5f, 0.95f, sr, 5000.0, samples);
    ok &= require(rmsDifference(trebleLow, trebleHigh, samples / 2) > 1.0e-5f,
                  "FMV treble control changes response");

    return ok ? 0 : 1;
}
