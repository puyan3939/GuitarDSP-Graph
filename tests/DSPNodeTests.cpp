#include "guitardsp/dsp/BasicNodes.h"
#include <cmath>
#include <iostream>

using namespace guitardsp::graph;
using namespace guitardsp::dsp;

namespace {
bool require(bool c, const char* m) { std::cout << (c ? "PASS " : "FAIL ") << m << '\n'; return c; }
float rms(const AudioBuffer& b, int n) {
    double s = 0.0; int count = 0;
    for (int ch = 0; ch < b.channels(); ++ch) for (int i = 0; i < n; ++i) { const float x = b.channel(ch)[i]; s += x*x; ++count; }
    return count ? static_cast<float>(std::sqrt(s / count)) : 0.0f;
}
}

int main() {
    bool ok = true; constexpr int block = 512; constexpr double sr = 48000.0;
    PrepareSpec spec{sr, block, 2, ProcessingQuality::high};
    AudioBuffer in(2, block), out(2, block);

    for (int i = 0; i < block; ++i) {
        const float x = 0.8f * std::sin(2.0 * 3.141592653589793 * 440.0 * i / sr);
        in.channel(0)[i] = x; in.channel(1)[i] = x;
    }
    CompressorNode comp; comp.prepare(spec); comp.setThresholdDb(-24.0f); comp.setRatio(8.0f); comp.setAttackMs(1.0f); comp.setReleaseMs(80.0f);
    comp.process(in, out, block);
    ok &= require(std::isfinite(rms(out, block)), "compressor output finite");
    ok &= require(rms(out, block) < rms(in, block), "compressor reduces high-level RMS");

    TransientEnhancerNode transient; transient.prepare(spec); transient.setAmount(0.8f); transient.setBrightnessHz(1800.0f);
    in.clear(); in.channel(0)[0] = 1.0f; in.channel(1)[0] = 1.0f;
    transient.process(in, out, block);
    ok &= require(std::abs(out.channel(0)[0]) > std::abs(in.channel(0)[0]), "transient enhancer lifts attack");
    ok &= require(std::abs(out.channel(0)[200]) < 1.0e-4f, "transient enhancer does not create long DC tail");

    OnePoleFilterNode hp(OnePoleFilterNode::Mode::highPass, 1000.0f); hp.prepare(spec);
    for (int i = 0; i < block; ++i) { in.channel(0)[i] = 1.0f; in.channel(1)[i] = 1.0f; }
    hp.process(in, out, block);
    ok &= require(std::abs(out.channel(0)[block-1]) < 1.0e-3f, "high-pass rejects DC");

    return ok ? 0 : 1;
}
