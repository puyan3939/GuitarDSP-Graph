#include "guitardsp/graph/LockFreeTapBuffer.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/dsp/DriveNodes.h"
#include <cmath>
#include <iostream>

using namespace guitardsp::graph;
using namespace guitardsp::dsp;

namespace {
bool require(bool c, const char* m) { std::cout << (c ? "PASS " : "FAIL ") << m << '\n'; return c; }
float rms(const AudioBuffer& b, int n) { double s=0; int count=0; for(int ch=0;ch<b.channels();++ch)for(int i=0;i<n;++i){const float x=b.channel(ch)[i];s+=x*x;++count;}return count?static_cast<float>(std::sqrt(s/count)):0.0f; }
}

int main() {
    bool ok = true; constexpr int block=256; constexpr double sr=48000.0;
    auto registry = NodeRegistry::createBuiltins();
    ok &= require(registry.create("drive.ds1_prototype") != nullptr, "registry creates DS-1 prototype");
    ok &= require(registry.create("drive.ds1_hq") != nullptr, "registry creates HQ DS-1");
    ok &= require(registry.create("drive.ts808_hq") != nullptr, "registry creates HQ TS808");
    ok &= require(registry.create("drive.bd2_hq") != nullptr, "registry creates HQ BD-2");
    ok &= require(registry.create("amp.reference_hq") != nullptr, "registry creates reference HQ amp");
    ok &= require(registry.create("drive.preamp_circuit_hq") != nullptr, "registry creates HQ preamp circuit");
    ok &= require(registry.create("cab.partitioned_hq") != nullptr, "registry creates partitioned HQ cab");
    ok &= require(registry.create("cab.speaker_dynamics_hq") != nullptr, "registry creates HQ speaker dynamics");
    ok &= require(registry.create("route.split") != nullptr, "registry creates split node");
    ok &= require(registry.create("time.digital_delay") != nullptr, "registry creates digital delay");
    ok &= require(registry.create("does.not.exist") == nullptr, "registry rejects unknown node type");

    DS1PrototypeNode drive; PrepareSpec spec{sr,block,2,ProcessingQuality::high}; drive.prepare(spec);
    AudioBuffer in(2,block),out(2,block);
    for(int i=0;i<block;++i){const float x=0.08f*std::sin(2.0*3.141592653589793*440.0*i/sr);in.channel(0)[i]=x;in.channel(1)[i]=x;}
    const float before=rms(in,block); drive.process(in,out,block); const float after=rms(out,block);
    ok &= require(std::isfinite(after) && after > 1.0e-6f, "DS-1 prototype output finite");
    ok &= require(std::abs(after-before) > 1.0e-4f, "DS-1 prototype changes signal");
    in.clear(); drive.reset(); drive.process(in,out,block); ok &= require(rms(out,block) < 1.0e-8f, "DS-1 prototype preserves digital silence");

    LockFreeTapBuffer tap; tap.prepare(2,1024); AudioBuffer source(2,block),latest(2,64); source.clear();
    for(int i=0;i<block;++i){source.channel(0)[i]=static_cast<float>(i);source.channel(1)[i]=-static_cast<float>(i);}
    tap.push(source,block); const int copied=tap.readLatest(latest,64);
    ok &= require(copied==64, "tap buffer returns requested recent samples");
    ok &= require(std::abs(latest.channel(0)[0]-192.0f)<1.0e-6f && std::abs(latest.channel(0)[63]-255.0f)<1.0e-6f,"tap buffer preserves newest window");
    return ok?0:1;
}
