#pragma once

#include "AudioNode.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace guitardsp::graph {

// One input, two coherent outputs. Low + High reconstructs the original sample exactly.
// This is intentionally a simple complementary split; higher-order crossover nodes can be added later.
class CrossoverSplitNode final : public AudioNode {
public:
    std::string_view typeName() const noexcept override { return "CrossoverSplit"; }
    int outputPortCount() const noexcept override { return 2; }
    std::size_t parameterCount() const noexcept override { return 1; }
    ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        return i==0 ? ParameterDescriptor{"frequency","Frequency",40.0f,8000.0f,250.0f,ParameterUnit::hertz,0.35f} : ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override { return i==0 ? frequency_.load(std::memory_order_relaxed) : 0.0f; }
    bool setParameterValue(std::size_t i,float v) noexcept override { if(i!=0)return false;frequency_.store(clampParameter(parameterDescriptor(0),v),std::memory_order_relaxed);return true; }
    void prepare(const PrepareSpec& spec) override { sampleRate_=spec.sampleRate;channels_=std::clamp(spec.channels,1,2);reset(); }
    void reset() noexcept override { state_.fill(0.0f); }
    void process(const AudioBuffer& input,AudioBuffer& output,int n) noexcept override { output.copyFrom(input,n); }
    void processPorts(const ProcessPorts& ports,int n) noexcept override {
        if(ports.inputs.empty()||ports.outputs.size()<2)return;
        const auto& in=*ports.inputs[0];auto& low=*ports.outputs[0];auto& high=*ports.outputs[1];
        const float hz=std::clamp(frequency_.load(std::memory_order_relaxed),20.0f,static_cast<float>(0.45*sampleRate_));
        const float a=std::exp(-2.0f*3.14159265358979323846f*hz/static_cast<float>(sampleRate_));
        const int chs=std::min({channels_,in.channels(),low.channels(),high.channels()});
        for(int ch=0;ch<chs;++ch){float z=state_[static_cast<std::size_t>(ch)];const float*src=in.channel(ch);float*l=low.channel(ch);float*h=high.channel(ch);for(int i=0;i<n;++i){z=(1.0f-a)*src[i]+a*z;l[i]=z;h[i]=src[i]-z;}state_[static_cast<std::size_t>(ch)]=z;}
    }
private:
    std::atomic<float> frequency_{250.0f};double sampleRate_=48000.0;int channels_=2;std::array<float,2>state_{};
};

} // namespace guitardsp::graph
