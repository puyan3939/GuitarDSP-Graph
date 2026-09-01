#pragma once

#include "guitardsp/graph/AudioNode.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace guitardsp::dsp {

class KeyedGateNode final : public guitardsp::graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "KeyedGate"; }
    guitardsp::graph::NodeCategory category() const noexcept override { return guitardsp::graph::NodeCategory::dynamics; }
    int inputPortCount() const noexcept override { return 2; }
    std::size_t parameterCount() const noexcept override { return 6; }
    guitardsp::graph::ParameterDescriptor parameterDescriptor(std::size_t i) const noexcept override {
        using namespace guitardsp::graph;
        static constexpr ParameterDescriptor p[]={
            {"threshold","Threshold",-90.0f,-10.0f,-52.0f,ParameterUnit::decibels,1.0f},
            {"range","Range",-90.0f,0.0f,-72.0f,ParameterUnit::decibels,1.0f},
            {"attack","Attack",0.05f,50.0f,0.8f,ParameterUnit::milliseconds,0.4f},
            {"hold","Hold",0.0f,500.0f,40.0f,ParameterUnit::milliseconds,0.5f},
            {"release","Release",5.0f,2000.0f,145.0f,ParameterUnit::milliseconds,0.5f},
            {"hysteresis","Hysteresis",0.0f,18.0f,5.0f,ParameterUnit::decibels,1.0f}};
        return i<6?p[i]:ParameterDescriptor{};
    }
    float parameterValue(std::size_t i) const noexcept override {
        switch(i){case 0:return thresholdDb_.load();case 1:return rangeDb_.load();case 2:return attackMs_.load();case 3:return holdMs_.load();case 4:return releaseMs_.load();case 5:return hysteresisDb_.load();default:return 0.0f;}
    }
    bool setParameterValue(std::size_t i,float v) noexcept override {
        if(i>=6)return false;
        v=guitardsp::graph::clampParameter(parameterDescriptor(i),v);
        switch(i){case 0:thresholdDb_.store(v);break;case 1:rangeDb_.store(v);break;case 2:attackMs_.store(v);break;case 3:holdMs_.store(v);break;case 4:releaseMs_.store(v);break;case 5:hysteresisDb_.store(v);break;}
        return true;
    }
    void prepare(const guitardsp::graph::PrepareSpec& spec) override {sampleRate_=spec.sampleRate;channels_=std::clamp(spec.channels,1,2);reset();}
    void reset() noexcept override {envelope_.fill(0.0f);gain_.fill(1.0f);holdSamples_.fill(0);open_.fill(true);}
    void process(const guitardsp::graph::AudioBuffer& input,guitardsp::graph::AudioBuffer& output,int n) noexcept override {output.copyFrom(input,n);}
    void processPorts(const guitardsp::graph::ProcessPorts& ports,int n) noexcept override {
        if(ports.inputs.size()<2||ports.outputs.empty())return;
        const auto&audio=*ports.inputs[0];const auto&key=*ports.inputs[1];auto&out=*ports.outputs[0];
        const float threshold=thresholdDb_.load(),range=rangeDb_.load(),attack=attackMs_.load(),hold=holdMs_.load(),release=releaseMs_.load(),hyst=hysteresisDb_.load();
        const float envAttack=coeff(std::max(0.1f,attack*0.6f)),envRelease=coeff(std::max(10.0f,release*0.55f));
        const float gainAttack=coeff(std::max(0.05f,attack)),gainRelease=coeff(std::max(5.0f,release));
        const int chs=std::min({channels_,audio.channels(),key.channels(),out.channels()});
        for(int ch=0;ch<chs;++ch){
            float env=envelope_[ch],g=gain_[ch];int holdCount=holdSamples_[ch];bool isOpen=open_[ch];const float*a=audio.channel(ch);const float*k=key.channel(ch);float*d=out.channel(ch);
            for(int i=0;i<n;++i){
                const float target=std::abs(k[i]);const float ec=target>env?envAttack:envRelease;env=ec*env+(1.0f-ec)*target;const float db=20.0f*std::log10(std::max(env,1.0e-9f));const float openT=threshold+0.5f*hyst,closeT=threshold-0.5f*hyst;
                if(!isOpen&&db>=openT){isOpen=true;holdCount=static_cast<int>(0.001*hold*sampleRate_);}else if(isOpen){if(db>=closeT)holdCount=static_cast<int>(0.001*hold*sampleRate_);else if(holdCount>0)--holdCount;else isOpen=false;}
                const float targetGain=isOpen?1.0f:dbToGain(range);const float gc=targetGain>g?gainAttack:gainRelease;g=gc*g+(1.0f-gc)*targetGain;d[i]=a[i]*g;
            }
            envelope_[ch]=env;gain_[ch]=g;holdSamples_[ch]=holdCount;open_[ch]=isOpen;
        }
    }
private:
    float coeff(float ms)const noexcept{return std::exp(-1.0f/(0.001f*ms*static_cast<float>(sampleRate_)));}
    static float dbToGain(float db)noexcept{return std::pow(10.0f,db/20.0f);}
    double sampleRate_=48000.0;int channels_=2;
    std::atomic<float>thresholdDb_{-52.0f},rangeDb_{-72.0f},attackMs_{0.8f},holdMs_{40.0f},releaseMs_{145.0f},hysteresisDb_{5.0f};
    std::array<float,2>envelope_{},gain_{1.0f,1.0f};std::array<int,2>holdSamples_{};std::array<bool,2>open_{true,true};
};

} // namespace guitardsp::dsp
