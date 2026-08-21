#pragma once

#include "guitardsp/graph/AudioNode.h"
#include <algorithm>
#include <atomic>
#include <vector>

namespace guitardsp::dsp {

// Allocation-free direct FIR at process time. Intended as the correctness/reference path.
// A partitioned FFT implementation can replace it behind the same node API for long cabinet IRs.
class ConvolutionNode final : public guitardsp::graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "ConvolutionIR"; }
    guitardsp::graph::NodeCategory category() const noexcept override { return guitardsp::graph::NodeCategory::cab; }
    void setImpulseResponse(std::vector<float> ir) { if(ir.empty())ir.push_back(1.0f);if(ir.size()>maxIrLength_)ir.resize(maxIrLength_);ir_=std::move(ir); }
    void setMix(float mix) noexcept {mix_.store(std::clamp(mix,0.0f,1.0f),std::memory_order_relaxed);}
    std::size_t parameterCount()const noexcept override{return 1;}
    guitardsp::graph::ParameterDescriptor parameterDescriptor(std::size_t i)const noexcept override{using namespace guitardsp::graph;return i==0?ParameterDescriptor{"mix","Mix",0.0f,1.0f,1.0f,ParameterUnit::percent,1.0f}:ParameterDescriptor{};}
    float parameterValue(std::size_t i)const noexcept override{return i==0?mix_.load():0.0f;}
    bool setParameterValue(std::size_t i,float v)noexcept override{if(i!=0)return false;setMix(v);return true;}
    void prepare(const guitardsp::graph::PrepareSpec&spec)override{channels_=std::clamp(spec.channels,1,2);if(ir_.empty())ir_={1.0f};history_.assign(static_cast<std::size_t>(channels_),std::vector<float>(ir_.size(),0.0f));write_.assign(static_cast<std::size_t>(channels_),0);}
    void reset()noexcept override{for(auto&h:history_)std::fill(h.begin(),h.end(),0.0f);std::fill(write_.begin(),write_.end(),0);}
    void process(const guitardsp::graph::AudioBuffer&input,guitardsp::graph::AudioBuffer&output,int n)noexcept override{const int chs=std::min({channels_,input.channels(),output.channels()});const float mix=mix_.load(std::memory_order_relaxed);for(int ch=0;ch<chs;++ch){auto&hist=history_[static_cast<std::size_t>(ch)];int&w=write_[static_cast<std::size_t>(ch)];const int size=static_cast<int>(hist.size());const float*src=input.channel(ch);float*dst=output.channel(ch);for(int i=0;i<n;++i){hist[static_cast<std::size_t>(w)]=src[i];float y=0.0f;int r=w;for(std::size_t k=0;k<ir_.size();++k){y+=ir_[k]*hist[static_cast<std::size_t>(r)];if(--r<0)r=size-1;}dst[i]=src[i]+mix*(y-src[i]);if(++w>=size)w=0;}}}
private:
    static constexpr std::size_t maxIrLength_=4096;int channels_=2;std::vector<float>ir_{1.0f};std::vector<std::vector<float>>history_;std::vector<int>write_;std::atomic<float>mix_{1.0f};
};

} // namespace guitardsp::dsp
