#include "guitardsp/graph/CompiledAudioGraph.h"
#include "guitardsp/graph/AdvancedRoutingNodes.h"
#include "guitardsp/graph/UtilityNodes.h"
#include "guitardsp/dsp/KeyedGateNode.h"
#include "guitardsp/dsp/ConvolutionNode.h"
#include <cmath>
#include <iostream>
#include <memory>

using namespace guitardsp::graph;using namespace guitardsp::dsp;
namespace{bool req(bool c,const char*m){std::cout<<(c?"PASS ":"FAIL ")<<m<<'\n';return c;}}
int main(){bool ok=true;constexpr int n=128;
    {Graph g;const auto cross=g.addNode(std::make_unique<CrossoverSplitNode>());const auto merge=g.addNode(std::make_unique<MergeNode>());g.connect(cross,0,merge,0);g.connect(cross,1,merge,0);CompiledAudioGraph rt;ok&=req(rt.build(g,48000,n,2),"multi-output crossover graph builds");AudioBuffer in(2,n),out(2,n);for(int i=0;i<n;++i){in.channel(0)[i]=std::sin(0.17f*i);in.channel(1)[i]=in.channel(0)[i];}rt.process(in,out,n);double err=0;for(int i=0;i<n;++i)err+=std::abs(out.channel(0)[i]-in.channel(0)[i]);ok&=req(err/n<1e-5,"complementary crossover recombines transparently");}
    {KeyedGateNode gate;PrepareSpec spec{48000,n,2,ProcessingQuality::high};gate.prepare(spec);gate.setParameterValue(3,5.0f);gate.setParameterValue(4,20.0f);AudioBuffer audio(2,n),key(2,n),out(2,n);for(int i=0;i<n;++i){audio.channel(0)[i]=audio.channel(1)[i]=1.0f;key.channel(0)[i]=key.channel(1)[i]=0.0f;}const AudioBuffer*ins[]={&audio,&key};AudioBuffer*outs[]={&out};ProcessPorts ports{ins,outs};for(int b=0;b<40;++b)gate.processPorts(ports,n);ok&=req(std::abs(out.channel(0)[n-1])<0.1f,"keyed gate closes on silent detector");for(int i=0;i<n;++i)key.channel(0)[i]=key.channel(1)[i]=1.0f;gate.processPorts(ports,n);ok&=req(out.channel(0)[n-1]>0.5f,"keyed gate opens from independent detector");}
    {ConvolutionNode ir;ir.setImpulseResponse({1.0f,0.5f});PrepareSpec spec{48000,n,2,ProcessingQuality::high};ir.prepare(spec);AudioBuffer in(2,n),out(2,n);in.clear();in.channel(0)[0]=1.0f;in.channel(1)[0]=1.0f;ir.process(in,out,n);ok&=req(std::abs(out.channel(0)[0]-1.0f)<1e-6f&&std::abs(out.channel(0)[1]-0.5f)<1e-6f,"convolution reference path applies IR exactly");}
    return ok?0:1;}
