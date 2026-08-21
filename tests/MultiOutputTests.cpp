#include "guitardsp/graph/CompiledAudioGraph.h"
#include "guitardsp/graph/IONodes.h"
#include "guitardsp/graph/UtilityNodes.h"
#include <cmath>
#include <iostream>
#include <memory>

using namespace guitardsp::graph;
namespace { bool req(bool c,const char*m){std::cout<<(c?"PASS ":"FAIL ")<<m<<'\n';return c;} }

int main(){
    bool ok=true;constexpr int n=32;
    Graph g;
    const auto split=g.addNode(std::make_unique<SplitNode>());
    const auto mainGain=g.addNode(std::make_unique<GainNode>(0.75f));
    const auto subGain=g.addNode(std::make_unique<GainNode>(0.25f));
    const auto out0=g.addNode(std::make_unique<OutputBusNode>(0));
    const auto out1=g.addNode(std::make_unique<OutputBusNode>(1));
    g.connect(split,mainGain);g.connect(split,subGain);g.connect(mainGain,out0);g.connect(subGain,out1);
    CompiledAudioGraph rt;ok&=req(rt.build(g,48000.0,n,2),"multi-output graph builds");
    AudioBuffer in(2,n),a(2,n),b(2,n);for(int i=0;i<n;++i){in.channel(0)[i]=1.0f;in.channel(1)[i]=1.0f;}
    AudioBuffer* buses[]={&a,&b};rt.processMultiOutput(in,buses,n);
    ok&=req(std::abs(a.channel(0)[4]-0.75f)<1e-6f,"bus 0 receives main branch only");
    ok&=req(std::abs(b.channel(0)[4]-0.25f)<1e-6f,"bus 1 receives sub branch only");
    OutputBusNode output;ok&=req(output.parameterIndex("bus")==0,"output bus is preset-addressable");output.setParameterValue(0,3.0f);ok&=req(output.physicalOutputBusIndex()==3,"output bus index parameter applies");
    return ok?0:1;
}
