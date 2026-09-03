#include "guitardsp/graph/RealtimeGraphHost.h"
#include <cmath>
#include <iostream>

using namespace guitardsp::graph;
namespace { bool req(bool c,const char*m){std::cout<<(c?"PASS ":"FAIL ")<<m<<'\n';return c;} }

static GraphDocument gainRig(float gain) {
    GraphDocument d; d.name="gain rig";
    NodeDocument n; n.id=1; n.typeId="utility.gain"; n.displayName="Gain"; n.parameters.push_back({"gain",gain});
    d.nodes.push_back(std::move(n)); return d;
}

int main(){
    bool ok=true; constexpr int block=32;
    NodeRegistry registry=NodeRegistry::createBuiltins();
    RealtimeGraphHost host;
    AudioBuffer in(2,block),out(2,block);in.clear();in.channel(0)[0]=1.0f;in.channel(1)[0]=1.0f;

    host.process(in,out,block);
    ok&=req(std::abs(out.channel(0)[0]-1.0f)<1e-6f,"empty host passes input through");
    ok&=req(host.nodeOutputByTypeId("utility.gain")==nullptr,
            "nodeOutputByTypeId reads nothing before any graph is active");

    auto first=host.prepare(gainRig(0.25f),registry,48000.0,block,2);
    ok&=req(first!=nullptr,"first graph prepares off audio thread");host.submit(std::move(first));
    host.process(in,out,block);
    ok&=req(std::abs(out.channel(0)[0]-0.25f)<1e-6f,"pending graph activates at block boundary");
    ok&=req(host.generation()==1,"graph generation increments after first swap");

    auto second=host.prepare(gainRig(0.75f),registry,48000.0,block,2);
    ok&=req(second!=nullptr,"replacement graph prepares");host.submit(std::move(second));
    host.process(in,out,block);
    ok&=req(std::abs(out.channel(0)[0]-0.75f)<1e-6f,"replacement graph swaps without audio-thread destruction");
    ok&=req(host.generation()==2,"generation increments after replacement");
    ok&=req(host.collectRetired()==1,"control thread reclaims retired graph");

    auto third=host.prepare(gainRig(0.4f),registry,48000.0,block,2);
    auto fourth=host.prepare(gainRig(0.6f),registry,48000.0,block,2);
    host.submit(std::move(third));host.submit(std::move(fourth));
    host.process(in,out,block);
    ok&=req(std::abs(out.channel(0)[0]-0.6f)<1e-6f,"latest pending graph wins before block boundary");
    ok&=req(host.collectRetired()==1,"previous active graph retires after latest-wins swap");

    // Issue #76 change 2: nodeOutputByTypeId() is how a monitor tap resolves
    // a SIGNAL CHAIN stage (typeId) against the currently active graph.
    auto fifth=host.prepare(gainRig(0.5f),registry,48000.0,block,2);
    ok&=req(fifth!=nullptr,"gain-node graph prepares for the typeId tap test");host.submit(std::move(fifth));
    host.process(in,out,block);
    const auto* tapped=host.nodeOutputByTypeId("utility.gain");
    ok&=req(tapped!=nullptr,"nodeOutputByTypeId resolves a typeId present in the active graph");
    ok&=req(tapped!=nullptr && std::abs(tapped->channel(0)[0]-0.5f)<1e-6f,
            "resolved tap buffer carries the node's actual output for the block just processed");
    ok&=req(host.nodeOutputByTypeId("route.merge")==nullptr,
            "nodeOutputByTypeId returns nullptr for a typeId absent from the active graph");
    return ok?0:1;
}
