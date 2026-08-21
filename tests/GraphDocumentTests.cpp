#include "guitardsp/graph/GraphBuilder.h"
#include "guitardsp/graph/NodeRegistry.h"
#include <cmath>
#include <iostream>

using namespace guitardsp::graph;

namespace { bool require(bool c,const char*m){std::cout<<(c?"PASS ":"FAIL ")<<m<<'\n';return c;} }

int main(){
    bool ok=true;
    GraphDocument doc; doc.name="Document build";
    doc.nodes={
        NodeDocument{10,"route.split","Split",0,0,false,false,{}},
        NodeDocument{20,"utility.gain","A",100,-40,false,false,{{"gain",0.5f}}},
        NodeDocument{30,"utility.gain","B",100,40,false,false,{{"gain",0.25f}}},
        NodeDocument{40,"route.merge","Merge",220,0,false,false,{}}
    };
    doc.connections={{10,20},{10,30},{20,40},{30,40}};
    SceneDocument scene;scene.name="Wide";scene.nodes.push_back(SceneNodeState{20,false,false,{{"gain",0.8f}}});doc.scenes.push_back(scene);

    Graph g;auto registry=NodeRegistry::createBuiltins();auto result=buildGraphFromDocument(doc,registry,g);
    ok&=require(result.ok,"graph builds from stable document IDs");
    ok&=require(g.schedule().size()==4,"document graph compiles");
    const NodeId runtimeA=result.documentToRuntimeId.at(20);auto*node=g.node(runtimeA);
    ok&=require(node&&node->parameterIndex("gain")==0,"generic parameter lookup uses stable ID");
    ok&=require(node&&std::abs(node->parameterValue(0)-0.5f)<1e-6f,"document parameter applied");
    ok&=require(applyScene(doc.scenes[0],result.documentToRuntimeId,g),"scene applies without topology rebuild");
    ok&=require(node&&std::abs(node->parameterValue(0)-0.8f)<1e-6f,"scene updates realtime parameter");

    GraphDocument bad;bad.nodes.push_back(NodeDocument{1,"missing.node","Bad"});Graph g2;auto badResult=buildGraphFromDocument(bad,registry,g2);
    ok&=require(!badResult.ok,"unknown node type rejected during document build");
    return ok?0:1;
}
