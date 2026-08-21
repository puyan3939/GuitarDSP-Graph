#include "guitardsp/graph/CompiledAudioGraph.h"
#include <algorithm>
#include <set>

namespace guitardsp::graph {

CompiledAudioGraph::NodeRuntime* CompiledAudioGraph::runtime(NodeId id)noexcept{const auto it=runtimeIndex_.find(id);return it==runtimeIndex_.end()?nullptr:&nodes_[it->second];}
const CompiledAudioGraph::NodeRuntime* CompiledAudioGraph::runtime(NodeId id)const noexcept{const auto it=runtimeIndex_.find(id);return it==runtimeIndex_.end()?nullptr:&nodes_[it->second];}

bool CompiledAudioGraph::build(Graph& graph,double sampleRate,int maxBlockSize,int channels,ProcessingQuality quality){
    if(maxBlockSize<=0||channels<=0)return false;
    const auto validation=graph.compile();if(!validation.ok)return false;
    maxBlockSize_=maxBlockSize;channels_=channels;totalLatencySamples_=graph.maximumGraphLatencySamples();order_=graph.schedule();nodes_.clear();runtimeIndex_.clear();sinks_.clear();nodes_.reserve(order_.size());
    const PrepareSpec spec{sampleRate,maxBlockSize,channels,quality};

    for(const NodeId id:order_){
        auto* node=graph.node(id);if(!node)return false;NodeRuntime r;r.id=id;r.node=node;
        r.inputs.resize(static_cast<std::size_t>(std::max(0,node->inputPortCount())));
        r.outputs.resize(static_cast<std::size_t>(std::max(0,node->outputPortCount())));
        for(auto& in:r.inputs)in.mix.resize(channels,maxBlockSize);
        for(auto& out:r.outputs)out.resize(channels,maxBlockSize);
        node->prepare(spec);runtimeIndex_[id]=nodes_.size();nodes_.push_back(std::move(r));
    }

    for(auto& r:nodes_){
        for(int port=0;port<static_cast<int>(r.inputs.size());++port){
            int maxParentLatency=0;
            for(const auto&e:graph.connections())if(e.to==r.id&&e.toPort==port)maxParentLatency=std::max(maxParentLatency,graph.cumulativeLatencySamples(e.from).value_or(0));
            for(const auto&e:graph.connections())if(e.to==r.id&&e.toPort==port){
                InputEdgeRuntime edge;edge.source=e.from;edge.sourcePort=e.fromPort;edge.compensation.prepare(channels,totalLatencySamples_,maxBlockSize);edge.compensation.setDelaySamples(maxParentLatency-graph.cumulativeLatencySamples(e.from).value_or(0));r.inputs[static_cast<std::size_t>(port)].upstream.push_back(std::move(edge));
            }
        }
    }

    std::set<std::pair<NodeId,int>> connectedOutputs;
    for(const auto&e:graph.connections())connectedOutputs.emplace(e.from,e.fromPort);
    for(const NodeId id:order_){
        const auto* node=graph.node(id);if(!node)continue;
        for(int port=0;port<node->outputPortCount();++port){
            if(connectedOutputs.contains({id,port}))continue;
            SinkRuntime sink;sink.source=id;sink.sourcePort=port;sink.busIndex=std::max(0,node->physicalOutputBusIndex());sink.compensation.prepare(channels,totalLatencySamples_,maxBlockSize);sink.compensation.setDelaySamples(totalLatencySamples_-graph.cumulativeLatencySamples(id).value_or(0));sinks_.push_back(std::move(sink));
        }
    }
    reset();return !nodes_.empty()&&!sinks_.empty();
}

void CompiledAudioGraph::reset()noexcept{
    for(auto&r:nodes_){for(auto&in:r.inputs){in.mix.clear();for(auto&e:in.upstream)e.compensation.reset();}for(auto&out:r.outputs)out.clear();if(r.node)r.node->reset();}
    for(auto&s:sinks_)s.compensation.reset();
}

void CompiledAudioGraph::process(const AudioBuffer& externalInput,AudioBuffer& externalOutput,int numSamples)noexcept{
    AudioBuffer* buses[]={&externalOutput};
    processMultiOutput(externalInput,std::span<AudioBuffer* const>(buses,1),numSamples);
}

void CompiledAudioGraph::processMultiOutput(const AudioBuffer& externalInput,std::span<AudioBuffer* const> outputBuses,int numSamples)noexcept{
    if(numSamples<=0||numSamples>maxBlockSize_)return;
    for(auto* bus:outputBuses)if(bus)bus->clear(numSamples);
    for(auto&r:nodes_){
        r.inputPointers.clear();r.outputPointers.clear();
        for(std::size_t port=0;port<r.inputs.size();++port){
            auto&in=r.inputs[port];in.mix.clear(numSamples);
            if(in.upstream.empty()){
                if(port==0)in.mix.copyFrom(externalInput,numSamples);
            }else{
                for(auto&e:in.upstream){const auto*u=runtime(e.source);if(!u||e.sourcePort<0||e.sourcePort>=static_cast<int>(u->outputs.size()))continue;e.compensation.processAdd(u->outputs[static_cast<std::size_t>(e.sourcePort)],in.mix,numSamples);}
            }
            r.inputPointers.push_back(&in.mix);
        }
        for(auto&out:r.outputs){out.clear(numSamples);r.outputPointers.push_back(&out);}
        if(r.node->isMuted())continue;
        if(r.node->isBypassed()){
            if(!r.inputs.empty())for(auto&out:r.outputs)out.copyFrom(r.inputs[0].mix,numSamples);
            continue;
        }
        const ProcessPorts ports{std::span<const AudioBuffer* const>(r.inputPointers.data(),r.inputPointers.size()),std::span<AudioBuffer* const>(r.outputPointers.data(),r.outputPointers.size())};
        r.node->processPorts(ports,numSamples);
    }
    for(auto&s:sinks_){
        if(s.busIndex<0||s.busIndex>=static_cast<int>(outputBuses.size())||outputBuses[static_cast<std::size_t>(s.busIndex)]==nullptr)continue;
        const auto*r=runtime(s.source);if(!r||s.sourcePort<0||s.sourcePort>=static_cast<int>(r->outputs.size()))continue;
        s.compensation.processAdd(r->outputs[static_cast<std::size_t>(s.sourcePort)],*outputBuses[static_cast<std::size_t>(s.busIndex)],numSamples);
    }
}

} // namespace guitardsp::graph
