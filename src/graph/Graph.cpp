#include "guitardsp/graph/Graph.h"
#include <algorithm>
#include <deque>

namespace guitardsp::graph {

NodeId Graph::addNode(std::unique_ptr<AudioNode> n){if(!n)return 0;const NodeId id=nextId_++;nodes_.emplace(id,std::move(n));executionOrder_.clear();cumulativeLatencies_.clear();maxGraphLatency_=0;return id;}
bool Graph::removeNode(NodeId id){if(nodes_.erase(id)==0)return false;edges_.erase(std::remove_if(edges_.begin(),edges_.end(),[id](const Connection&e){return e.from==id||e.to==id;}),edges_.end());executionOrder_.clear();cumulativeLatencies_.clear();maxGraphLatency_=0;return true;}

bool Graph::connect(NodeId from,int fromPort,NodeId to,int toPort){
    if(from==0||to==0||from==to||!nodes_.contains(from)||!nodes_.contains(to))return false;
    const auto* a=node(from);const auto* b=node(to);
    if(!a||!b||fromPort<0||fromPort>=a->outputPortCount()||toPort<0||toPort>=b->inputPortCount())return false;
    const auto duplicate=std::find_if(edges_.begin(),edges_.end(),[=](const Connection&e){return e.from==from&&e.fromPort==fromPort&&e.to==to&&e.toPort==toPort;});
    if(duplicate!=edges_.end())return false;
    edges_.push_back({from,fromPort,to,toPort});executionOrder_.clear();cumulativeLatencies_.clear();maxGraphLatency_=0;return true;
}

bool Graph::disconnect(NodeId from,int fromPort,NodeId to,int toPort){const auto old=edges_.size();edges_.erase(std::remove_if(edges_.begin(),edges_.end(),[=](const Connection&e){return e.from==from&&e.fromPort==fromPort&&e.to==to&&e.toPort==toPort;}),edges_.end());if(edges_.size()==old)return false;executionOrder_.clear();cumulativeLatencies_.clear();maxGraphLatency_=0;return true;}
void Graph::clear(){nodes_.clear();edges_.clear();executionOrder_.clear();cumulativeLatencies_.clear();maxGraphLatency_=0;nextId_=1;}
AudioNode* Graph::node(NodeId id)noexcept{const auto it=nodes_.find(id);return it==nodes_.end()?nullptr:it->second.get();}
const AudioNode* Graph::node(NodeId id)const noexcept{const auto it=nodes_.find(id);return it==nodes_.end()?nullptr:it->second.get();}

ValidationResult Graph::compile(){
    executionOrder_.clear();cumulativeLatencies_.clear();maxGraphLatency_=0;if(nodes_.empty())return{false,"Graph contains no nodes"};
    std::unordered_map<NodeId,int> indegree;std::unordered_map<NodeId,std::vector<NodeId>> outgoing,incoming;
    for(const auto&[id,p]:nodes_){(void)p;indegree[id]=0;}
    for(const auto&e:edges_){
        const auto* a=node(e.from);const auto* b=node(e.to);
        if(!a||!b)return{false,"Graph contains a connection to a missing node"};
        if(e.fromPort<0||e.fromPort>=a->outputPortCount()||e.toPort<0||e.toPort>=b->inputPortCount())return{false,"Graph contains an invalid port connection"};
        ++indegree[e.to];outgoing[e.from].push_back(e.to);incoming[e.to].push_back(e.from);
    }
    std::vector<NodeId> roots;for(const auto&[id,d]:indegree)if(d==0)roots.push_back(id);std::sort(roots.begin(),roots.end());std::deque<NodeId> ready(roots.begin(),roots.end());
    while(!ready.empty()){const NodeId id=ready.front();ready.pop_front();executionOrder_.push_back(id);auto dst=outgoing[id];std::sort(dst.begin(),dst.end());for(const NodeId d:dst){auto&v=indegree[d];if(--v==0)ready.push_back(d);}}
    if(executionOrder_.size()!=nodes_.size()){executionOrder_.clear();return{false,"Graph contains a cycle"};}
    for(const NodeId id:executionOrder_){int up=0;if(const auto it=incoming.find(id);it!=incoming.end())for(const NodeId parent:it->second)if(const auto l=cumulativeLatencies_.find(parent);l!=cumulativeLatencies_.end())up=std::max(up,l->second);const auto*current=node(id);const int total=up+(current?std::max(0,current->latencySamples()):0);cumulativeLatencies_[id]=total;maxGraphLatency_=std::max(maxGraphLatency_,total);}
    return{true,{}};
}
std::optional<int> Graph::cumulativeLatencySamples(NodeId id)const{const auto it=cumulativeLatencies_.find(id);if(it==cumulativeLatencies_.end())return std::nullopt;return it->second;}

} // namespace guitardsp::graph
