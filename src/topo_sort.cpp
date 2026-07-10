#include "topo_sort.h"

#include <iostream>

topoGraph::topoGraph(int const num_agent): num_agent_(num_agent)
{
    nodes_.resize(num_agent);
    int i = 0;
    for (auto& node: nodes_) {
        node.agent_id = i;
        i++;
    }
}

void topoGraph::showTopoGraph() const {
    int i = 0;
    for (const auto& node: nodes_) {
#if DEBUG_TOPO
        printf("For agent %d\n", i);
        printf("Listing in edges:\n");
#endif
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> tmp_edge_in = node.in_edges;
        while(not tmp_edge_in.empty()) {
#if DEBUG_TOPO
            printf("Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
            tmp_edge_in.top()->valid,
                    tmp_edge_in.top()->from_agent, tmp_edge_in.top()->from_idx,
                     tmp_edge_in.top()->out_agent,  tmp_edge_in.top()->out_idx);
#endif
            // tmp_edge_in.top()->valid=false;
            tmp_edge_in.pop();
        }
#if DEBUG_TOPO
        printf("Listing out edges:\n");
#endif
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> tmp_edge_out = node.out_edges;
        while(not tmp_edge_out.empty()) {
#if DEBUG_TOPO
            printf("Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
            tmp_edge_out.top()->valid,
                    tmp_edge_out.top()->from_agent, tmp_edge_out.top()->from_idx,
                     tmp_edge_out.top()->out_agent,  tmp_edge_out.top()->out_idx);
#endif
            // tmp_edge_out.top()->valid=false;
            tmp_edge_out.pop();
        }
        i++;
    }
}

void topoGraph::findSingleRoundOrder(std::vector<agentOrder>& order) {
    int i = 0;
    std::priority_queue<std::pair<int, agentOrder>, std::vector<std::pair<int, agentOrder>>, orderCompare> sort_order;
    // After the agent with the most edge can be solved is found, we need to continue search
    // (since some edges are already invalid)
    for (auto& node:nodes_) {
#if DEBUG_TOPO
        printf("topoSort::iterate agent: %d\n", i);
#endif
        int in_time_limit = INF;
        int resol_edge_count = 0;
        std::vector<std::shared_ptr<topoEdge>> resol_edges;
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> tmp_edge_in = node.in_edges;
        while(not tmp_edge_in.empty()) {
            if (tmp_edge_in.top()->valid) {
#if DEBUG_TOPO
                 printf("Bottle nect edge: Is Valid: %d, From (Agent: %d, time: %d) -> (Agent: %d, time: %d)\n",
                     tmp_edge_in.top()->valid, tmp_edge_in.top()->from_agent, tmp_edge_in.top()->from_idx,
                     tmp_edge_in.top()->out_agent,  tmp_edge_in.top()->out_idx);
#endif
                if (node.agent_id == DEBUG_AGENT1 or node.agent_id == DEBUG_AGENT2) {
                    printf("Bottleneck edge: Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
                           tmp_edge_in.top()->valid, tmp_edge_in.top()->from_agent, tmp_edge_in.top()->from_idx,
                           tmp_edge_in.top()->out_agent,  tmp_edge_in.top()->out_idx);
                }
                in_time_limit = tmp_edge_in.top()->out_idx;
                break;
            }
            tmp_edge_in.pop();
        }
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> tmp_edge_out = node.out_edges;
        while(not tmp_edge_out.empty()) {

            if (tmp_edge_out.top()->from_idx >= in_time_limit) {
                break;
            }
            if (tmp_edge_out.top()->valid) {
                resol_edge_count++;

#if DEBUG_TOPO
                printf("ResolEdge::Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
                    tmp_edge_out.top()->valid, tmp_edge_out.top()->from_agent, tmp_edge_out.top()->from_idx,
                    tmp_edge_out.top()->out_agent,  tmp_edge_out.top()->out_idx);
#endif
            }

            resol_edges.push_back(tmp_edge_out.top());
            tmp_edge_out.pop();
        }
        agentOrder tmp_agent_order(i, in_time_limit);
        tmp_agent_order.solved_edges = resol_edges;
        sort_order.emplace(resol_edge_count, tmp_agent_order);
        // printf("Agent: %d, Resolved edge total count: %d, last index: %d\n", i, resol_edge_count, in_time_limit);
        i++;
    }
    order.clear();
    int debug_sol_cnt = 0;
    while (not sort_order.empty()) {
        auto tmp_order = sort_order.top();
        debug_sol_cnt += tmp_order.first;
        sort_order.pop();
        order.push_back(tmp_order.second);
#if DEBUG_TOPO
        printf("topoSort::Agent: %d, Resolved edge total count: %d, last index: %d\n", tmp_order.second.agent_id,
            tmp_order.first, tmp_order.second.last_idx);
#endif
    }

}


void topoGraph::removeInvalidEdge(agentOrder& index) {
    int cnt = 0;
    for (auto tmp_edge: index.solved_edges) {
        if (tmp_edge->valid) {
            tmp_edge->valid = false;
            cnt++;
        }
    }

}

// No need to loop from the begining
void topoGraph::removeInvalidEdges(std::vector<agentOrder>& indexs) {
    int i = 0;
    std::vector<std::shared_ptr<topoEdge>> all_solved_edge;
    all_solved_edge.clear();
    for (auto& node:nodes_) {
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> tmp_edge_out = node.out_edges;
        while(not tmp_edge_out.empty()) {

            if (tmp_edge_out.top()->from_idx >= indexs[i].last_idx) {
                break;
            }
            if (tmp_edge_out.top()->valid) {
                all_solved_edge.push_back(tmp_edge_out.top());
                tmp_edge_out.top()->valid = false;
            }
            tmp_edge_out.pop();
        }
        i++;
    }
}

bool topoGraph::findEdge(int agent_id, const std::shared_ptr<topoEdge>& tmp_edge) {
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> prev_in_edges = nodes_[agent_id].prev_in_edges;
    while (not prev_in_edges.empty()) {
        if (tmp_edge == prev_in_edges.top()) {
            return false;
        }
        prev_in_edges.pop();
    }
    return true;
}

void topoGraph::findCommitCut(std::vector<estimateState>& last_states, std::vector<std::deque<action>>& enque_actions, double planning_horizon)
{

    start_vertex.clear();
    start_vertex.reserve(last_states.size());
    for (auto& last_state: last_states) {
        start_vertex.push_back(last_state.idx);
    }

    std::vector<int> prev_end_vertex;
    if (end_vertex.empty()) {
        prev_end_vertex.resize(last_states.size(), 0);
    } else {
        prev_end_vertex = end_vertex;
    }
    end_vertex.clear();
    end_vertex.reserve(start_vertex.size());
    for (int i = 0; i < start_vertex.size(); i++) {
        end_vertex.push_back(std::max(start_vertex[i]+static_cast<int>(planning_horizon), prev_end_vertex[i]));
    }

    // Find all type-2 edges that need to be figured
    bool reverse_edge = true;
    while (reverse_edge) {
        reverse_edge = false;
        for (int agent_id = 0; agent_id < start_vertex.size(); agent_id++) {
            int tmp_start_vertex = start_vertex[agent_id];
            int tmp_end_vertex = end_vertex[agent_id];
            std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> tmp_edge_in = nodes_[agent_id].all_in_edges;
            while(not tmp_edge_in.empty()) {
                // include start and end points
                tmp_edge_in.top()->valid=false;
                if (tmp_edge_in.top()->out_idx >= tmp_start_vertex and tmp_edge_in.top()->out_idx <= tmp_end_vertex) {
                    if (tmp_edge_in.top()->from_idx > end_vertex[tmp_edge_in.top()->from_agent]) {
                        // If the type-2 edge beyond the current horizon, need to include it
                        end_vertex[tmp_edge_in.top()->from_agent] = tmp_edge_in.top()->from_idx;
                        reverse_edge = true;
                    }
                }
                tmp_edge_in.pop();
            }
        }
    }
    for (int i = 0; i < num_agent_; i++) {
#if DEBUG_TOPO
        printf("Agent %d from %d to %d\n", i, start_vertex[i], end_vertex[i]);
#endif
        if (i == DEBUG_AGENT1 or i == DEBUG_AGENT2) {
            printf("Agent %d from %d to %d\n", i, start_vertex[i], end_vertex[i]);
        }
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> prev_in_edges;
        std::swap(nodes_[i].in_edges, prev_in_edges);
        std::swap(nodes_[i].prev_in_edges, prev_in_edges);
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> prev_out_edges;
        std::swap(nodes_[i].out_edges, prev_out_edges);
        std::swap(nodes_[i].prev_out_edges, prev_out_edges);
    }

    for (int agent_id = 0; agent_id < start_vertex.size(); agent_id++) {
        int tmp_start_vertex = start_vertex[agent_id];
        int tmp_end_vertex = end_vertex[agent_id];
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> tmp_edge_in = nodes_[agent_id].all_in_edges;
        while(not tmp_edge_in.empty()) {
 #if DEBUG_TOPO
//             printf("Loop::Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
//             tmp_edge_in.top()->valid,
//                     tmp_edge_in.top()->from_agent, tmp_edge_in.top()->from_idx,
//                      tmp_edge_in.top()->out_agent,  tmp_edge_in.top()->out_idx);
 #endif
            tmp_edge_in.top()->valid=false;
            if (tmp_edge_in.top()->out_idx >= tmp_start_vertex and tmp_edge_in.top()->out_idx <= tmp_end_vertex) {
                if (tmp_edge_in.top()->from_idx <= start_vertex[tmp_edge_in.top()->from_agent]) {
                    // If the type-2 edge from the previous horizon, ignore it (already resolved at previous horizon)
                    ;
                } else if (tmp_edge_in.top()->from_idx <= end_vertex[tmp_edge_in.top()->from_agent]) {
                    // If the type-2 edge from the current horizon, include it
                    tmp_edge_in.top()->valid=findEdge(agent_id, tmp_edge_in.top());

#if DEBUG_TOPO
                    printf("Insert::Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
                    tmp_edge_in.top()->valid,
                            tmp_edge_in.top()->from_agent, tmp_edge_in.top()->from_idx,
                             tmp_edge_in.top()->out_agent,  tmp_edge_in.top()->out_idx);
#endif
                    if (tmp_edge_in.top()->out_agent == DEBUG_AGENT1) {
                        printf("Insert::Is Valid: %d, From (Agent: %d, idx: %d) -> (Agent: %d, idx: %d)\n",
                               tmp_edge_in.top()->valid,
                               tmp_edge_in.top()->from_agent, tmp_edge_in.top()->from_idx,
                               tmp_edge_in.top()->out_agent,  tmp_edge_in.top()->out_idx);
                    }
                    nodes_[agent_id].in_edges.push(tmp_edge_in.top());
                    nodes_[tmp_edge_in.top()->from_agent].out_edges.push(tmp_edge_in.top());
                } else {
                    // If the type-2 edge beyond the current horizon, need to include it
                    std::cerr << "Incorrect type-2 edge" << std::endl;
                    exit(-1);
                }
            }
            tmp_edge_in.pop();
        }
    }
}

void topoGraph::findValidEdges() {
    for (size_t i = 0; i < num_agent_; i++) {
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> tmp_out_edges = nodes_[i].out_edges;

    }
}