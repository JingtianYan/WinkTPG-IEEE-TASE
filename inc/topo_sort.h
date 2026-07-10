#pragma once

#include <queue>
#include <set>

#include "define.h"
#include "simulator.h"

#define DEBUG_TOPO 0


struct estimateState {
    int idx=-1;
    int speed_level=-1;
    double min_t=0.0;
    estimateState()=default;
};

struct topoEdge {
    int from_agent;
    int from_idx;
    int out_agent;
    int out_idx;
    bool valid = true;
    bool in_horizon = false;
    topoEdge(const int from_a, const int from_idx, const int out_a, const int out_idx):
    from_agent(from_a), from_idx(from_idx), out_agent(out_a), out_idx(out_idx) {}
};

struct agentOrder {
    int agent_id;
    // index of last element
    int last_idx;
    std::vector<std::shared_ptr<topoEdge>> solved_edges;
    agentOrder(int agent_id, int last_idx): agent_id(agent_id), last_idx(last_idx) {}
};

struct orderCompare {
    bool operator() (const std::pair<int, agentOrder>& lhs, const std::pair<int, agentOrder>& rhs) const {
        if (lhs.first != rhs.first) {
            return lhs.first < rhs.first;
        } else {
            return lhs.second.last_idx < rhs.second.last_idx;
        }
    }
};

class outEdgeCompare
{
public:
    bool operator() (const std::shared_ptr<topoEdge>& E1, const std::shared_ptr<topoEdge>& E2) const {
        if (E1->from_idx > E2->from_idx) {
            return true;
        } else {
            return false;
        }
    }
};


class inEdgeCompare
{
public:
    bool operator() (const std::shared_ptr<topoEdge>& E1, const std::shared_ptr<topoEdge>& E2) const {
        if (E1->out_idx > E2->out_idx) {
            return true;
        } else {
            return false;
        }
    }
};

struct topoEdgeEqual{
    bool operator()(const std::shared_ptr<topoEdge>& lhs, const std::shared_ptr<topoEdge>& rhs) const {
        return lhs->from_agent == rhs->from_agent and lhs->from_idx == rhs->from_idx
               and lhs->out_agent == rhs->out_agent and lhs->out_idx == rhs->out_idx;
    }
};

struct topoNode {
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> prev_out_edges;
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> prev_in_edges;
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> out_edges;
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> in_edges;
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare> all_out_edges;
    std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare> all_in_edges;

    int agent_id = -1;
    topoNode()=default;
};

class topoGraph {
public:
    explicit topoGraph(int num_agent);
    void showTopoGraph() const;
    void findSingleRoundOrder(std::vector<agentOrder>& order);
    void removeInvalidEdges(std::vector<agentOrder>& indexs);
    void removeInvalidEdge(agentOrder& indexs);
    void findCommitCut(std::vector<estimateState>& last_states, std::vector<std::deque<action>>& enque_actions, double planning_horizon);
    void findValidEdges();
    bool findEdge(int agent_id, const std::shared_ptr<topoEdge>& tmp_edge);

public:
    std::vector<topoNode> nodes_;
    std::vector<int> start_vertex;
    std::vector<int> end_vertex;

private:
    int num_agent_;
};