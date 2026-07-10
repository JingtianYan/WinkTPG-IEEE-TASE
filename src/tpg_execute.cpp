#include "tpg_execute.h"

TPGExecutor::TPGExecutor(size_t num_agent, std::shared_ptr<Graph> graph, std::shared_ptr<simEnv> sim_ptr):
num_agent_(num_agent), graph_(graph), sim_ptr_(sim_ptr) {
    last_enque_idx.resize(num_agent_, 0);
    last_finish_idx.resize(num_agent_, -1);
    topo_ptr = graph_->getTopoGraph();
}

void TPGExecutor::updateTPG() {
    std::vector<simState> last_locations;
    std::vector<double> last_acc_delay;
    sim_ptr_->getPrevLocations(last_locations, last_acc_delay);
    for (int i = 0; i < num_agent_; i++) {
        last_finish_idx[i] = last_locations[i].idx;
    }
}

bool TPGExecutor::getNextActions(std::vector<std::vector<action> > &new_actions) {
    bool succeed_flag = true;
    auto plan_start = std::chrono::high_resolution_clock::now();
    new_actions.resize(num_agent_);
    updateTPG();
    double curr_sim_time = sim_ptr_->getSimTime();
#ifdef DEBUG_TPG_EXECUTE
    for (int i = 0; i < last_enque_idx.size(); i++) {
        printf("BEGIN::Agent %d, with finish idx: %d, last enque idx: %d, full path length: %lu\n",
            i, last_finish_idx[i], last_enque_idx[i], (*graph_->paths)[i].size());
    }
#endif

    for (int i = 0; i < num_agent_; i++) {
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, outEdgeCompare>
            tmp_out_edges = topo_ptr->nodes_[i].all_out_edges;
        while (not tmp_out_edges.empty()) {
            if (tmp_out_edges.top()->from_idx <= last_finish_idx[i]) {
                tmp_out_edges.top()->valid = false;
            } else {
                tmp_out_edges.top()->valid = true;
            }
            tmp_out_edges.pop();
        }
    }

    for (int i = 0; i < num_agent_; i++) {
        std::priority_queue<std::shared_ptr<topoEdge>, std::vector<std::shared_ptr<topoEdge>>, inEdgeCompare>
            tmp_in_edges = topo_ptr->nodes_[i].all_in_edges;
        int tmp_prev_enque_idx = last_enque_idx[i];
        Path full_path = (*graph_->paths)[i];
        last_enque_idx[i] = static_cast<int> (full_path.size())-1;
        while (not tmp_in_edges.empty()) {
            if (tmp_in_edges.top()->valid) {
                last_enque_idx[i] = std::min(tmp_in_edges.top()->out_idx-1, static_cast<int> (full_path.size())-1);
                break;
            }
            tmp_in_edges.pop();
        }
        std::vector<temporalConstraint> tmp_edge_constraints;
        std::vector<action> tmp_exec_plan;
        double min_t = curr_sim_time;
        int tmp_enque_idx = last_enque_idx[i];
        assert(tmp_prev_enque_idx <= tmp_enque_idx);
#ifdef DEBUG_TPG_EXECUTE
        printf("Finding path for agent %d\n", i);
#endif

        if (tmp_enque_idx != tmp_prev_enque_idx) {
            Path tmp_path(full_path.begin()+tmp_prev_enque_idx, full_path.begin() + tmp_enque_idx + 1);
            auto ll_start = std::chrono::high_resolution_clock::now();
            auto tmp_solution = singleAgentSolver.run(tmp_path, tmp_edge_constraints,
                tmp_exec_plan, 0.0, min_t);
            auto ll_stop = std::chrono::high_resolution_clock::now();
            // Calculate the duration
            auto ll_duration = std::chrono::duration_cast<std::chrono::microseconds>(ll_stop - ll_start);
            auto ll_duration_seconds = std::chrono::duration<double>(ll_duration).count();
            ll_runtime += ll_duration_seconds;
        }
        auto curr_time = std::chrono::high_resolution_clock::now();
        auto curr_duration = std::chrono::duration_cast<std::chrono::microseconds>(curr_time - plan_start);
        auto curr_duration_seconds = std::chrono::duration<double>(curr_duration).count();
        if (curr_duration_seconds > sim_ptr_->plan_time_limit) {
            succeed_flag = false;
            break;
        }
        new_actions[i] = tmp_exec_plan;
    }
#ifdef DEBUG_TPG_EXECUTE
    for (int i = 0; i < last_enque_idx.size(); i++) {
        printf("END::Agent %d, with finish idx: %d, last enque idx: %d, full path length: %lu\n",
            i, last_finish_idx[i], last_enque_idx[i], (*graph_->paths)[i].size());
    }
#endif
    return succeed_flag;
}