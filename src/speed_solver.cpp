#include "speed_solver.h"

#include <utility>

speedSolver::speedSolver(int num_agent, std::shared_ptr<Graph> graph, std::shared_ptr<simEnv> sim) {
    TCT_ptr = std::make_shared<constraintTable>(num_agent);
    topo_graph = graph->getTopoGraph();
    graph_ = graph;
    sim_ptr = std::move(sim);
    num_agent_ = num_agent;
}



/**
 *
 * @param last_loc
 * @param enque_actions
 * @param curr_locs return (current location idx, init start time)
 */
void speedSolver::noiseEstimation(std::vector<std::deque<action>>& enque_actions) {
    curr_loc.clear();
    assert(last_seen_locations.size() == enque_actions.size());
    std::vector<std::vector<double>> estimated_leave_time;
    estimated_leave_time.resize(num_agent_);
    std::vector<bool> non_station_agents;
    for (int i = 0; i < last_seen_locations.size(); i++) {
        estimateState tmp_state;
        tmp_state.idx = last_seen_locations[i].idx + static_cast<int>(enque_actions[i].size());
#if DEBUG_SPEED_SOLVER
        printf("For agent %d, Last loc: %d, enque actions size: %lu\n", i,
               last_seen_locations[i].idx, enque_actions[i].size());
#endif
        if (enque_actions[i].empty()) {
            tmp_state.speed_level = last_seen_locations[i].v;
            tmp_state.min_t = last_seen_locations[i].time;
        } else {
            tmp_state.speed_level = enque_actions[i].back().level_g;
            double tmp_t = last_seen_locations[i].time;
            for (auto& tmp_act: enque_actions[i]) {
                tmp_t = std::max(tmp_act.start_t, tmp_t)  + tmp_act.cost;
                estimated_leave_time[i].push_back(tmp_t);
            }
            tmp_state.min_t = tmp_t;
#if DEBUG_SPEED_SOLVER
            for (int k = 0; k < estimated_leave_time[i].size(); k++) {
                printf("For agent %d, leave time at idx %d is %f\n", i, last_seen_locations[i].idx+k, estimated_leave_time[i][k]);
            }
#endif
        }
        if (tmp_state.speed_level != 0) {
            non_station_agents.push_back(i);
        }
        curr_loc.push_back(tmp_state);
    }
#if DEBUG_SPEED_SOLVER
    for (size_t i = 0; i < curr_loc.size(); i++) {
        printf("Agent %lu start from idx:%d, with velocity: %d; At time: %f\n", i, curr_loc[i].idx,
        curr_loc[i].speed_level, curr_loc[i].min_t);
    }
#endif
    for (size_t i = 0; i < curr_loc.size(); i++) {
        if (i == DEBUG_AGENT1 or i == DEBUG_AGENT2) {
            printf("Agent %lu start from idx:%d, with velocity: %d; At time: %f, expected time: %f, with total delay: %f\n", i, curr_loc[i].idx,
                   curr_loc[i].speed_level, curr_loc[i].min_t, acc_last_seen_locations_time[i], curr_loc[i].min_t - acc_last_seen_locations_time[i]);
        }
    }

    for (int i = 0; i < last_seen_locations.size(); i++) {
        auto& entry = TCT_ptr->table[i];
//        double tmp_delay = std::max(last_acc_delays[i], 0.0);
        double  tmp_delay = curr_loc[i].min_t - acc_last_seen_locations_time[i];
        for (auto& tmp_entry_at_idx: entry) {
            for (auto& tmp_entry_at_agent: tmp_entry_at_idx.second) {
//                result_constraints.push_back(elem.second);
                for (auto& elem: tmp_entry_at_agent.second) {
                    if (i == DEBUG_AGENT1) {
                        printf("PREV::Free interval constraint for Agent %d, at idx %d (%d), from constraining agent %d at idx %d; From %f -> %f\n",
                               i, elem.second.goal_path_idx-curr_loc[i].idx, tmp_entry_at_idx.first,
                               tmp_entry_at_agent.first, elem.first,
                               elem.second.interval.first, elem.second.interval.second);
                    }
                    if (elem.second.interval.first == 0) {
                        elem.second.interval.second = elem.second.interval.second + tmp_delay;
                    }
                    if (elem.second.interval.second == INF) {
                        elem.second.interval.first = elem.second.interval.first + tmp_delay;
                    }
                    if (i == DEBUG_AGENT1) {
                        printf("AFTER::Free interval constraint for Agent %d, at idx %d (%d), from constraining agent %d at idx %d; From %f -> %f\n",
                               i, elem.second.goal_path_idx-curr_loc[i].idx, tmp_entry_at_idx.first,
                               tmp_entry_at_agent.first, elem.first,
                               elem.second.interval.first, elem.second.interval.second);
                    }
                }
            }
        }
    }

    for (int i = 0; i < last_seen_locations.size(); i++) {
        auto all_out_edge = topo_graph->nodes_[i].all_out_edges;
        while (not all_out_edge.empty()) {
            auto topo_edge = all_out_edge.top();
//            assert(topo_edge->out_idx != curr_loc[topo_edge->out_agent].idx);
            if (topo_edge->from_idx > last_seen_locations[i].idx and
                    topo_edge->from_idx <= curr_loc[i].idx and
                    topo_edge->out_idx >= curr_loc[topo_edge->out_agent].idx) {
                double tmp_leave_t = estimated_leave_time[i][topo_edge->from_idx - last_seen_locations[i].idx - 1];
#if DEBUG_SPEED_SOLVER
                printf("Agent %d, leave time at %d (%d) is: %f\n", i, topo_edge->from_idx - 1, topo_edge->from_idx - last_seen_locations[i].idx - 1,
                       tmp_leave_t);
#endif
                double tmp_delay = 0.0;
                // Only Gaussian delay model supported: delay_type_ == 1 enables gaussian delay
                if (sim_ptr->delay_type_ == 1) {
                    tmp_delay = computeGaussianDelay(topo_edge->from_idx-1- last_seen_locations[topo_edge->from_agent].idx,
                                                     topo_edge->out_idx - last_seen_locations[topo_edge->out_agent].idx);
                }
                int tmp_out_idx = topo_edge->out_idx;
                temporalConstraint new_constraint(tmp_out_idx, tmp_leave_t+CONS_SLACK+tmp_delay, INF);
#if DEBUG_SPEED_SOLVER
                printf("Insert constraint for agent %d at idx: %d (%d): from %f to %f\n", topo_edge->out_agent,
                       topo_edge->out_idx, tmp_out_idx, new_constraint.interval.first, new_constraint.interval.second);
#endif
                int tmp_from_idx = topo_edge->from_idx-1;
                if (topo_edge->from_agent == DEBUG_AGENT1 or topo_edge->out_agent == DEBUG_AGENT1) {
//                    printf("Insert constraint for agent %d at idx: %d (%d): from %f to %f\n", topo_edge->out_agent,
//                           topo_edge->out_idx, tmp_out_idx, new_constraint.interval.first, new_constraint.interval.second);
                    printf("PrevRound::Insert constraint from agent %d at idx %d to agent %d at idx %d: from %f to %f\n",
                           topo_edge->from_agent, tmp_from_idx,
                           topo_edge->out_agent, tmp_out_idx,
                           new_constraint.interval.first, new_constraint.interval.second);
                }
                bool debug = true;
                auto constraints_at_idx = TCT_ptr->table[topo_edge->out_agent].find(tmp_out_idx);
                if (constraints_at_idx != TCT_ptr->table[topo_edge->out_agent].end()) {
                    auto constraint_agent = constraints_at_idx->second.find(topo_edge->from_agent);
                    if (constraint_agent != constraints_at_idx->second.end()) {
                        auto constraint_at_agent_at_idx = constraint_agent->second.find(tmp_from_idx);
                        if (constraint_at_agent_at_idx != constraint_agent->second.end()) {
                            constraint_at_agent_at_idx->second = new_constraint;
                            debug = false;
                        } else {
                            constraint_agent->second.insert(std::make_pair(tmp_from_idx, new_constraint));
                        }
                    } else {
                        std::map<int, temporalConstraint> tmp_constraint_map;
                        tmp_constraint_map.insert(std::make_pair(tmp_from_idx, new_constraint));
                        constraints_at_idx->second.insert(std::make_pair(topo_edge->from_agent, tmp_constraint_map));
                    }
                } else {
                    std::map<int, temporalConstraint> tmp_constraint_map;
                    tmp_constraint_map.insert(std::make_pair(tmp_from_idx, new_constraint));
                    std::map<int, std::map<int, temporalConstraint>> tmp_constraints;
                    tmp_constraints.insert(std::make_pair(topo_edge->from_agent, tmp_constraint_map));
                    TCT_ptr->table[topo_edge->out_agent].insert(std::make_pair(tmp_out_idx, tmp_constraints));
                }

            }
            all_out_edge.pop();
        }
    }


}

bool speedSolver::solve(std::vector<std::vector<action>>& exec_actions) {
    exec_actions.clear();
    exec_actions.resize(num_agent_);
    std::vector<std::deque<std::pair<double, double>>> all_path_entry;
    all_path_entry.resize(num_agent_);
    std::vector<int> order;
    bool plan_succedd_flag = true;
    bool finished = false;
    double sum_sol = 0.0;
//    TCT_ptr->clear();
#if DEBUG_SPEED_SOLVER
    printf("Start speed solver::solver started!\n");
#endif
    // printf("Start speed solver::solver started!\n");

    int round_count = 0;
    sim_ptr->getPrevLocations(last_seen_locations, acc_last_seen_locations_time);
    std::vector<std::deque<action>> enque_actions;
    sim_ptr->getEnqueActions(enque_actions);

    noiseEstimation(enque_actions);
//    printf("Finding commit cuts!\n");
    topo_graph->findCommitCut(curr_loc, enque_actions, sim_ptr->planning_horizon);
//    printf("Finish finding commit cuts!\n");
    auto plan_start = std::chrono::high_resolution_clock::now();
    while (not finished) {
        sum_sol = 0.0;
        auto curr_time = std::chrono::high_resolution_clock::now();
        auto curr_duration = std::chrono::duration_cast<std::chrono::microseconds>(curr_time - plan_start);
        auto curr_duration_seconds = std::chrono::duration<double>(curr_duration).count();
        if (curr_duration_seconds > sim_ptr->plan_time_limit) {
            plan_succedd_flag = false;
            break;
        }

        auto topo_start = std::chrono::high_resolution_clock::now();
        std::vector<agentOrder> solve_order;
        topo_graph->findSingleRoundOrder(solve_order);
        auto topo_stop = std::chrono::high_resolution_clock::now();
        auto topo_duration = std::chrono::duration_cast<std::chrono::microseconds>(topo_stop - topo_start);
        auto topo_duration_seconds = std::chrono::duration<double>(topo_duration).count();
        topo_runtime += topo_duration_seconds;
        finished = true;
        for (auto& agent_order: solve_order) {
            // Only part of the path that can be solved
#if DEBUG_SPEED_SOLVER
            printf("Finding path for agent: %d, with last index: %d, the last vertex in path: %d"
                   ", path length: %lu\n",
                agent_order.agent_id, agent_order.last_idx, topo_graph->end_vertex[agent_order.agent_id],
                (*graph_->paths)[agent_order.agent_id].size());
#endif
            Path full_path = (*graph_->paths)[agent_order.agent_id];

            if (curr_loc[agent_order.agent_id].idx >= full_path.size() or
                    curr_loc[agent_order.agent_id].idx >= topo_graph->end_vertex[agent_order.agent_id]) {
                continue;
            }
            int tmp_last_idx = min(agent_order.last_idx, static_cast<int>(full_path.size()));
            tmp_last_idx = min(tmp_last_idx, topo_graph->end_vertex[agent_order.agent_id]+1);
            if (tmp_last_idx <= topo_graph->end_vertex[agent_order.agent_id] and tmp_last_idx != full_path.size()) {
#if DEBUG_SPEED_SOLVER
                printf("Unfinished::agent %d, path from %d to %d, end vertex: %d, path length: %lu. "
                       "Newly solved edge count: %lu\n",
                       agent_order.agent_id, curr_loc[agent_order.agent_id].idx, tmp_last_idx,
                       topo_graph->end_vertex[agent_order.agent_id], full_path.size(),
                       agent_order.solved_edges.size());
#endif
                finished = false;
            }

            assert(curr_loc[agent_order.agent_id].idx <= tmp_last_idx);
            Path tmp_path(full_path.begin()+curr_loc[agent_order.agent_id].idx, full_path.begin() + tmp_last_idx);

#if DEBUG_SPEED_SOLVER
             printf("find path for agent: %d\n", agent_order.agent_id);
#endif
            bool debug_output = false;



            std::vector<temporalConstraint> tmp_edge_constraints;
            std::vector<action> tmp_exec_plan;
            TCT_ptr->getConstraints(agent_order.agent_id, curr_loc[agent_order.agent_id].idx, tmp_last_idx, tmp_edge_constraints);
            if (agent_order.agent_id == DEBUG_AGENT1 or agent_order.agent_id == DEBUG_AGENT2) {

                printf("Single::Round: agent %d, from %d to %d\n", agent_order.agent_id, curr_loc[agent_order.agent_id].idx, tmp_last_idx);
                debug_output = true;
            }

            if (tmp_path.size() <= 1) {
                continue;
            }
            auto ll_start = std::chrono::high_resolution_clock::now();
            auto tmp_solution = singleAgentSolver.run(tmp_path, tmp_edge_constraints,
                tmp_exec_plan, curr_loc[agent_order.agent_id].speed_level,
                curr_loc[agent_order.agent_id].min_t, debug_output);
            auto ll_stop = std::chrono::high_resolution_clock::now();
            // Calculate the duration
            auto ll_duration = std::chrono::duration_cast<std::chrono::microseconds>(ll_stop - ll_start);
            auto ll_duration_seconds = std::chrono::duration<double>(ll_duration).count();
            ll_runtime += ll_duration_seconds;

            if (tmp_solution.size() != tmp_path.size()) {
                printf("For agent %d, solution size: %lu, path size: %lu\n", agent_order.agent_id, tmp_solution.size(), tmp_path.size());
                std::cerr << "No solution found!" << std::endl;

                return false;
            }


            all_path_entry[agent_order.agent_id] = tmp_solution;
            if (agent_order.agent_id == DEBUG_AGENT1 or agent_order.agent_id == DEBUG_AGENT2) {
                printf("AGENT %d:\n", agent_order.agent_id);
                for (int j = 0; j < all_path_entry[agent_order.agent_id].size(); j++) {
                    printf("entry %d at (%d, %d) from %f -> %f;\n", curr_loc[agent_order.agent_id].idx + j,
                           ((*graph_->paths)[agent_order.agent_id].begin() + curr_loc[agent_order.agent_id].idx + j)->loc.first,
                           ((*graph_->paths)[agent_order.agent_id].begin() + curr_loc[agent_order.agent_id].idx + j)->loc.second,
                           all_path_entry[agent_order.agent_id][j].first, all_path_entry[agent_order.agent_id][j].second);
                }
                printf("\n");
            }
            if (tmp_solution.empty()) {
                printf("No solution found for Agent %d!\n", agent_order.agent_id);
                exit(0);
//                continue;
            }
            exec_actions[agent_order.agent_id] = tmp_exec_plan;
            // printf("solution cost for agent %d is: %f\n", agent_order.agent_id, tmp_solution.back().second);
            sum_sol += tmp_solution.back().second;
            insertEdge2Table(agent_order.solved_edges, tmp_solution);
            topo_graph->removeInvalidEdge(agent_order);
        }
#if DEBUG_SPEED_SOLVER
        printf("Single agent solver::finish round %d!\n", round_count);
#endif
        round_count++;
    }
#if DEBUG_SPEED_SOLVER
    printf("finish planning with solution cost: %f!\n", sum_sol);
#endif
    return plan_succedd_flag;
}

inline double normal_cdf(double x, double stddev = 1.0, double mean = 0.0) {
    return 0.5 * (1 + std::erf((x - mean) / (stddev * std::sqrt(2.0))));
}

double speedSolver::computeGaussianDelay(double dis1, double dis2) {
    double k = sim_ptr->gaussian_var;
    double var = dis1 + dis2;
    double diff_t1_t2 = 2 * k * var + EPS;
//    printf("Gaussian Delay is: %f\n", diff_t1_t2);
    return diff_t1_t2;
}



void speedSolver::insertEdge2Table(const std::vector<std::shared_ptr<topoEdge>>& resolved_edges, const std::deque<std::pair<double, double>>& plan) {

    for (auto& topo_edge: resolved_edges) {
        int tmp_from_idx = topo_edge->from_idx-1;
        std::pair<double, double> occupy_interval = plan[tmp_from_idx-curr_loc[topo_edge->from_agent].idx];
        // Add constraints to the original agent
        temporalConstraint org_constraint(tmp_from_idx, 0, occupy_interval.second+CONS_SLACK);
        double tmp_org_upper_bound = org_constraint.interval.second;
#if DEBUG_SPEED_SOLVER
        printf("Insert constraint for agent %d at idx %d: from %f to %f, it occupies %f -> %f\n",
            topo_edge.from_agent, tmp_from_idx, org_constraint.interval.first,
            org_constraint.interval.second, occupy_interval.first, occupy_interval.second);
#endif


        int debug_flag = 0;
        auto org_constraints_at_idx = TCT_ptr->table[topo_edge->from_agent].find(tmp_from_idx);
        if (org_constraints_at_idx != TCT_ptr->table[topo_edge->from_agent].end()) {
            auto constraint_agent = org_constraints_at_idx->second.find(topo_edge->out_agent);
            if (constraint_agent != org_constraints_at_idx->second.end()) {
                auto constraint_at_agent_at_idx = constraint_agent->second.find(topo_edge->out_idx);
                if (constraint_at_agent_at_idx != constraint_agent->second.end()) {
                    debug_flag = 1;
                    if (constraint_at_agent_at_idx->second.interval.second > org_constraint.interval.second) {
                        constraint_at_agent_at_idx->second = org_constraint;
                    } else {
                        tmp_org_upper_bound = constraint_at_agent_at_idx->second.interval.second;
                    }

                } else {
                    constraint_agent->second.insert(std::make_pair(topo_edge->out_idx, org_constraint));
                }
                debug_flag = 1;
            } else {
                std::map<int, temporalConstraint> tmp_constraint_map;
                tmp_constraint_map.insert(std::make_pair(topo_edge->out_idx, org_constraint));
                org_constraints_at_idx->second.insert(std::make_pair(topo_edge->out_agent, tmp_constraint_map));
            }
        } else {
            std::map<int, temporalConstraint> tmp_constraint_map;
            tmp_constraint_map.insert(std::make_pair(topo_edge->out_idx, org_constraint));
            std::map<int, std::map<int, temporalConstraint>> tmp_constraints;
            tmp_constraints.insert(std::make_pair(topo_edge->out_agent, tmp_constraint_map));
            TCT_ptr->table[topo_edge->from_agent].insert(std::make_pair(tmp_from_idx, tmp_constraints));
        }
        // Add constraints to the other agent
        double tmp_delay = 0.0;
        if (sim_ptr->delay_type_ == 1) {
            tmp_delay = computeGaussianDelay(topo_edge->from_idx-1- last_seen_locations[topo_edge->from_agent].idx,
                topo_edge->out_idx - last_seen_locations[topo_edge->out_agent].idx);
        }
        int tmp_out_idx = topo_edge->out_idx;
        if (topo_edge->from_agent == DEBUG_AGENT1 or topo_edge->out_agent == DEBUG_AGENT1) {
            printf("Insert constraint from agent %d at idx %d to agent %d at idx %d: from %f to %f, it occupies %f -> %f, delay safety is: %f\n",
                   topo_edge->from_agent, tmp_from_idx,
                   topo_edge->out_agent, topo_edge->out_idx, org_constraint.interval.first,
                   org_constraint.interval.second, occupy_interval.first, occupy_interval.second,
                   tmp_delay);
        }
        temporalConstraint new_constraint(tmp_out_idx, tmp_org_upper_bound + CONS_SLACK + tmp_delay, INF);
#if DEBUG_SPEED_SOLVER
        printf("Insert constraint for agent %d at idx: %d: from %f to %f\n", topo_edge.out_agent,
            tmp_out_idx, new_constraint.interval.first, new_constraint.interval.second);
#endif

        auto constraints_at_idx = TCT_ptr->table[topo_edge->out_agent].find(tmp_out_idx);
        if (constraints_at_idx != TCT_ptr->table[topo_edge->out_agent].end()) {
            auto constraint_agent = constraints_at_idx->second.find(topo_edge->from_agent);
            if (constraint_agent != constraints_at_idx->second.end()) {

                auto constraint_at_agent_at_idx = constraint_agent->second.find(tmp_from_idx);
                if (constraint_at_agent_at_idx != constraint_agent->second.end()) {
                    constraint_at_agent_at_idx->second = new_constraint;
                    if (new_constraint.interval.first > constraint_at_agent_at_idx->second.interval.first) {
                        printf("Temporal constraints increase by %f\n", new_constraint.interval.first -
                        constraint_at_agent_at_idx->second.interval.first);
                    }
                    assert(debug_flag == 1);
                } else {
                    constraint_agent->second.insert(std::make_pair(tmp_from_idx, new_constraint));
                }

            } else {
                std::map<int, temporalConstraint> tmp_constraint_map;
                tmp_constraint_map.insert(std::make_pair(tmp_from_idx, new_constraint));
                constraints_at_idx->second.insert(std::make_pair(topo_edge->from_agent, tmp_constraint_map));
            }
        } else {

            std::map<int, temporalConstraint> tmp_constraint_map;
            tmp_constraint_map.insert(std::make_pair(tmp_from_idx, new_constraint));
            std::map<int, std::map<int, temporalConstraint>> tmp_constraints;
            tmp_constraints.insert(std::make_pair(topo_edge->from_agent, tmp_constraint_map));
            TCT_ptr->table[topo_edge->out_agent].insert(std::make_pair(tmp_out_idx, tmp_constraints));

        }
    }
}