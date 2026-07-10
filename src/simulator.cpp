#include "simulator.h"

simEnv::simEnv(int num_agent,
    std::vector<std::pair<Location, orient>>& init_locs,
    std::vector<std::pair<Location, orient>>& goal_locs,
    double planning_horizon,
    int execution_horizon,
    int planner_type,
    int delay_type,
    double delay_gaussian_thresh,
    double delay_var): planning_horizon(planning_horizon), execution_horizon(execution_horizon),
    type_planner(planner_type), gen(std::random_device{}()), distr_normal(0.0, delay_var) {
    num_agent_ = num_agent;
    goal_locs_ = goal_locs;
    for (int i = 0; i < num_agent_; i++) {
        curr_state.emplace_back(init_locs[i].first, init_locs[i].second, 0.0);
        prev_state.emplace_back(init_locs[i].first, init_locs[i].second, 0.0);
        acc_arr_last_loc_time.push_back(0.0);
    }
    sim_time = 1e-8;
    enque_actions_.resize(num_agent_);
    delay_type_=delay_type;
    gaussian_thresh = delay_gaussian_thresh;
    gaussian_var = delay_var;
    accumulated_delay.resize(num_agent_, 0);
    accumulated_round_delay.resize(num_agent_, 0.0);
    agent_finish_time.resize(num_agent_, -1.0);
    assert(num_agent == init_locs.size());
}

void simEnv::updateSingleRound(double step_size)
{
    if (step_size == 0.0) {
        step_size = execution_horizon;
    }
    accumulated_round_delay.resize(num_agent_, 0.0);
    sim_time += step_size;
    occupied_intervals.clear();
#ifdef DEBUG_SIM
    printf("\n\nUpdate Single Round::time: %f\n", sim_time);
#endif

    for (int i = 0; i < num_agent_; i++) {
        if (i == DEBUG_AGENT1 or i == DEBUG_AGENT2) {
            printf("Agent %d start from x:%d, y:%d, and o:%d with velocity: %d; At time: %f\n", i,
                   prev_state[i].loc.first,
                   prev_state[i].loc.second, prev_state[i].orient, prev_state[i].v, prev_state[i].time);
        }
#ifdef DEBUG_SIM
        printf("Agent %d start from x:%d, y:%d, and o:%d with velocity: %d; At time: %f\n", i, prev_state[i].loc.first,
               prev_state[i].loc.second, prev_state[i].orient, prev_state[i].v, prev_state[i].time);
#endif
        if (curr_state[i].time <= sim_time) {
            // Move step by step
            auto& agent_action_list = enque_actions_[i];
            while (not agent_action_list.empty()) {
                curr_state[i] = applyAction(i, agent_action_list.front());
                if (curr_state[i].time > sim_time) {
                    break;
                }
                occupied_intervals[prev_state[i].loc].emplace_back(prev_state[i].time, curr_state[i].time, i, prev_state[i].idx);
                prev_state[i] = curr_state[i];
                agent_action_list.pop_front();
            }
        }

#ifdef DEBUG_SIM
        printf("Agent %d reach x:%d, y:%d, and o:%d with velocity: %d, with time: %f; Remaining actions: %lu\n",
            i, curr_state[i].loc.first,
            curr_state[i].loc.second, curr_state[i].orient, curr_state[i].v, curr_state[i].time,
            enque_actions_[i].size());
#endif
        if (i == DEBUG_AGENT1 or i == DEBUG_AGENT2) {
            printf("Agent %d reach x:%d, y:%d, and o:%d with velocity: %d, with time: %f; Remaining actions: %lu\n",
                   i, curr_state[i].loc.first,
                   curr_state[i].loc.second, curr_state[i].orient, curr_state[i].v, curr_state[i].time,
                   enque_actions_[i].size());
        }
    }
    if (not checkOccupiedTblCollisionFree()) {
        printf("Found collision in this round!\n");
        exit(-1);
    }
}

// Enque some actions into the list
void simEnv::enqueActions(std::vector<std::vector<action>>& input_actions) {
    for (int i = 0; i < num_agent_; i++) {
        // Move step by step
        auto& agent_action_list = enque_actions_[i];
        int enque_size = 0;
        if (type_planner == 0) {
            enque_size = static_cast<int>(input_actions[i].size());
        } else if (type_planner == 1) {
            int j;
            for (j = 0; j < input_actions[i].size(); j++) {
                if (input_actions[i][j].start_t > sim_time + execution_horizon) {
                    break;
                }
            }
            enque_size = std::min(static_cast<int>(input_actions[i].size()), execution_horizon);
        }
        std::move(input_actions[i].begin(), input_actions[i].begin() + enque_size, std::back_inserter(agent_action_list));
    }
    for (int i = 0; i < num_agent_; i++) {
        if (not enque_actions_[i].empty()) {
            acc_arr_last_loc_time[i] = enque_actions_[i].back().start_t + enque_actions_[i].back().cost;
        }
    }
#ifdef DEBUG_SIM
    for (size_t i = 0; i < input_actions.size(); i++) {
        printf("For agent %lu:\n", i);
        int enque_size = 0;
        if (type_planner == 0) {
            enque_size = static_cast<int>(input_actions[i].size());
        } else if (type_planner == 1) {
            enque_size = std::min(static_cast<int>(input_actions[i].size()), execution_horizon);
        }
        for (auto tmp_act = input_actions[i].begin(); tmp_act != input_actions[i].begin() + enque_size; tmp_act++) {
            printf("INPUT::Get Plan with action with start velocity: %d, end velocity: %d, rotate: %d, from orient %d to %d; "
                   "time cost: %f, start time: %f\n",
                tmp_act->level_s, tmp_act->level_g, tmp_act->rotate, tmp_act->from_orient, tmp_act->to_orient,
                tmp_act->cost, tmp_act->start_t);
        }
    }
#endif
    for (size_t i = 0; i < input_actions.size(); i++) {
        if (i == DEBUG_AGENT1 or i == DEBUG_AGENT2) {
            printf("For agent %lu:\n", i);
            int enque_size = 0;
            if (type_planner == 0) {
                enque_size = static_cast<int>(input_actions[i].size());
            } else if (type_planner == 1) {
                enque_size = std::min(static_cast<int>(input_actions[i].size()), execution_horizon);
            }
            for (auto tmp_act = input_actions[i].begin(); tmp_act != input_actions[i].begin() + enque_size; tmp_act++) {
                printf("INPUT::Get Plan with action with start velocity: %d, end velocity: %d, rotate: %d, from orient %d to %d; "
                       "time cost: %f, start time: %f\n",
                       tmp_act->level_s, tmp_act->level_g, tmp_act->rotate, tmp_act->from_orient, tmp_act->to_orient,
                       tmp_act->cost, tmp_act->start_t);
            }
        }
    }
}

void simEnv::getPrevLocations(std::vector<simState>& last_locs, std::vector<double>& acc_arr_time) const {
    last_locs = prev_state;
    acc_arr_time = acc_arr_last_loc_time;
#ifdef DEBUG_SIM
    for (size_t i = 0; i < prev_state.size(); i++) {
        printf("PREV::Agent %lu start from x:%d, y:%d, and o:%d with velocity: %d; At time: %f. Idx: %d\n", i,
            prev_state[i].loc.first, prev_state[i].loc.second, prev_state[i].orient, prev_state[i].v,
            prev_state[i].time, prev_state[i].idx);
    }
#endif
}

void simEnv::getCurrLocations(std::vector<simState>& curr_locs) const {
    curr_locs = curr_state;
}


void simEnv::getEnqueActions(std::vector<std::deque<action>>& enque_actions) const {
    enque_actions = enque_actions_;
}