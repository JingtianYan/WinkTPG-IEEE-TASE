#pragma once
#include <utility>
#include <vector>
#include <random>
#include "define.h"
#include "sipp_ip.h"
// #include "tpg_execute.h"

// #define DEBUG_SIM
// Structure to represent a time interval [start, end]
struct TimeInterval {
    double start;
    double end;
    int agent_id;
    int entry_idx;
    // Constructor for easy initialization
    TimeInterval(double s, double e, int id, int entry) : start(s), end(e), agent_id(id), entry_idx(entry) {}
};


struct hashLocation {
    std::size_t operator()(const Location & coordinates) const {
        std::size_t hash = 1001;  // Prime number for hashing
        hash = hash * coordinates.first + coordinates.second; // Hash the second element (y)
        return hash;
    }
};

// Custom equality function for std::pair<int, int>
struct equalLocation {
    bool operator()(const Location& a, const Location& b) const {
        return a.first == b.first && a.second == b.second;
    }
};

struct simState {
    Location loc;
    int orient=-1;
    int idx=0;
    int v =0;
    double time=0.0;
    simState()=default;
    simState(Location loc, int orient, double time): loc(std::move(loc)), orient(orient), time(time) {}
};

class simEnv{
public:
    simEnv(int num_agent, std::vector<std::pair<Location, orient>>& init_locs,
        std::vector<std::pair<Location, orient>>& goal_locs, double planning_horizon,
        int execution_horizon, int planner_type, int delay_type,
        double delay_gaussian_thresh, double delay_var);

    void getPrevLocations(std::vector<simState>& last_locs, std::vector<double>& acc_arr_time) const;

    void getEnqueActions(std::vector<std::deque<action>>& enque_actions) const;

    // Move all robot by a single time step
    void updateSingleRound(double step_size=0.0);

    // Enque some actions into the list
    void enqueActions(std::vector<std::vector<action>>& input_actions);

    [[nodiscard]] bool isAllFinished() {
        bool all_finished = true;
        for (size_t i=0; i < num_agent_; i++) {
            if (curr_state[i].loc != goal_locs_[i].first or curr_state[i].orient != goal_locs_[i].second) {
                all_finished = false;
                agent_finish_time[i] = -1;
                break;
            } else {
                if (agent_finish_time[i] < 0) {
                    agent_finish_time[i] = curr_state[i].time;
                }
            }
        }
        return  all_finished;
    }

    double getRuntime() {
#ifdef DEBUG_SIM
        for (int i = 0; i < agent_finish_time.size(); i++) {
            printf("finish time for agent %d is: %f\n", i, agent_finish_time[i]);
        }
#endif
        double sum_of_cost = std::accumulate(agent_finish_time.begin(), agent_finish_time.end(), 0.0);
//        printf("Sum of all agents finish time: %f\n", sum_of_cost);
        return  sum_of_cost;
    }

    [[nodiscard]] double getSimTime() const {
        return sim_time;
    }

private:
    // Get current locations of all robots
    void getCurrLocations(std::vector<simState>& curr_locs) const;

    double sampleGaussian() {
        double tmp_delay = distr_normal(gen);
        double tmp_max = 2*gaussian_var;
        return std::max(-tmp_max, std::min(tmp_max, tmp_delay));;
    }

    // Only Gaussian delay sampling is kept

    // Function to check if two intervals collide
    bool static isCollision(const TimeInterval& a, const TimeInterval& b) {
        return (a.start < b.end && b.start < a.end);
    }

    // Function to check for collisions in a list of time intervals for a given location
    bool static checkCollisions(const vector<TimeInterval>& intervals) {
        int n = intervals.size();

        // Compare all pairs of intervals
        for (int i = 0; i < n - 1; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (isCollision(intervals[i], intervals[j])) {
                    printf("collision::Agent %d, at %d from %f -> %f; Agent %d, at %d from %f->%f\n",
                           intervals[i].agent_id, intervals[i].entry_idx, intervals[i].start, intervals[i].end,
                           intervals[j].agent_id, intervals[j].entry_idx, intervals[j].start, intervals[j].end);
                    return true;  // Collision found
                }
            }
        }
        return false;  // No collision
    }

    /***
     *
     * @return true if collision free, false otherwise
     */
    bool checkOccupiedTblCollisionFree() {
        for (auto& it: occupied_intervals) {
            if (checkCollisions(it.second)) {
                printf("Find collison at (%d, %d):\t", it.first.first, it.first.second);
                return false;
            }
        }
        return true;
    }

    inline simState applyAction(int agent_id, const action& next_move) {
        // 0: forward, 1: left, 2: right, 3: backward
        const simState& tmp_prev_state = prev_state[agent_id];
        simState next_state = tmp_prev_state;
        if (agent_id == DEBUG_AGENT1 or agent_id == DEBUG_AGENT2) {
            printf("Perform action with start velocity: %d, end velocity: %d, rotate: %d, from orient %d to %d; "
                   "time cost: %f, start time: %f\n",
                   next_move.level_s, next_move.level_g, next_move.rotate, next_move.from_orient, next_move.to_orient,
                   next_move.cost, next_move.start_t);
        }
#ifdef DEBUG_SIM
        printf("Perform action with start velocity: %d, end velocity: %d, rotate: %d, from orient %d to %d; "
               "time cost: %f, start time: %f\n",
            next_move.level_s, next_move.level_g, next_move.rotate, next_move.from_orient, next_move.to_orient,
            next_move.cost, next_move.start_t);
#endif
        // assert(tmp_prev_state.time <= sim_time);
        double move_start_t = max(tmp_prev_state.time, next_move.start_t);
        next_state.idx = tmp_prev_state.idx + 1;
        assert(tmp_prev_state.v == next_move.level_s);
        if (next_move.rotate == 0) {
            int dx = (tmp_prev_state.orient%2)*(2-tmp_prev_state.orient);
            int dy = (tmp_prev_state.orient+1)%2*(1-tmp_prev_state.orient);
            next_state.loc = std::make_pair(tmp_prev_state.loc.first+dx, tmp_prev_state.loc.second+dy);
        } else if (next_move.rotate == 1) {
            next_state.orient = (tmp_prev_state.orient+3)%4;
        } else if (next_move.rotate == 2) {
            next_state.orient = (tmp_prev_state.orient+1)%4;
        } else if (next_move.rotate == 3) {
            next_state.orient = (tmp_prev_state.orient+2)%4;
        }
        next_state.time = move_start_t + next_move.cost;
        double delay = 0.0;
        // Only Gaussian delay model is supported now: 1 -> Gaussian, 0 -> no delay
        if (delay_type_ == 1) {
            delay = sampleGaussian();
        }
        next_state.time += delay;
//        accumulated_round_delay[agent_id] += delay;
        // printf("Sim::delay type: %lu, with value: %f\n", delay_type_, delay);
        next_state.v = next_move.level_g;
        return next_state;
    }
public:
    double planning_horizon;
    int execution_horizon;
    int delay_type_; // 0 for no delay, 1 for Gaussian delay
    double gaussian_var;
    double gaussian_thresh;

    double plan_time_limit = 300.0;

private:
    // std::vector<std::deque<action>> action_list;
    std::vector<int> accumulated_delay;
    std::vector<double> accumulated_round_delay;
    std::vector<double> agent_finish_time;
    int num_agent_;
    std::vector<simState> curr_state;
//    std::vector<simState> acc_curr_state;
    std::vector<simState> prev_state;
//    std::vector<simState> acc_prev_state;
    std::vector<double> acc_arr_last_loc_time;
    std::vector<std::deque<action>> enque_actions_;
    std::vector<std::pair<Location, orient>> goal_locs_;
    double sim_time;
    size_t update_round = 0;
    // std::random_device rd;  // Obtain a random number from hardware
    std::mt19937 gen; // Seed the generator
    std::normal_distribution<> distr_normal;
    std::unordered_map<Location, std::vector<TimeInterval>, hashLocation, equalLocation> occupied_intervals;
    int type_planner = 0; // 0 for tpg, 1 for ours
};