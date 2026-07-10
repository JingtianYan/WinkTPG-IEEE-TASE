#include "sipp_ip.h"

SIPP_IP::SIPP_IP() {
    state_levels = 3;
    assert(state_levels >= 2);
    fillActions();
}


void SIPP_IP::generateSuccessors(const std::shared_ptr<sippNode>& curr_node, vector<std::shared_ptr<sippNode>>& succs) {
    int curr_idx = curr_node->idx;
    if (curr_idx >= path_.size()-1) {
        return;
    }
    auto& curr_entry = path_[curr_idx];
    auto& next_entry = path_[curr_idx+1];
    if (curr_entry.loc == next_entry.loc) {
        if (IS_ROTATE and curr_node->v_level != 0) {
            succs.clear();
            return;
        }
        assert(curr_entry.o != next_entry.o);
        double t_lower = curr_node->interval.t_lower;
        int rotate_flag = 0;
        double act_cost = 0.0;
        if (((curr_entry.o+4)-next_entry.o)%2 == 0) {
            // rotate 180 degree
            act_cost = TURN_BACK_COST;
            rotate_flag = 3;
        } else {
            // rotate 90 degree
            act_cost = ROTATE_COST;
            if (((curr_entry.o+4)-next_entry.o)%4 == 1) {
                rotate_flag = 1;
            } else {
                rotate_flag = 2;
            }
        }
        t_lower += act_cost;
        action new_act(curr_node->v_level, curr_node->v_level, rotate_flag, 0, 0, act_cost);
        new_act.from_orient = curr_entry.o;
        new_act.to_orient = next_entry.o;
        std::set<sippInterval> overlap_bound =
                findIntersections(free_interval_tbl[curr_idx+1],
                                  std::make_pair(t_lower, curr_node->interval.t_upper));
        for (auto tmp_interval = overlap_bound.begin(); tmp_interval != overlap_bound.end(); tmp_interval++) {
            double tmp_t_upper;
            if (tmp_interval == (overlap_bound.end()--)) {
                tmp_t_upper = std::max(tmp_interval->t_upper, tmp_interval->safe_interval.second);
            } else {
                tmp_t_upper = tmp_interval->t_upper;
            }
            sippInterval new_interval(tmp_interval->t_lower, tmp_t_upper, tmp_interval->safe_interval);
            std::shared_ptr<sippNode> succ = std::make_shared<sippNode>(
                curr_node->v_level, new_interval, curr_node, getHeuristic(curr_node->idx+1, new_interval.t_lower), new_act);
            succs.push_back(succ);
        }
    } else {
        for (auto& mp: motion_primitives[curr_node->v_level].mvs) {
            // determine if the agent can perform mp at the current node
#if DEBUG_SIPP_IP
            printf("motion with cost: %f\n", mp.cost);
#endif
            if ((curr_node->interval.safe_interval.second - curr_node->interval.t_lower) < mp.cost) {
                continue;
            }
            double t_lower = curr_node->interval.t_lower + mp.cost;
            double t_upper = min(curr_node->interval.t_upper, curr_node->interval.safe_interval.second-mp.cost) + mp.cost;
#if DEBUG_SIPP_IP
            printf("ORG duration: %f -> %f\n", curr_node->interval.t_lower, t_upper);
#endif
            // state as the time interval at that node
            std::set<sippInterval> overlap_bound =
                        findIntersections(free_interval_tbl[curr_idx+1],
                                          std::make_pair(curr_node->interval.t_lower, t_upper));
            std::set<sippInterval> new_bound;
            for (auto& interval: overlap_bound) {
#if DEBUG_SIPP_IP
                printf("overlap bound from %f -> %f\n", interval.t_lower, interval.t_upper);
#endif
                double tmp_lower_bound = max(interval.t_lower + mp.cost, t_lower);
#if DEBUG_SIPP_IP
                printf("tmp lower bound: %f, previous lower bound: %f\n", tmp_lower_bound, curr_node->interval.t_lower);
#endif
                if (tmp_lower_bound < interval.t_upper) {
#if DEBUG_SIPP_IP
                    printf("insert next from %f to %f\n", tmp_lower_bound, interval.t_upper);
#endif
                    new_bound.insert(sippInterval(tmp_lower_bound, interval.t_upper, interval.safe_interval));
                }
            }

            // Indicate zero speed
            if (mp.level_g == 0 and not new_bound.empty()) {
                auto last_interval = new_bound.end();
                --last_interval;
                for (const auto &[fst, snd] : free_interval_tbl[curr_idx+1]) {
                    if (fst <= last_interval->t_lower and snd >= last_interval->t_upper) {
                        auto tmp_interval = sippInterval(last_interval->t_lower, snd, last_interval->safe_interval);
                        new_bound.erase(last_interval);
                        new_bound.insert(tmp_interval);
                    }
                }

            }
            action new_act(mp.level_s, mp.level_g, 0, mp.v_s, mp.v_g, mp.cost);
            new_act.from_orient = curr_entry.o;
            new_act.to_orient = next_entry.o;
            for (auto tmp_interval: new_bound) {
                std::shared_ptr<sippNode> succ = std::make_shared<sippNode>(
                    mp.level_g, tmp_interval, curr_node, getHeuristic(curr_node->idx+1, tmp_interval.t_lower), new_act);
#if DEBUG_SIPP_IP
                printf("insert::success with index: %d speed level: %d, interval: %f, %f, safe interval: %f -> %f\n",
                    succ->idx, succ->v_level, succ->interval.t_lower, succ->interval.t_upper,
                    succ->interval.safe_interval.first, succ->interval.safe_interval.second);
#endif
                succs.push_back(succ);
            }
        }
    }
}

void SIPP_IP::constraintToRSRV(const std::vector<temporalConstraint>& constraints)
{
    free_interval_tbl.resize(path_.size());
    for (int i = 0; i < path_.size(); i++) {
        free_interval_tbl[i].emplace(0,INF);
    }

    for (auto& tmp_con: constraints) {
        if (tmp_con.goal_path_idx < path_.size()) {
            free_interval_tbl[tmp_con.goal_path_idx] = findFreeIntervals(free_interval_tbl[tmp_con.goal_path_idx], tmp_con.interval);
        }
    }
#if DEBUG_SIPP_IP
    for (int i = 0; i < path_.size(); i++) {
        for (auto& item: free_interval_tbl[i]) {
            printf("SIPP-IP::Free interval at %d: %f -> %f\n", i,
                item.first, item.second);
        }
    }
#endif
}

bool SIPP_IP::dominanceCheck(const std::shared_ptr<sippNode> &curr_node) {
    // printf("Dominance::curr_node: (%f, %f)\n", curr_node->t_lower, curr_node->t_upper);
    for (auto it: CLOSED[curr_node->idx][curr_node->v_level]) {
        // printf("Dominance::it: (%f, %f), is true: %d\n", it->t_lower, it->t_upper, it->t_lower <= curr_node->t_lower and it->t_upper >= curr_node->t_upper);
        if (it->interval.t_lower <= curr_node->interval.t_lower and it->interval.t_upper >= curr_node->interval.t_upper) {
#if DEBUG_SIPP_IP
            printf("Found closed state with index: %d speed level: %d, interval: %f, %f, safe interval: %f -> %f\n",
            it->idx, it->v_level,
            it->interval.t_lower, it->interval.t_upper, it->interval.safe_interval.first,
            it->interval.safe_interval.second);
#endif
            return true;
        }
    }

    return false;
}

double SIPP_IP::getHeuristic(int idx, double t_lower) {
    if (idx >= path_.size() - 1) {
        return 0.0;
    }
    double togo_h = (static_cast<double>(path_.size()) - idx - 1.0)/V_MAX*SUBOPTIMAL_BOUND;
    double max_time = -1.0;
    for (int i = idx; i < path_.size()-1; i++) {
        max_time = std::max(free_interval_tbl[i+1].begin()->first + (path_.size()-i-1)/V_MAX*SUBOPTIMAL_BOUND, max_time);
    }
    double time_h = max_time - t_lower;
    return std::max(togo_h, time_h) + EPS*(path_.size()-idx);
}


std::deque<std::pair<double, double>> SIPP_IP::run(const Path& path,
    std::vector<temporalConstraint>& constraints,
    std::vector<action>& exec_plan,
    int init_v,
    double init_min_t,
    bool debug) {
    debug_sipp_ip = debug;
    num_low_level_calls++;
#if DEBUG_SIPP_IP
    printf("SIPP_IP::Input of single agent solver:\n");
    for (auto tmp_con : constraints) {
        printf("Free interval at %d, from: %f to %f\n", tmp_con.goal_path_idx, tmp_con.interval.first,
            tmp_con.interval.second);
    }
    for (int i = 0; i < path.size(); i++) {
        printf("entry %d at (%d, %d)\n", i, path[i].loc.first, path[i].loc.second);
    }
#endif

    path_ = path;

    low_node_generated = 0;
    low_node_expanded = 0;
    free_interval_tbl.clear();
    std::priority_queue<std::shared_ptr<sippNode>, std::vector<std::shared_ptr<sippNode>>, NodeCompare> empty_pq;
    OPEN.swap(empty_pq);
    CLOSED.clear();
    CLOSED.resize(path_.size());
    for (int i = 0; i < path_.size(); i++) {
        CLOSED[i].resize(state_levels);
    }
    constraintToRSRV(constraints);

    if (debug_sipp_ip) {
        printf("SIPP_IP::Input of single agent solver:\n");
        printf("Init speed: %d, init start min time: %f\n", init_v, init_min_t);

        for (int i = 0; i < path.size(); i++) {
            printf("entry %d at (%d, %d) with free intervals: ", i, path[i].loc.first, path[i].loc.second);
            for (auto& free_entry: free_interval_tbl[i]) {
                printf("(%f, %f)\t", free_entry.first, free_entry.second);
            }
            printf("\n");
        }
    }

    double init_upper_bound = INF;
    if (init_v != 0) {
        init_upper_bound = 0.0;
        for (auto& mp: motion_primitives[init_v].mvs) {
            if (mp.cost > init_upper_bound) {
                init_upper_bound = mp.cost;
            }
        }
        init_upper_bound += init_min_t;
        init_upper_bound += EPS;
    }
    auto free_intervals = findIntersections(free_interval_tbl[0], std::make_pair(init_min_t, init_upper_bound));

    auto it = free_intervals.begin();


    if (it->t_lower != init_min_t) {
        printf("Impossible init_min_t, bugs in the code!\n");
        std::deque<std::pair<double, double>> result_empty_path;
        return result_empty_path;
    }
    action init_act(init_v, init_v, 0, 0, 0, 0);
    OPEN.push(std::make_shared<sippNode>(init_v, *it, getHeuristic(0, it->t_lower), init_act));
    int cntNodes = 0;

    auto plan_start = std::chrono::high_resolution_clock::now();
    while(!OPEN.empty()){
        auto curr_time = std::chrono::high_resolution_clock::now();
        auto curr_duration = std::chrono::duration_cast<std::chrono::microseconds>(curr_time - plan_start);
        auto curr_duration_seconds = std::chrono::duration<double>(curr_duration).count();
        if (curr_duration_seconds > 60) {
            std::deque<std::pair<double, double>> result_empty_path;
            std::cout << "hit max planning time" << std::endl;
            return result_empty_path;
        }
        auto tmp = OPEN.top();
        OPEN.pop();
#if DEBUG_SIPP_IP
        printf("pop out new state with index: %d speed level: %d, interval: %f, %f, safe interval: %f -> %f\n",
            tmp->idx, tmp->v_level,
            tmp->interval.t_lower, tmp->interval.t_upper, tmp->interval.safe_interval.first,
            tmp->interval.safe_interval.second);
#endif
        if (debug_sipp_ip) {
            printf("pop out new state with index: %d speed level: %d, interval: %f, %f, safe interval: %f -> %f"
                   ", heuristic: %f, f: %f\n",
            tmp->idx, tmp->v_level,
            tmp->interval.t_lower, tmp->interval.t_upper, tmp->interval.safe_interval.first,
            tmp->interval.safe_interval.second,
            tmp->h, tmp->f);
        }

        if (dominanceCheck(tmp)) {
            continue;
        }
        CLOSED[tmp->idx][tmp->v_level].insert(tmp);


        if(tmp->idx == (path_.size()-1) and tmp->v_level == 0){
            std::deque<std::pair<double, double>> result_path;
            RetrivePath(tmp, result_path, exec_plan);
            return result_path;
        }
        vector<std::shared_ptr<sippNode>> succs;
        low_node_expanded++;
        generateSuccessors(tmp, succs);
        for(auto &succ:succs){
            OPEN.push(succ);
#if DEBUG_SIPP_IP
            printf("Generate Succ at %d with speed %d, (%f, %f) with prev cost: %f\n", succ->idx, succ->v_level,
                succ->interval.t_lower, succ->interval.t_upper, succ->prev_act.cost);
#endif
            if (debug_sipp_ip) {
                printf("Generate Succ at %d with speed %d, (%f, %f) with prev cost: %f and heuristic: %f, f: %f\n", succ->idx, succ->v_level,
                       succ->interval.t_lower, succ->interval.t_upper, succ->prev_act.cost, succ->h, succ->f);
            }
            ++cntNodes;
            low_node_generated++;
        }
    }
    std::deque<std::pair<double, double>> result_empty_path;
#if DEBUG_SIPP_IP
    printf("[INFO] No solution find!\n");
    for (auto tmp_con : constraints) {
        printf("Free interval at %d, from: %f to %f\n", tmp_con.goal_path_idx, tmp_con.interval.first,
            tmp_con.interval.second);
    }
#endif

    return result_empty_path;
}

void SIPP_IP::fillActions() {
    std::vector<double> speed_levels;
    speed_levels.resize(state_levels);

    std::vector<double> acc_cost_levels;
    acc_cost_levels.resize(state_levels-1);

    double tmp_v = v_min;
    for (int i = 0; i < state_levels; i++) {
        speed_levels[i] = tmp_v;
        double v_acc = sqrt(pow(tmp_v, 2) + 2*a_max);
        double acc_cost = (v_acc - tmp_v)/a_max;
        if (i == state_levels - 1) {
            continue;
        }
        acc_cost_levels[i] = acc_cost;
        tmp_v = v_acc;
    }
    motion_primitives.resize(state_levels);
    for (int i = 0; i < state_levels; i++) {
        Primitive tmp;
        // Acc and dcc
        if (i == 0) {
            tmp.mvs.emplace_back(i, i, speed_levels[i], speed_levels[i], 2*std::sqrt(1/a_max));
        }
        // Acc
        if (i != state_levels - 1) {
            tmp.mvs.emplace_back(i, i+1, speed_levels[i], speed_levels[i+1], acc_cost_levels[i]);
        }
        // Dec & uniform
        if (i != 0) {
            // uniform
            tmp.mvs.emplace_back(i, i, speed_levels[i], speed_levels[i], 1.0/speed_levels[i]);
            // Dec
            tmp.mvs.emplace_back(i, i-1, speed_levels[i], speed_levels[i-1], acc_cost_levels[i-1]);
        }
        motion_primitives[i] = tmp;
    }
#if DEBUG_SIPP_IP
    printf("start filling actions, with %d levels\n", state_levels);
    printf("showing all actions:\n");
    for (auto& prim: motion_primitives) {
        std::cout << prim;
    }
#endif
}

double calculateTransitionTime(double initial_speed, double target_speed) {
    // If initial speed == target speed and moving at constant speed
    if (initial_speed == target_speed) {
        if (initial_speed == 0) {
            return 2*sqrt(DISTANCE/A_MAX);
        }
        return DISTANCE / initial_speed;
    }

    // Calculate using kinematic equations: v^2 = u^2 + 2as and s = ut + 0.5at^2
    double acceleration = (target_speed > initial_speed) ? A_MAX : -A_MAX;
    double time = (target_speed - initial_speed) / acceleration;
    double distance = (initial_speed * time) + (0.5 * acceleration * time * time);
    if (distance > DISTANCE+EPS) {
        // If the distance is sufficient, return the time
        return -1;
    } else {
        // Otherwise, calculate remaining distance to cover at constant speed
        double remaining_distance = DISTANCE - distance;
        return time + (remaining_distance / std::max(initial_speed, target_speed));
    }
}



void SIPP_IP::RetrivePath(const std::shared_ptr<sippNode> &goal_node,
                          std::deque<std::pair<double, double>>& path_intervals,
                          std::vector<action>& timed_plan) {
    std::shared_ptr<sippNode> it_node = goal_node;
#if DEBUG_SIPP_IP
    printf("solution found!\n");
#endif
    std::deque<std::shared_ptr<sippNode>> path_nodes;
    path_intervals.clear();
    // time agent arrive at its goal
    double tmp_t = goal_node->interval.t_lower;
    double tmp_t_arr = tmp_t;
    double tmp_t_leave = tmp_t;
    double tmp_t_lower = tmp_t_arr-it_node->prev_act.cost;
    double tmp_t_upper = tmp_t;
    path_intervals.emplace_front(tmp_t_lower, tmp_t_upper);
    while (true) {
        tmp_t_leave = tmp_t_arr-it_node->prev_act.cost;
        tmp_t_upper = tmp_t_leave + it_node->prev_act.cost;
        path_nodes.push_front(it_node);
        it_node = it_node->parent;
        if (it_node == nullptr) {
            break;
        }
#if DEBUG_SIPP_IP
        printf("At idx %d, leave time: %f, interval lower: %f\n", it_node->idx, tmp_t_arr, tmp_t_leave);
#endif
        tmp_t_arr = tmp_t_leave;
        if (it_node->v_level == 0) {
            tmp_t_arr = min(tmp_t_leave, it_node->interval.t_lower);
        }
        tmp_t_lower = tmp_t_arr - it_node->prev_act.cost;
#if DEBUG_SIPP_IP
        printf("lower time: %f, interval upper: %f\n", tmp_t_lower, tmp_t_upper);
#endif
        path_intervals.emplace_front(tmp_t_lower, tmp_t_upper);
    }
    for (size_t i = 1; i < path_nodes.size(); i++) {
        action tmp_action = path_nodes[i]->prev_act;
        tmp_action.start_t = path_intervals[i].first;
        timed_plan.push_back(tmp_action);
    }

#if DEBUG_SIPP_IP
    for (int i = 0; i < path_intervals.size(); i++) {
        printf("Solution node with index: %d, speed level: %d, interval: %f, %f, "
               "occupy node at: (%f, %f)\n", path_nodes[i]->idx, path_nodes[i]->v_level,
               path_nodes[i]->interval.t_lower, path_nodes[i]->interval.t_upper,
               path_intervals[i].first, path_intervals[i].second);
    }
    for (auto& tmp_act: timed_plan) {
        printf("Perform action with start velocity: %d, end velocity: %d, rotate: %d, from orient %d to %d; "
               "time cost: %f, start time: %f\n",
            tmp_act.level_s, tmp_act.level_g, tmp_act.rotate, tmp_act.from_orient, tmp_act.to_orient,
            tmp_act.cost, tmp_act.start_t);
    }
#endif
    if (debug_sipp_ip) {
        for (int i = 0; i < path_intervals.size(); i++) {
            printf("Solution node with index: %d, speed level: %d, interval: %f, %f, "
                   "occupy node at: (%f, %f)\n", path_nodes[i]->idx, path_nodes[i]->v_level,
                   path_nodes[i]->interval.t_lower, path_nodes[i]->interval.t_upper,
                   path_intervals[i].first, path_intervals[i].second);
        }
        for (auto& tmp_act: timed_plan) {
            printf("Perform action with start velocity: %d, end velocity: %d, rotate: %d, from orient %d to %d; "
                   "time cost: %f, start time: %f\n",
                   tmp_act.level_s, tmp_act.level_g, tmp_act.rotate, tmp_act.from_orient, tmp_act.to_orient,
                   tmp_act.cost, tmp_act.start_t);
        }
    }
}

void showPathIntervals(std::vector<std::pair<double, double>>& path_intervals) {
}