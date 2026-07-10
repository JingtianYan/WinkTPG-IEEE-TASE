#pragma once
#include <map>
#include <cassert>


#define DEBUG_AGENT1 (-1)
#define DEBUG_AGENT2 (-1)

#define DEBUG_TEMPORAL 0

struct edgeConstraint {
    int from_agent;
    int goal_agent;
    int goal_path_idx;
    std::pair<double, double> interval;
    edgeConstraint(int from_a, int goal_a, int goal_idx, double t_min, double t_max):
        from_agent(from_a), goal_agent(goal_a), goal_path_idx(goal_idx) {
        interval.first = t_min;
        interval.second = t_max;
    }
};

struct temporalConstraint {
    int goal_path_idx;
    bool is_increase = false;
    std::pair<double, double> interval;
    temporalConstraint()=default;
    temporalConstraint(int goal_path_idx, double t_lower, double t_upper):
        goal_path_idx(goal_path_idx), interval(std::make_pair(t_lower, t_upper)) {}
};


class constraintTable {
public:
    explicit constraintTable(int num_of_agent) {
        table.resize(num_of_agent);
        num_of_agent_ = num_of_agent;
    }
    void getConstraints(int const agent_id, int start_time, int end_time, std::vector<temporalConstraint>& result_constraints) {
        result_constraints.clear();
        auto& entry = table[agent_id];
        for (auto& tmp_entry_at_idx: entry) {
            for (auto& tmp_entry_at_agent: tmp_entry_at_idx.second) {
                for (auto& elem: tmp_entry_at_agent.second) {
                    assert (elem.second.goal_path_idx == tmp_entry_at_idx.first);
                    if (start_time <= elem.second.goal_path_idx and elem.second.goal_path_idx < end_time) {
                        result_constraints.emplace_back(elem.second.goal_path_idx-start_time,
                                                        elem.second.interval.first, elem.second.interval.second);

                    }
                }
            }
        }
#if DEBUG_TEMPORAL
        for (const auto& tmp_constraint: result_constraints) {
            printf("Constraint at idx: %d, from %f to %f\n", tmp_constraint.goal_path_idx, tmp_constraint.interval.first,
                tmp_constraint.interval.second);
        }
#endif
    }
    void clear() {
        table.clear();
        table.resize(num_of_agent_);
    }
    // vector: num of agent, index in the path, constraining agent, constraining idx
    std::vector<std::map<int, std::map<int, std::map<int, temporalConstraint> >>> table;
    int num_of_agent_;
};