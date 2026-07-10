#pragma once

#include <iostream>
#include <algorithm>
#include <set>
#include <utility>
#include <vector>
#include <chrono>
#include <fstream>
// #include <iomanip>
#include <cassert>
#include <cstdio>
#include <queue>
#include <unordered_set>

#include "define.h"
// #include "agent.h"

using namespace std;
using namespace std::chrono;
// Horizon timestep. This equals to INF/F (seconds).
// Please be attention that this variable affects the stored memory in generate_obstacles file.

#define DEBUG_SIPP_IP 0
#define SUBOPTIMAL_BOUND 10.0

struct action {
    action(int level_s, int level_g, int rotate, double v_s, double v_g, double cost):
            level_s(level_s), level_g(level_g), rotate(rotate), v_s(v_s), v_g(v_g), cost(cost){}
    int level_s, level_g;
    // 0: forward, 1: left, 2: right, 3: backward
    int rotate = 0;
    int from_orient=0, to_orient=0;
    double v_s, v_g, cost;
    double start_t = 0.0;
};

class Primitive{
public:
    struct move{
        move(int level_s, int level_g, double v_s, double v_g, double cost):
            level_s(level_s), level_g(level_g), v_s(v_s), v_g(v_g), cost(cost){}
        int level_s, level_g;
        double v_s, v_g, cost;
    };

    vector<move> mvs;
    int v;
    Primitive(){
        mvs.clear();
        v = 0;
    }
    Primitive(vector<move> mvs, int v): mvs(std::move(mvs)),v(v){}
    friend ostream &operator<<( ostream &output, const Primitive &p ) {
        output << "Moves:"<<endl;
        for(auto const it:p.mvs){
            output << "  start v:"<<it.v_s<<", end v:"<<it.v_g<<", cost:"<<it.cost<<endl;
        }
        // output<<"Final_o:"<<p.o<<", Final_v:"<<p.v<<endl;
        return output;
    }
};

struct sippInterval{
    double t_lower = 0.0, t_upper = 0.0;
//    int interval_idx = -1;
    std::pair<double, double> safe_interval;
    sippInterval() = default;
    sippInterval(double t_l, double t_u, std::pair<double, double> safe_interval):
    t_lower(t_l), t_upper(t_u), safe_interval(safe_interval) {}

    bool operator<(const sippInterval& interval) const {
        if (t_lower != interval.t_lower) {
            return t_lower < interval.t_lower;
        } else {
            return t_upper < interval.t_upper;
        }
    }
};


class sippNode{
public:
    int v_level = 0, idx = 0;
    double f = 0.0;
    double h = 0.0;
    action prev_act;
    std::shared_ptr<sippNode> parent = nullptr;
    sippInterval interval;
    sippNode(int v_l, sippInterval init_it, double transition_cost, action act):
        v_level(v_l), h(transition_cost), prev_act(act), interval(init_it) {
        f = init_it.t_lower + h;
    }

    sippNode(int v_l, const sippInterval &interval, std::shared_ptr<sippNode> parentID, double transition_cost, action act):
        idx(parentID->idx+1), v_level(v_l), h(transition_cost),
        parent(std::move(parentID)), interval(interval), prev_act(act) {
        f = interval.t_lower + h;
    }

    sippNode(const sippNode&) = default;
    sippNode &operator=(const sippNode&other)= default;
    friend bool operator == (const sippNode& a, const sippNode& b)
    {
        return a.idx == b.idx && a.v_level == b.v_level &&
             a.interval.t_lower == b.interval.t_lower && a.interval.t_upper == b.interval.t_upper && a.parent==b.parent;
    }
    friend ostream &operator<<( ostream &output, const sippNode &p ) {
        output<<"idx="<<p.idx<<", v_level="<<p.v_level<<", t_lower="<<p.interval.t_lower<<", t_upper="<<p.interval.t_upper;
        return output;
    }
};

class NodeCompare
{
public:
    bool operator() (const std::shared_ptr<sippNode> &N1, const std::shared_ptr<sippNode> &N2) const {
        if (N1->f > N2->f) {
            return true;
        } else if (N1->f < N2->f) {
            return false;
        } else {
            if (N1->idx < N2->idx) {
                return true;
            }

            if (N1->v_level > N2->v_level) {
                return true;
            }

            if (N1->interval.t_lower > N2->interval.t_lower) {
                return true;
            }


            return false;
        }
    }
};

struct sippNodeEqual{
    bool operator()(const std::shared_ptr<sippNode>& lhs, const std::shared_ptr<sippNode>& rhs) const {
        return lhs->idx == rhs->idx and lhs->v_level == rhs->v_level;
    }
};


class SIPP_IP
{
public:
    SIPP_IP();
    std::deque<std::pair<double, double>> run(const Path& path, std::vector<temporalConstraint>& constraints,
        std::vector<action>& exec_plan, int init_v, double init_min_t, bool debug = false);
    void minimumStopTime(double init_min_t, int init_v, std::vector<std::pair<double, double>>& time_occupied_before_stop) {
        assert(init_v != 0);

    }
private:
    void generateSuccessors(const std::shared_ptr<sippNode>& curr_node, vector<std::shared_ptr<sippNode>>& succs);
    void fillActions();
    double getHeuristic(int idx, double t_lower);
    void RetrivePath(const std::shared_ptr<sippNode> &goal_node, std::deque<std::pair<double, double>>& occupy_intervals, std::vector<action>& timed_plan);
    void constraintToRSRV(const std::vector<temporalConstraint>& constraints);
    bool dominanceCheck(const std::shared_ptr<sippNode> &curr_node);
    /**
     * @brief Check if two interval intersects
     *
     * @param a first interval
     * @param b second interval
     * @return Bool variable determine if they intesect
     */
    static std::pair<double, double> findIntersection(const std::pair<double, double>& a, const std::pair<double, double>& b) {
        double start = std::max(a.first, b.first);
        double end = std::min(a.second, b.second);
        return {start, end};
    }
    /**
         * @brief Find all intervals that intersect with a certain interval
         *
         * @param intervals Free intervals in the interval table
         * @param target Target interval
         * @return Intervals that insect with target interval
         */
    std::set<sippInterval> findIntersections(const std::set<std::pair<double, double>>& intervals, const std::pair<double, double>& target) {
        std::set<sippInterval> result;

        for (const auto& interval : intervals) {
            auto intersection = findIntersection(interval, target);
            if (intersection.first < intersection.second) { // Valid intersection
                sippInterval tmp_interval(intersection.first, intersection.second, interval);
                result.insert(tmp_interval);
            }
        }

        return result;
    }

    std::set<std::pair<double, double>> findFreeIntervals(const std::set<std::pair<double, double>>& intervals, const std::pair<double, double>& target) {
        std::set<std::pair<double, double>> result;

        for (const auto& interval : intervals) {
            auto intersection = findIntersection(interval, target);
            if (intersection.first < intersection.second) { // Valid intersection
                result.insert(intersection);
            }
        }

        return result;
    }
public:
    int low_node_generated = 0;
    int low_node_expanded = 0;
    int num_low_level_calls = 0;

private:
    vector<Primitive> motion_primitives;
    std::vector<std::set<pair<double, double>>> free_interval_tbl;
    Path path_;
    std::priority_queue<std::shared_ptr<sippNode>, std::vector<std::shared_ptr<sippNode>>, NodeCompare> OPEN;
    std::vector< std::vector< std::set< std::shared_ptr<sippNode>> > > CLOSED;

    // variables for motion primitives
    int state_levels;
    double v_max = V_MAX;
    double v_min = V_MIN;
    double a_max = A_MAX;
    double a_min = -A_MAX;
    double end_v = 0.0;
    bool debug_sipp_ip = false;
};