#pragma once
#include <cfloat>
#include <limits>
#include <utility>
#include <vector>
#include <memory>
#include <cmath>
#include <climits>

#include "temporal_constraint.h"

#define COST_TYPE float
#define DEFAULT_EDGE_COST 1
#define INF INT_MAX

#define V_MIN 0.0
#define V_MAX 2.0
#define A_MAX 1.0
#define DISTANCE 1.0

#define IS_ROTATE 1
#if IS_ROTATE == 1
#define ROTATE_COST 0.5
#define TURN_BACK_COST 0.9
#else
#define ROTATE_COST 0.0
#define TURN_BACK_COST 0.0
#endif

#define EPS 0.0001
#define CONS_SLACK 0.001



// CASE 1
#define K_DELAY 1.0

// CASE 2
// #define P_THRESH 0.05
// #define DELAY_NOISE_RATIO 0.01

// CASE 3
#define STN_MIN (-0.01)
#define STN_MAX (0.01)

enum orient {
    N = 0,
    E = 1,
    S = 2,
    W = 3,
    None = 4
};

struct runtimeStats {
    double runtime;
    double runtime_sim;
    double sol_cost;
    double min_sol_cost;
    int num_replan;
};

typedef std::pair<int, int> Location;

struct PathEntry {
    Location loc;
    orient o;
    int t;
    PathEntry(Location loc, orient o, int t): loc(std::move(loc)), o(o), t(t) {}
};

// <location, timestep>
typedef std::vector<PathEntry> Path;

// paths for all agents
typedef std::vector<Path> Paths;

struct speedProfile {
    enum type {
        SIPP_IP,
        Bezier
    };
};

struct Action{
    enum type{
        turnLeft,
        turnRight,
        turnBack,
        forward,
        none,
        wait,
        rotate
    } motion_type;
    Location start_loc;
    orient start_o;
    double start_t;
    Location goal_loc;
    orient goal_o;
    double goal_t;
    // Path local_path;
    // std::shared_ptr<speedProfile> speed_profile;

    Action(Action::type motion_type, Location start_loc, orient start_o, double start_t,
              Location goal_loc, orient goal_o, double goal_t,
              const std::shared_ptr<speedProfile>& speed_profile=nullptr):
              motion_type(motion_type), start_loc(std::move(start_loc)), start_o(start_o), start_t(start_t),
              goal_loc(std::move(goal_loc)), goal_o(goal_o), goal_t(goal_t)
    {}

};

typedef std::vector<std::shared_ptr<Action>> motionPlan;