#pragma once

#include <vector>
#include "define.h"



struct Agent
{
    int id = -1;
    int start_location = -1;
    orient start_o = orient::S;
    int goal_location = -1;
    orient end_o = orient::S;

    double v_min = V_MIN;
    double v_max = V_MAX;
    double a_min = -A_MAX;
    double a_max = A_MAX;
    double length = 0.4; // radius of the robot
    double rotation_cost = ROTATE_COST;
    double turn_back_cost = TURN_BACK_COST;

    /*For time window*/
    bool is_solved = false;
    double earliest_start_time = 0.0;
    double init_velocity = 0.0;
    std::vector<int> trajectory;

    Agent() = default;
    Agent(int agent_id, int start_loc)
    {
        start_location = start_loc;
        id = agent_id;
    }

    /*statistic for result*/
    double total_cost = 0;
    double tmp_cost = 0;
};
