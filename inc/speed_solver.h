#pragma once

#include<graph.h>
#include <boost/math/distributions/normal.hpp>

#include "sipp_ip.h"

#define DEBUG_SPEED_SOLVER 0

class speedSolver{
public:
    explicit speedSolver(int num_of_agent, std::shared_ptr<Graph> graph, std::shared_ptr<simEnv> sim);
    bool solve(std::vector<std::vector<action>>& exec_actions);
    void saveResults(bool plan_succeed, const string &fileName, const string &instanceName, runtimeStats& runtime_stats)
    {
        std::ifstream infile(fileName);
        bool exist = infile.good();
        infile.close();
        if (!exist)
        {
            ofstream addHeads(fileName);
            addHeads << "success,runtime,solution cost,min-solution cost," <<
                     "#type-2 edges,#type-1 edges,#Nodes," \
                     "#low-level calls,#replan," <<
                     "runtime of low-level planning," <<
                     "runtime of topo-logical sort," <<
                     "runtime of simulation," <<
                     "planning horizon,execution horizon," <<
                     "delay type,Gaussian variance,Gaussian P Thresh,"
                     "solver name,instance name,number of agent" << endl;
            addHeads.close();
        }
        ofstream stats(fileName, std::ios::app);
        stats << static_cast<int>(plan_succeed) << "," << runtime_stats.runtime << "," << runtime_stats.sol_cost << "," << runtime_stats.min_sol_cost << "," <<
        graph_->getType2CNT() << "," << graph_->getType1CNT() << "," << graph_->get_num_states(-1) << "," <<
        singleAgentSolver.num_low_level_calls << "," << runtime_stats.num_replan << "," <<
        ll_runtime << "," << topo_runtime << "," << runtime_stats.runtime_sim << "," <<
        sim_ptr->planning_horizon << "," << sim_ptr->execution_horizon << "," <<
        sim_ptr->delay_type_ << "," << sim_ptr->gaussian_var << "," << sim_ptr->gaussian_thresh << "," <<
        "speed_solver" << "," <<  instanceName << "," << num_agent_ << endl;
        stats.close();
    }
private:
    double computeGaussianDelay(double dis1, double dis2);
    // STN delay model removed; only Gaussian delay supported
    void insertEdge2Table(const std::vector<std::shared_ptr<topoEdge>>& resolved_edges, const std::deque<std::pair<double, double>>& plan);
    void noiseEstimation(std::vector<std::deque<action>>& enque_actions);
public:
    double ll_runtime = 0.0;
    double topo_runtime = 0.0;

private:
    SIPP_IP singleAgentSolver;
    // temporal constraint table
    std::shared_ptr<constraintTable> TCT_ptr;
    std::shared_ptr<simEnv> sim_ptr;
    std::shared_ptr<topoGraph> topo_graph;
    std::shared_ptr<Graph> graph_;
    std::vector<estimateState> curr_loc;
    std::vector<simState> last_seen_locations;
    std::vector<double> acc_last_seen_locations_time;

    int num_agent_;
};