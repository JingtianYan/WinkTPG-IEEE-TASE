#pragma once
#include <utility>

#include "topo_sort.h"
#include "graph.h"
#include "sipp_ip.h"

// #define DEBUG_TPG_EXECUTE

class TPGExecutor {
public:
    TPGExecutor(size_t num_agent, std::shared_ptr<Graph> graph, std::shared_ptr<simEnv> sim_ptr);
    bool getNextActions(std::vector<std::vector<action>>& new_actions);
    void saveResults(bool get_plan_succeed, const string &fileName, const string &instanceName, runtimeStats& runtime_stats)
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
          stats << static_cast<int>(get_plan_succeed) << "," << runtime_stats.runtime << "," << runtime_stats.sol_cost << "," << runtime_stats.min_sol_cost << "," <<
              graph_->getType2CNT() << "," << graph_->getType1CNT() << "," << graph_->get_num_states(-1) << "," <<
              singleAgentSolver.num_low_level_calls << "," << runtime_stats.num_replan << "," <<
              ll_runtime << "," << 0.0 << "," << runtime_stats.runtime_sim << "," <<
              sim_ptr_->planning_horizon << "," << sim_ptr_->execution_horizon << "," <<
              sim_ptr_->delay_type_ << "," << sim_ptr_->gaussian_var << "," << sim_ptr_->gaussian_thresh << "," <<
              "speed_solver" << "," <<  instanceName << "," << num_agent_ << endl;
        stats.close();
    }
private:
    void updateTPG();

    std::shared_ptr<topoGraph> topo_ptr;
    std::shared_ptr<simEnv> sim_ptr_;
    size_t num_agent_;
    std::shared_ptr<Graph> graph_;
    SIPP_IP singleAgentSolver;
    double ll_runtime = 0.0;

public:
    std::vector<int> last_enque_idx; // indicate the idx before which is enque (include)
    std::vector<int> last_finish_idx; // indicate the idx that has finished (include)
};

