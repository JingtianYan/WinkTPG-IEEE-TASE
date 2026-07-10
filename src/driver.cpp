#include <fstream>
#include <boost/program_options.hpp>
//#include "Algorithm/Astar.h"
#include <boost/filesystem.hpp>
// #include <vector>
#include "nlohmann/json.hpp"
#include "generate_graph.h"
//#include "simulation/simulator.h"
// #include "topo_sort.h"
#include "speed_solver.h"
// #include "sipp_ip.h"
#include "tpg_execute.h"

using json = nlohmann::json;


double getMinCost(std::shared_ptr<Graph>& graph) {
    SIPP_IP singleAgentSolver;
    double sum_sols = 0.0;
    for (auto& path: (*graph->paths)) {
        std::vector<temporalConstraint> empty_temporal;
        std::vector<action> empty_plan;
        singleAgentSolver.run(path, empty_temporal, empty_plan, 0, 0.0);
        sum_sols += empty_plan.back().start_t + empty_plan.back().cost;
    }
    return sum_sols;
}

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    po::options_description desc("Switch ADG Optimization");
    desc.add_options()
            ("help", "show help message")
            ("method,m", po::value<int>()->default_value(0), "the lower bound of delay steps")
            ("path_fp,p", po::value<std::string>()->required(), "path file to construct TPG")
            ("stats_file,o", po::value<std::string>()->required(), "output folder to store statistics files")
            ("planning_horizon", po::value<double>()->default_value(20.0), "size of planning horizon")
            ("execution_horizon", po::value<int>()->default_value(10), "size of execution horizon")
            ("delay_type,d", po::value<int>()->default_value(0), "0 for no delay, 1 for gaussian delay")
            ("delay_steps", po::value<double>()->default_value(1.0), "(unused) maximum delay steps for legacy K-step model")
            ("delay_p_thresh", po::value<double>()->default_value(0.05), "gaussian delay probability")
            ("delay_var", po::value<double>()->default_value(0.01), "delay probability*1000. need to be a double")
            ("delay_steps_low", po::value<double>()->default_value(-0.01), "the lowerbound of delay steps")
            ("delay_steps_high", po::value<double>()->default_value(0.01), "the upperbound of delay steps");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    auto path_fp = vm.at("path_fp").as<string>();
    auto stats_file = vm.at("stats_file").as<string>();
    auto planning_horizon_size = vm.at("planning_horizon").as<double>();
    auto execution_horizon_size = vm.at("execution_horizon").as<int>();
    auto delay_type = vm.at("delay_type").as<int>();

    auto delay_var = vm.at("delay_var").as<double>();
    auto delay_thresh = vm.at("delay_p_thresh").as<double>();
    auto delay_steps_low = vm.at("delay_steps_low").as<double>();
    auto delay_steps_high = vm.at("delay_steps_high").as<double>();



    auto graph = construct_graph(path_fp.c_str());
    std::vector<std::pair<Location, orient>> init_locs;
    std::vector<std::pair<Location, orient>> goal_locs;

    for (auto& path: (*graph->paths)) {
        init_locs.emplace_back(path.front().loc, path.front().o);
        goal_locs.emplace_back(path.back().loc, path.back().o);
    }
    int planner = vm["method"].as<int>();

    std::shared_ptr<simEnv> sim_ptr = std::make_shared<simEnv>(graph->get_num_agents(),
                                                               init_locs,
                                                               goal_locs,
                                                               planning_horizon_size,
                                                               execution_horizon_size,
                                                               planner,
                                                               delay_type,
                                                               delay_thresh,
                                                               delay_var);

    /***********************************************Test Code For SIPP-IP***********************************************/
    // printf("add constraints\n");
    // std::vector<temporalConstraint> empty_constraints;
    // temporalConstraint cons_1;
    // cons_1.goal_path_idx = 26;
    // cons_1.interval = std::make_pair(3.0, 4000.0);
    // empty_constraints.push_back(cons_1);
    // temporalConstraint cons_2;
    // cons_2.goal_path_idx = 26;
    // cons_2.interval = std::make_pair(3500.0, 8000.0);
    // empty_constraints.push_back(cons_2);
    // SIPP_IP solver;
    // std::vector<action> test_exec_plan;
    // solver.run((*graph->paths)[0], empty_constraints, test_exec_plan, 0, 0, false);
    /***********************************************Test Code For SIPP-IP***********************************************/
    double sim_runtime = 0.0;
    /***********************************************Test Code For TPG***********************************************/
    bool get_plan_succeed = true;
    if (planner == 0) {
        auto start = std::chrono::high_resolution_clock::now();
        std::shared_ptr<TPGExecutor> tpg_executor = std::make_shared<TPGExecutor>(graph->get_num_agents(), graph, sim_ptr);
        double dt = 1.0;
        while (not sim_ptr->isAllFinished()) {
            std::vector<std::vector<action>> new_enque_actions;
            get_plan_succeed = tpg_executor->getNextActions(new_enque_actions);
            if (not get_plan_succeed) {
                break;
            }
            auto sim_start = std::chrono::high_resolution_clock::now();
            sim_ptr->enqueActions(new_enque_actions);
            sim_ptr->updateSingleRound(dt);
            auto sim_stop = std::chrono::high_resolution_clock::now();
            auto sim_duration = std::chrono::duration_cast<std::chrono::microseconds>(sim_stop - sim_start);
            auto sim_duration_seconds = std::chrono::duration<double>(sim_duration).count();
            sim_runtime += sim_duration_seconds;
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        auto duration_micro = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        auto duration_seconds = std::chrono::duration<double>(duration_micro).count();

        runtimeStats runtime_stats;
        runtime_stats.sol_cost = sim_ptr->getRuntime();
        runtime_stats.runtime = duration_seconds;
        runtime_stats.runtime_sim = sim_runtime;
        runtime_stats.num_replan = 0;
        runtime_stats.min_sol_cost = getMinCost(graph);
        tpg_executor->saveResults(get_plan_succeed, stats_file, path_fp, runtime_stats);
    }
    /***********************************************Test Code For TPG***********************************************/

    /***********************************************Test Code For ours***********************************************/
    else if (planner == 1) {
        speedSolver profile_solver(graph->get_num_agents(), graph, sim_ptr);
        auto start = std::chrono::high_resolution_clock::now();
        bool sim_finished = false;
        int debug_count = 0;
        while (not sim_finished) {
            std::vector<std::vector<action>> exec_actions;
            get_plan_succeed = profile_solver.solve(exec_actions);
            if (not get_plan_succeed) {
                printf("find failed case\n");
                std::string combinedArgs;

                for (int i = 0; i < argc; i++) {
                    combinedArgs += argv[i];  // Add each argument to the string
                    if (i < argc - 1) {
                        combinedArgs += " ";  // Add a space between arguments
                    }
                }

                std::cerr << "All arguments combined: " << combinedArgs << std::endl;
                break;
            }
            auto sim_start = std::chrono::high_resolution_clock::now();
            sim_ptr->enqueActions(exec_actions);
            sim_ptr->updateSingleRound();
            sim_finished = sim_ptr->isAllFinished();
            auto sim_stop = std::chrono::high_resolution_clock::now();
            auto sim_duration = std::chrono::duration_cast<std::chrono::microseconds>(sim_stop - sim_start);
            auto sim_duration_seconds = std::chrono::duration<double>(sim_duration).count();
            sim_runtime += sim_duration_seconds;
            debug_count++;
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        auto duration_micro = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        auto duration_seconds = std::chrono::duration<double>(duration_micro).count();

        runtimeStats runtime_stats;
        runtime_stats.sol_cost = sim_ptr->getRuntime();
        runtime_stats.runtime = duration_seconds;
        runtime_stats.runtime_sim = sim_runtime;
        runtime_stats.num_replan = debug_count;
        runtime_stats.min_sol_cost = getMinCost(graph);
        profile_solver.saveResults(get_plan_succeed, stats_file, path_fp, runtime_stats);
    }
    /***********************************************Test Code For ours***********************************************/
    return 0;
}
