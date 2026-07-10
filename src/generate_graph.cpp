#include "generate_graph.h"
#include "Timer.h"

bool same_locations(Location location1, Location location2) {
    int i1 = location1.first;
    int j1 = location1.second;
    int i2 = location2.first;
    int j2 = location2.second;

    return (i1 == i2 && j1 == j2);
}

orient getOrient(Location const prev_loc, Location const curr_loc)
{
    if (prev_loc == curr_loc) {
        return orient::None;
    } else if (prev_loc.first == curr_loc.first) {
        if (prev_loc.second - curr_loc.second == 1) {
            return orient::S;
        } else if (prev_loc.second - curr_loc.second == -1) {
            return orient::N;
        } else {
            std::cerr << "Invalid neighbor states!!!\n";
            exit(-1);
        }
    } else if (prev_loc.second == curr_loc.second) {
        if (prev_loc.first - curr_loc.first == 1) {
            return orient::W;
        } else if (prev_loc.first - curr_loc.first == -1) {
            return orient::E;
        } else {
            std::cerr << "Invalid neighbor states!!!\n";
            exit(-1);
        }
    } else {
        std::cerr << "Invalid neighbor states!!!\n";
        exit(-1);
    }
}

// Return path and stateCnt of an agent
tuple<Path, int> parse_path(string line) {
    int i, j, stateCnt = 0;
    int time = 0;
    size_t comma_pos, leftPar_pos, rightPar_pos;
    Path path;
    Location prev_location = make_pair(-1, -1);
    orient prev_o = orient::None;
    int prev_t = 0;

    while ((leftPar_pos = line.find("(")) != string::npos) {
        // Process an index pair
        comma_pos = line.find(",");
        i = stoi(line.substr(leftPar_pos + 1, comma_pos));
        rightPar_pos = line.find(")");
        j = stoi(line.substr(comma_pos + 1, rightPar_pos));
        line.erase(0, rightPar_pos + 1);

        // Create a location tuple and add it to the path
        Location location = make_pair(i, j);
        if (!same_locations(location, prev_location)) {
            stateCnt++;
            // First state, always assume as orient::S
            if (prev_o == orient::None) {
                path.emplace_back(location, orient::S, time);
                prev_o = orient::S;
            } else {
                orient curr_o = getOrient(prev_location, location);
                if (curr_o == orient::None) {
                    std::cerr << "same location, potential bugs!" << std::endl;
                    exit(-1);
                }
                if (curr_o != prev_o) {
                    path.emplace_back(prev_location, curr_o, prev_t);
                }
                path.emplace_back(location, curr_o, time);
                prev_o = curr_o;
            }
            // path.push_back(make_pair(location, time));
            prev_t = time;
            prev_location = location;
        }
        time++;
    }
    return make_tuple(path, stateCnt);
}


void printRawPath(Paths &paths) {
    int i = 0;
    for (auto &tmp_path: paths) {
        printf("For agent %d: ", i);
        int j = 0;
        for (auto &entry: tmp_path) {
            printf("(Loc: (%d, %d), orient: %d, time: %d, state id: %d);  ", entry.loc.first, entry.loc.second, entry.o, entry.t, j);
            j ++;
        }
        printf("\n\n");
        i++;
    }
}

// Return all paths, accumulated counts of states, and States
shared_ptr<Paths> parse_soln(const char *fileName) {
    auto paths_ptr = make_shared<Paths>();
    auto &paths = *paths_ptr;

    string fileName_string = fileName;
    ifstream file(fileName_string);
    if (file.is_open()) {
        string line;
        while (getline(file, line)) {
            // Sanity check that the line is a path
            if (line[0] == 'A') {
                tuple<Path, int> parse_result = parse_path(line);
                Path path = get<0>(parse_result);
                // Done with the agent
                paths.push_back(path);
            }
        }
        file.close();
    } else {
        std::cout << "exit\n";
        exit(0);
    }
#if DEBUG_GRAPH
    printRawPath(paths);
#endif
    return paths_ptr;
}


// duplication=true means allowing consecutive vertices in a path to be duplicate.
shared_ptr<Graph> construct_graph(const shared_ptr<Paths> &paths, bool duplication) {
    if (duplication) {
        std::cout << "not supported duplication now" << std::endl;
        exit(-1);
    }

    auto graph = make_shared<Graph>(paths);
    return graph;
}

shared_ptr<Graph> construct_graph(const char *fileName) {
    auto paths = parse_soln(fileName);
    return construct_graph(paths, false);
}

shared_ptr<Graph> construct_delayed_graph(const shared_ptr<Graph> &graph, const vector<int> &delay_steps,
                                          const vector<int> &states) {
    auto new_paths = make_shared<Paths>();

    int delay_sum = 0;

    for (int agent = 0; agent < graph->get_num_agents(); ++agent) {
        if (delay_steps[agent] == 0) {
            new_paths->push_back((*graph->paths)[agent]);
        } else {
            // this is a delayed agent
            Path &ori_path = (*graph->paths)[agent];
            Path new_path;
            int delayed_state = states[agent];

            for (int state = 0; state <= delayed_state; state++) {
                new_path.push_back(ori_path[state]);
            }
            // insert repeated current states for a multi-step delay.
            // <Location, timestep>
            // because now we still want to stick to the original plan.
            // pair<Location, int> repeat = make_pair(get<0>(new_path.back()), -1);
            PathEntry repeat(new_path.back().loc, new_path.back().o, -1);
            int delay = delay_steps[agent];
            delay_sum += delay;
            for (int state = 0; state < delay; state++) {
                new_path.push_back(repeat);
            }
            int ori_size = ori_path.size();
            for (int state = delayed_state + 1; state < ori_size; state++) {
                new_path.push_back(ori_path[state]);
            }
            new_paths->push_back(new_path);
        }
    }

    g_timer.record_p("make_graph_s");
    auto new_graph = make_shared<Graph>(new_paths, states);
    g_timer.record_d("make_graph_s", "make_graph");

    g_timer.print_all_d();

    return new_graph;
}
