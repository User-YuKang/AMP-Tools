// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    // Initializing workspace 1 with 3 agents
    srand(time(NULL));
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentPath2D path;
    MultiAgentProblem2D problem = HW8::getWorkspace1(6);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner(0.5, 0.05, 7500);
    // bool isValid = false;
    // while (!isValid){
    //     collision_states.clear();
    //     path = central_planner.plan(problem);
    //     isValid = HW8::check(path, problem, collision_states);
    // }
    // Visualizer::makeFigure(problem, path, collision_states);
    // collision_states = {{}};
    // HW8::generateAndCheck(central_planner, path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);

    // std::list<std::vector<double>> times;
    // std::list<std::vector<double>> num_nodes;
    // std::vector<double> average_times;
    // std::vector<double> average_tree;
    // for(int i = 1; i < 2; i++){
    //     LOG("Benchmarking for " << i+1 << " agent");
    //     problem = HW8::getWorkspace1(i+1);
    //     std::vector<double> this_times;
    //     std::vector<double> this_nodes;
    //     double total_time = 0;
    //     double total_node = 0;
    //     for(int j = 0; j < 100; j++){
    //         auto start = std::chrono::high_resolution_clock::now();
    //         path = central_planner.plan(problem);
    //         auto end = std::chrono::high_resolution_clock::now();
    //         auto duration = end - start;
    //         auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
    //         this_times.push_back(seconds*1000);
    //         this_nodes.push_back(central_planner.getNodes().size());
    //         total_time += seconds*1000;
    //         total_node += central_planner.getNodes().size();
    //     }
    //     times.push_back(this_times);
    //     num_nodes.push_back(this_nodes);
    //     average_times.push_back(total_time/100);
    //     average_tree.push_back(total_node/100);
    // }
    // std::vector<std::string> labels = {"1 agent", "2 agents", "3 agents", "4 agents", "5 agents", "6 agents"};
    // std::vector<std::string> labels = {"1 agent", "2 agents"};
    // std::vector<std::string> labels = {"2 agents"};
    // amp::Visualizer::makeBoxPlot(times, labels, "Time Required for Centralize", "Number of Agents", "time (ms)");
    // amp::Visualizer::makeBoxPlot(num_nodes, labels, "Tree size for Centralize", "Number of Agents", "Number of Nodes");
    // amp::Visualizer::makeBarGraph(average_times, labels , "Average Time Required for Centralize", "Number of Agents", "time (ms)");   
    // amp::Visualizer::makeBarGraph(average_tree, labels, "Average Tree size for Centralize", "Number of Agents", "Number of Nodes");    

    // std::ostringstream oss;
    // oss << "centralize" << ".txt";
    // std::string file_name = oss.str();

    // std::ofstream MyFile(file_name);

    // MyFile << "time: " << std::endl;
    // for (int i = 0; i < average_times.size(); i++){
    //     MyFile << average_times[i] << std::endl;
    //     LOG(average_times[i]);
    // }
    // MyFile << "tree_size: " << std::endl;
    // for (int i = 0; i < average_tree.size(); i++){
    //     MyFile << average_tree[i] << std::endl;
    // }
    // MyFile.close();

    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner(0.5, 0.05, 7500);
    // collision_states = {{}};
    // HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    // path = decentral_planner.plan(problem);
    // HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);

    std::list<std::vector<double>> times;
    std::vector<double> average_times;
    for(int i = 1; i < 2; i++){
        LOG("Benchmarking for " << i+1 << " agent");
        problem = HW8::getWorkspace1(i+1);
        std::vector<double> this_times;
        double total_time = 0;
        for(int j = 0; j < 100; j++){
            if(i == 5){
                LOG(j);
            }
            auto start = std::chrono::high_resolution_clock::now();
            path = decentral_planner.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = end - start;
            auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
            this_times.push_back(seconds*1000);
            total_time += seconds*1000;
        }
        times.push_back(this_times);
        average_times.push_back(total_time/100);
    }
    // std::vector<std::string> labels = {"1 agent", "2 agents", "3 agents", "4 agents", "5 agents", "6 agents"};
    // std::vector<std::string> labels = {"1 agent", "2 agents", "3 agents", "4 agents"};
    std::vector<std::string> labels = {"2 agents"};
    amp::Visualizer::makeBoxPlot(times, labels, "Time Required for Decouple", "Number of Agents", "time (ms)");
    // amp::Visualizer::makeBarGraph(average_times, labels , "Average Time Required for Decouple", "Number of Agents", "time (ms)");

    // std::ostringstream oss;
    // oss << "decentralize" << ".txt";
    // std::string file_name = oss.str();

    // std::ofstream MyFile(file_name);

    // MyFile << "time: " << std::endl;
    // for (int i = 0; i < average_times.size(); i++){
    //     MyFile << average_times[i] << std::endl;
    //     LOG(average_times[i]);
    // }
    // MyFile.close();


    // Visualize and grade methods
    Visualizer::saveFigures(true, "hw8_figs");
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("yu.kong@colorado.edu", argc, argv, std::make_tuple(0.5, 0.05, 7500), std::make_tuple(0.5, 0.05, 7500));
    return 0;
}