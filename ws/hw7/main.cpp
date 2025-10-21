// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <fstream> 
#include <sstream>


using namespace amp;

int main(int argc, char** argv) {
    RNG::seed(amp::RNG::randiUnbounded());
    srand(time(NULL));
    // HW7::hint(); // Consider implementing an N-dimensional planner 

    // Example of creating a graph and adding nodes for visualization
    // std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    // std::map<amp::Node, Eigen::Vector2d> nodes;
    
    // std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
    // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    // std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    // graphPtr->print();

    // Test PRM on Workspace1 of HW2
    // Problem2D problem = HW2::getWorkspace2();
    // problem.y_max = 3;
    // problem.y_min = -3;
    // MyPRM prm(200, 0, 2, false);
    // amp::Path2D path_prm = prm.plan(problem);
    // std::shared_ptr<amp::Graph<double>> prm_graphPtr = prm.getGraphPtr();
    // std::map<amp::Node, Eigen::Vector2d> prm_nodes = prm.getNodes2D();
    // // Visualizer::makeFigure(problem, *prm_graphPtr, prm_nodes);
    // Visualizer::makeFigure(problem, path_prm, *prm_graphPtr, prm_nodes);

    // Problem2D problem = HW2::getWorkspace1();
    // problem.y_max = 3;
    // problem.y_min = -3;
    
    // int num_valid = 0;
    // std::vector<double> path_length;
    // std::vector<double> time;

    // LOG("WS 1 smooth");
    // std::vector<int> points = {200, 500, 1000};
    // for (int idx = 0; idx < points.size(); idx++)
    // {
    //     for(double radius = 1; radius <= 2; radius += 1)
    //     {
    //         LOG("working for " << points[idx] << "_" << radius);
    //         path_length.clear();
    //         num_valid = 0;
    //         time.clear();
    //         MyPRM prm(points[idx], 0, radius, true);
    //         for(int i = 0; i < 100; i++){
    //             bool success = false;

    //             auto start = std::chrono::high_resolution_clock::now();
    //             Path2D path_prm = prm.plan(problem);

    //             success = HW7::check(path_prm, problem, false);

    //             auto end = std::chrono::high_resolution_clock::now();
    //             auto duration = end - start;
    //             auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();

    //             time.push_back(seconds);

    //             if(success)
    //             {
    //                 path_length.push_back(path_prm.length());
    //                 num_valid++;
    //             }
    //         }

    //         std::ostringstream oss;
    //         oss << "ws1_s_" << points[idx] << "_" << radius << ".txt";
    //         std::string file_name = oss.str();

    //         std::ofstream MyFile(file_name);

    //         MyFile << "path length: " << std::endl;
    //         for (int i = 0; i < path_length.size(); i++){
    //             MyFile << path_length[i] << std::endl;
    //         }
    //         MyFile << "time: " << std::endl;
    //         for (int i = 0; i < time.size(); i++){
    //             MyFile << time[i] << std::endl;
    //         }

    //         MyFile.close();
    //     }
    // }

    // LOG("WS 2");
    // problem = HW2::getWorkspace2();
    // // std::vector<int> points = {200, 500, 1000};
    // for (int idx = 0; idx < points.size(); idx++)
    // {
    //     for(double radius = 1; radius <= 2; radius += 1)
    //     {
    //         LOG("working for " << points[idx] << "_" << radius);
    //         path_length.clear();
    //         num_valid = 0;
    //         time.clear();
    //         MyPRM prm(points[idx], 0, radius, false);
    //         for(int i = 0; i < 100; i++){
    //             bool success = false;

    //             auto start = std::chrono::high_resolution_clock::now();
    //             Path2D path_prm = prm.plan(problem);

    //             success = HW7::check(path_prm, problem, false);

    //             auto end = std::chrono::high_resolution_clock::now();
    //             auto duration = end - start;
    //             auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();

    //             time.push_back(seconds);

    //             if(success)
    //             {
    //                 path_length.push_back(path_prm.length());
    //                 num_valid++;
    //             }
    //         }

    //         std::ostringstream oss;
    //         oss << "ws2_" << points[idx] << "_" << radius << ".txt";
    //         std::string file_name = oss.str();

    //         std::ofstream MyFile(file_name);

    //         MyFile << "path length: " << std::endl;
    //         for (int i = 0; i < path_length.size(); i++){
    //             MyFile << path_length[i] << std::endl;
    //         }
    //         MyFile << "time: " << std::endl;
    //         for (int i = 0; i < time.size(); i++){
    //             MyFile << time[i] << std::endl;
    //         }

    //         MyFile.close();
    //     }
    // }


    // LOG("WS 2 smooth");
    // // std::vector<int> points = {200, 500, 1000};
    // for (int idx = 0; idx < points.size(); idx++)
    // {
    //     for(double radius = 1; radius <= 2; radius += 1)
    //     {
    //         LOG("working for " << points[idx] << "_" << radius);
    //         path_length.clear();
    //         num_valid = 0;
    //         time.clear();
    //         MyPRM prm(points[idx], 0, radius, true);
    //         for(int i = 0; i < 100; i++){
    //             bool success = false;

    //             auto start = std::chrono::high_resolution_clock::now();
    //             Path2D path_prm = prm.plan(problem);

    //             success = HW7::check(path_prm, problem, false);

    //             auto end = std::chrono::high_resolution_clock::now();
    //             auto duration = end - start;
    //             auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();

    //             time.push_back(seconds);

    //             if(success)
    //             {
    //                 path_length.push_back(path_prm.length());
    //                 num_valid++;
    //             }
    //         }

    //         std::ostringstream oss;
    //         oss << "ws2_s_" << points[idx] << "_" << radius << ".txt";
    //         std::string file_name = oss.str();

    //         std::ofstream MyFile(file_name);

    //         MyFile << "path length: " << std::endl;
    //         for (int i = 0; i < path_length.size(); i++){
    //             MyFile << path_length[i] << std::endl;
    //         }
    //         MyFile << "time: " << std::endl;
    //         for (int i = 0; i < time.size(); i++){
    //             MyFile << time[i] << std::endl;
    //         }

    //         MyFile.close();
    //     }
    // }

    Problem2D problem = HW5::getWorkspace1();
    problem.y_max = 3;
    problem.y_min = -3;
    
    int num_valid = 0;
    std::list<std::vector<double>> path_length_list;
    std::list<std::vector<double>> time_list;
    std::vector<double> valid_list;
    std::vector<double> path_length;
    std::vector<double> time;

    std::vector<Problem2D> problems = {problem, HW2::getWorkspace1(), HW2::getWorkspace2()};
    for(int ws_idx = 0; ws_idx < problems.size(); ws_idx++)
    {
        // LOG("working for " << num_it[idx] << "_" << radius);
        path_length.clear();
        num_valid = 0;
        time.clear();
        MyRRT rrt(0.05, 5000, 0.5);
        for(int i = 0; i < 1; i++){
            bool success = false;

            auto start = std::chrono::high_resolution_clock::now();
            Path2D path_rrt = rrt.plan(problems[ws_idx]);

            success = HW7::check(path_rrt, problems[ws_idx], false);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = end - start;
            auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
            std::shared_ptr<amp::Graph<double>> rrt_graphPtr = rrt.getGraphPtr();
            std::map<amp::Node, Eigen::Vector2d> rrt_nodes = rrt.getNodes2D();
            Visualizer::makeFigure(problems[ws_idx], path_rrt, *rrt_graphPtr, rrt_nodes);

            time.push_back(seconds);

            if(success)
            {
                path_length.push_back(path_rrt.length());
                num_valid++;
            }
        }
        valid_list.push_back(num_valid);
        path_length_list.push_back(path_length);
        time_list.push_back(time);

        // std::ostringstream oss;
        // oss << "ws1_s_" << points[idx] << "_" << radius << ".txt";
        // std::string file_name = oss.str();

        // std::ofstream MyFile(file_name);

        // MyFile << "path length: " << std::endl;
        // for (int i = 0; i < path_length.size(); i++){
        //     MyFile << path_length[i] << std::endl;
        // }
        // MyFile << "time: " << std::endl;
        // for (int i = 0; i < time.size(); i++){
        //     MyFile << time[i] << std::endl;
        // }

        // MyFile.close();
    }
    // std::vector<std::string> labels = {"HW5 WS1", "HW2 WS1", "HW2 WS2"};
    // Visualizer::makeBoxPlot(time_list, labels, "Time", "Workspace", "time(s)");
    // Visualizer::makeBoxPlot(path_length_list, labels, "Path Length", "Workspace", "Path Length");
    // Visualizer::makeBarGraph(valid_list, labels, "Number of Valid Path", "Wrokspace", "Number of Valid Path");

    // Generate a random problem and test RRT
    // problem = HW2::getWorkspace2();
    // MyRRT rrt(0.05, 3000, 0.3);
    // Path2D rrt_path;
    // HW7::generateAndCheck(rrt, rrt_path, problem);
    // amp::Path2D rrt_path = rrt.plan(problem);
    // std::shared_ptr<amp::Graph<double>> rrt_graphPtr = rrt.getGraphPtr();
    // std::map<amp::Node, Eigen::Vector2d> rrt_nodes = rrt.getNodes2D();
    // rrt_graphPtr->print();
    // Visualizer::makeFigure(problem, *rrt_graphPtr, rrt_nodes);
    // Visualizer::makeFigure(problem, rrt_path, *rrt_graphPtr, rrt_nodes);
    Visualizer::saveFigures(true, "hw7_figs");

    // Grade method
    // HW7::grade<MyPRM, MyRRT>("yu.kong@colorado.edu", argc, argv, std::make_tuple(500, 10), std::make_tuple(0.05, 3000, 0.3));
    return 0;
}