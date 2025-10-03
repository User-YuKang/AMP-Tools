// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    // MyGDAlgorithm algo(1.0, 1.5, 10, 800.0, 40);
    MyGDAlgorithm algo(1.5, 2, 10, 800.0, 40);
    Problem2D problem = HW2::getWorkspace2();
    Path2D path = algo.plan(problem);
    Visualizer::makeFigure(problem, path);
    LOG(path.length());

    amp::Random2DEnvironmentSpecification spec;
    // for (int i = 1; i< 2; i ++){
    //     Problem2D problem = amp::EnvironmentTools::generateRandomPointAgentProblem(spec,1);
        // if (i == 87){
        //     for (int j = 0; j<problem.obstacles.size(); j++){
        //         std::string msg = "this is obstacles " + std::to_string(j);
        //         INFO(msg);
        //         problem.obstacles[j].print();
        //     }
        //     Visualizer::makeFigure(problem);
        //     Path2D path = algo.plan(problem);
        //     Visualizer::makeFigure(problem, path);
        // }
        // if (i<87){
        //     continue;
        // }
    //     Path2D path = algo.plan(problem);
    //     bool success = HW5::check(path, problem);
    //     Visualizer::makeFigure(problem, path);
    //     if (!success){
    //         LOG(i);
    //         Visualizer::makeFigure(problem, path);
    //     }
        
    // }

    // HW5::getWorkspace1().obstacles[1].print();
    MyPotentialFunction potential_function(1.5, 2, 10, 800.0, 40, HW5::getWorkspace1());

    
    // Visualize your potential function
    // Visualizer::makeFigure(potential_function, HW5::getWorkspace1(), 50, false);
    // Visualizer::makeFigure(potential_function, HW5::getWorkspace1(), 60);
    // Visualizer::makeFigure(HW5::getWorkspace1());
    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    // HW5::grade<MyGDAlgorithm>("yu.kong@colorado.edu", argc, argv, 1.5, 2, 10, 800.0, 40);
    return 0;
}