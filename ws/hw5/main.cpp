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
    MyGDAlgorithm algo(1.0, 1.0, 50, 200.0, 40);
    Problem2D problem = HW2::getWorkspace2();
    Path2D path = algo.plan(problem);
    Visualizer::makeFigure(problem, path);

    // Problem2D problem;
    // Path2D path;
    // bool success = HW5::generateAndCheck(algo, path, problem);
    // Visualizer::makeFigure(problem, path);

    // HW5::getWorkspace1().obstacles[1].print();
    // MyPotentialFunction potential_function(1.0, 1.0, 50, 50.0, 40, HW5::getWorkspace1());
    // test brushfire
    // amp::DenseArray2D<int> distant_array = potential_function.brushFireAll();
    // LOG(distant_array(50,50));

    
    // Visualize your potential function
    // Visualizer::makeFigure(potential_function, HW5::getWorkspace1(), 50, false);
    // Visualizer::makeFigure(potential_function, HW5::getWorkspace1(), 100);
    // Visualizer::makeFigure(HW5::getWorkspace1());
    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    // HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 1.0, 1.0, 50, 50.0, 40);
    return 0;
}