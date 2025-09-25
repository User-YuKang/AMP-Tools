// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    {
    // MyManipulator2D manipulator_1(std::vector<double>{1.0, 1.0}); 

    // // You can visualize your manipulator given an angle state like so:
    // amp::ManipulatorState test_state;
    // test_state.resize(2);
    // test_state = Eigen::Vector2d(M_PI, M_PI);
    // Eigen::Vector2d end_effector= manipulator_1.getJointLocation(test_state, 2);
    // LOG(end_effector);

    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    // Visualizer::makeFigure(manipulator_1, test_state); 

    // MyManipulator2D manipulator_2(std::vector<double>{0.5, 1.0, 0.5}); 
    // Eigen::Vector2d end_effector = Eigen::Vector2d(2.0, 0.0);
    // const Eigen::Vector3d result_state = manipulator_2.getConfigurationFromIK(end_effector);
    // Visualizer::makeFigure(manipulator_2, result_state); 

    // Eigen::Vector2d end_effector_check = manipulator_2.getJointLocation(result_state, 3);
    // LOG(end_effector_check);
}

    // Create the collision space constructor
    std::size_t n_cells = 1000;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    MyManipulator2D manipulator(std::vector<double>{1.0, 1.0});
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::saveFigures(true, "hw4_figs");

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}