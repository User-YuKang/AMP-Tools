// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <random>
#include <time.h>

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

    std::uniform_real_distribution<double> length(0,2);
    std::uniform_real_distribution<double> angle(0,2*M_PI);
    std::default_random_engine re;
    // re.seed(time(NULL));
    

    // for (int i = 0; i< 1; i++){
    //     std::vector<double> links = std::vector<double>{length(re), length(re), length(re), length(re)};
    //     Eigen::Vector4d state = Eigen::Vector4d(angle(re), angle(re), angle(re), angle(re));
    //     MyManipulator2D manipulator(links);

    //     Eigen::Vector2d end_point = manipulator.getJointLocation(state, 4);

    //     Eigen::Vector4d result_state = manipulator.getConfigurationFromIK(end_point);
    //     Eigen::Vector2d test_end_point = manipulator.getJointLocation(result_state, 4);

    //     if ((test_end_point - end_point).norm() > pow(10,-6)){
    //         LOG(i);
    //     }
    //     LOG(end_point);
    //     LOG(test_end_point);
    //     Visualizer::makeFigure(manipulator, state);
    //     Visualizer::makeFigure(manipulator, result_state);
    // }


    // MyManipulator2D manipulator_2(std::vector<double>{1.69235, 1.87844, 1.89511}); 
    // Eigen::Vector2d end_effector = Eigen::Vector2d(0.66904, -1.56798);
    // const Eigen::Vector3d result_state = manipulator_2.getConfigurationFromIK(end_effector);
    // Visualizer::makeFigure(manipulator_2, result_state); 

    // Eigen::Vector2d end_effector_check = manipulator_2.getJointLocation(result_state, 3);
    // LOG(end_effector_check);

    // Create the collision space constructor
    std::size_t n_cells = 200;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    MyManipulator2D manipulator(std::vector<double>{1.0, 1.0});
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // You can visualize your cspace 
    // Visualizer::makeFigure(*cspace);

    

    std::unique_ptr<MyGridCSpace2D> ws_ptr = std::make_unique<MyGridCSpace2D>(n_cells, n_cells, -3, 3, -3, 3);
    MyGridCSpace2D& ws = *ws_ptr;
    for(int i = 1; i < n_cells; i++){
        for(int j = 1; j < n_cells; j++){
            ws(i, j) = cspace_constructor.check_collision(HW4::getEx3Workspace3(), Eigen::Vector2d(-3+6*double(i)/n_cells, -3+6*double(j)/n_cells));
        }
    }
    // Visualizer::makeFigure(ws);
    // Visualizer::saveFigures(true, "hw4_figs");

    // HW4::checkCSpace(*cspace, manipulator, HW4::getEx3Workspace3(), 500, true);

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "yu.kong@colorado.edu", argc, argv);
    return 0;
}