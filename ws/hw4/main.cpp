// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <random>
#include <time.h>
#include <algorithm> 

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
    // std::vector<amp::Polygon> cs_obstacles;
    // std::vector<double> height;

    // std::vector<Eigen::Vector2d> obstacle_ccw = std::vector<Eigen::Vector2d>{Eigen::Vector2d(0, 0),Eigen::Vector2d(1, 2),Eigen::Vector2d(0,2)};
    // std::vector<double> obstacle_angle;
    // for(int i = 0; i < 3; i++){
    //     Eigen::Vector2d vec;
    //     if(i == 2){
    //         vec = obstacle_ccw[0] - obstacle_ccw[2];
    //     }
    //     else{
    //         vec = obstacle_ccw[i+1] - obstacle_ccw[i];
    //     }
    //     double temp = atan2(vec[1],vec[0]);
    //     if (temp < 0){temp += 2*M_PI;}
    //     obstacle_angle.push_back(temp);
    // }
    
    // std::vector<Eigen::Vector2d> robot_ccw = std::vector<Eigen::Vector2d>{Eigen::Vector2d(-1, -2),Eigen::Vector2d(0, -2),Eigen::Vector2d(0,0)};
    // std::vector<double> robot_angle;
    // for(int i = 0; i < 3; i++){
    //     Eigen::Vector2d vec;
    //     if(i == 2){
    //         vec = robot_ccw[0] - robot_ccw[2];
    //     }
    //     else{
    //         vec = robot_ccw[i+1] - robot_ccw[i];
    //     }
    //     double temp = atan2(vec[1],vec[0]);
    //     if (temp < 0){temp += 2*M_PI;}
    //     robot_angle.push_back(temp);
    // }
    // double n = 12;
    // double step_angle = 2*M_PI/n;
    // Eigen::Rotation2Dd rot = Eigen::Rotation2Dd(step_angle);
    // for(int i = 0; i < 12; i++){
    //     LOG(i);
    //     height.push_back(step_angle*i);
    //     std::vector<Eigen::Vector2d> cs_obstacle_ccw;
    //     cs_obstacle_ccw.push_back(obstacle_ccw[0]+robot_ccw[0]);

    //     int j = 0;
    //     int k = 0;
    //     while(j<3 && k <3){
    //         if(obstacle_angle[j] < robot_angle[k]){j += 1;}
    //         else if(obstacle_angle[j] > robot_angle[k]){k += 1;}
    //         else{
    //             j += 1;
    //             k += 1;
    //         }
    //         int temp_j = j;
    //         int temp_k = k;
    //         if(j == 3){
    //             temp_j = 0;
    //         }
    //         if(k == 3){
    //             temp_k = 0;
    //         }
    //         cs_obstacle_ccw.push_back(obstacle_ccw[temp_j] + robot_ccw[temp_k]);
    //     }
    //     cs_obstacles.push_back(cs_obstacle_ccw);

    //     for (int l = 0; l< 3; l++){
    //         robot_angle[l] = robot_angle[l] + step_angle;
    //         LOG(robot_angle[l]);
    //         if(robot_angle[l] >= 2*M_PI){robot_angle[l] = robot_angle[l]-2*M_PI;}
    //         robot_ccw[l] = rot*robot_ccw[l];
    //     }

    //     auto min_it = std::min_element(robot_angle.begin(), robot_angle.end());
    //     double current_min_angle = *min_it;
    //     while (robot_angle[0] != current_min_angle){
    //         double last_angle = robot_angle[2];
    //         Eigen::Vector2d last_vertic = robot_ccw[2];

    //         for(int l = 2; l > 0; l--){
    //             robot_angle[l] = robot_angle[l-1];
    //             robot_ccw[l] = robot_ccw[l-1];
    //         }
    //         robot_angle[0] = last_angle;
    //         robot_ccw[0] = last_vertic;
    //     }
    // }

    // Visualizer::makeFigure(cs_obstacles, height);
    // Visualizer::saveFigures(true, "hw4_figs");



    // std::uniform_real_distribution<double> length(0,2);
    // std::uniform_real_distribution<double> angle(0,2*M_PI);
    // std::default_random_engine re;
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


    // MyManipulator2D manipulator_2(std::vector<double>{1, 0.5, 1}); 
    // Eigen::Vector2d end_effector = Eigen::Vector2d(2, 0);
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
    Visualizer::makeFigure(*cspace);

    

    std::unique_ptr<MyGridCSpace2D> ws_ptr = std::make_unique<MyGridCSpace2D>(n_cells, n_cells, -3, 3, -3, 3);
    MyGridCSpace2D& ws = *ws_ptr;
    for(int i = 1; i < n_cells; i++){
        for(int j = 1; j < n_cells; j++){
            ws(i, j) = cspace_constructor.check_collision(HW4::getEx3Workspace3(), Eigen::Vector2d(-3+6*double(i)/n_cells, -3+6*double(j)/n_cells));
        }
    }
    Visualizer::makeFigure(ws);
    Visualizer::saveFigures(true, "hw4_figs");

    // HW4::checkCSpace(*cspace, manipulator, HW4::getEx3Workspace3(), 500, true);

    // Grade method
    // amp::HW4::grade<MyManipulator2D>(cspace_constructor, "yu.kong@colorado.edu", argc, argv);
    return 0;
}