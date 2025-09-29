#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    amp::DenseArray2D<int> distant_array = brush_fire_distance(problem);

    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(problem.q_goal);
    return path;
}

amp::DenseArray2D<int> MyGDAlgorithm::brush_fire_distance(const amp::Problem2D& problem){
    int x_cells_num = (problem.x_max-problem.x_min)*cells_per_unit;
    int y_cells_num = (problem.y_max-problem.y_min)*cells_per_unit;
    int num_of_cells = x_cells_num*y_cells_num;
    int changed_num_of_cells = 0;
    amp::DenseArray2D<int> distant_array(x_cells_num, y_cells_num);

    double cell_size = double(1.0)/cells_per_unit;
    
    // find obstacle
    for(int i = 0; i < x_cells_num; i++){
        for(int j = 0; j < y_cells_num; j++){
            if (check_collision_problem(problem, Eigen::Vector2d(cell_size*i+cell_size/2, cell_size*i+cell_size/2))){
                for(int k = -1; k < 2; k++){
                    for(int l = -1; l < 2; l++){
                        if((i+k<0) || j+l<0 || i+k>=x_cells_num || j+l>=y_cells_num){
                            continue;
                        }
                        if (distant_array(i+k, j+l) == 0){
                            distant_array(i+k, j+l) = 1;
                            changed_num_of_cells += 1;
                        }
                    }
                }
            }
        }
    }

    int current_distant = 2;
    while(changed_num_of_cells < num_of_cells){
        for(int i = 0; i < x_cells_num; i++){
            for(int j = 0; j < y_cells_num; j++){
                if(distant_array(i,j) == current_distant - 1){
                    for(int k = -1; k < 2; k++){
                        for(int l = -1; l < 2; l++){
                            if((i+k<0) || j+l<0 || i+k>=x_cells_num || j+l>=y_cells_num){
                                continue;
                            }
                            if (distant_array(i+k, j+l) == 0){
                                distant_array(i+k, j+l) = current_distant;
                                changed_num_of_cells += 1;
                            }
                        }
                    }

                }
            }
        }
        current_distant += 1;
    }

    return distant_array;
}
