#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d current_location;
    MyPotentialFunction potential_function(d_star,zetta,Q_star,eta,cells_per_unit,problem);

    path.waypoints.push_back(problem.q_init);
    current_location = problem.q_init;

    // Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
    while ((current_location-problem.q_goal).norm() > 0.25){
        if (checkTimeout(t1, 1.0)){return path;} // Timeout
        current_location = current_location - potential_function.getGradient(current_location)*0.05;
        path.waypoints.push_back(current_location);
    }

    path.waypoints.push_back(problem.q_goal);
    return path;
}

MyPotentialFunction::MyPotentialFunction(double d_star, double zetta, double Q_star, double eta, int cells_per_unit, const amp::Problem2D& problem)
    : d_star(d_star), zetta(zetta), Q_star(Q_star), eta(eta), cells_per_unit(cells_per_unit), problem(problem)
{
    distant_arrays = brushFireIdividual();
}

amp::DenseArray2D<int> MyPotentialFunction::brushFireAll(){
    check_collision_problem check_collision(problem);
    int x_cells_num = (problem.x_max-problem.x_min)*cells_per_unit;
    int y_cells_num = (problem.y_max-problem.y_min)*cells_per_unit;
    int num_of_cells = x_cells_num*y_cells_num;
    int changed_num_of_cells = 0;
    amp::DenseArray2D<int> distant_array(x_cells_num, y_cells_num);

    double cell_size = double(1.0)/cells_per_unit;
    
    // find obstacle
    for(int i = 0; i < x_cells_num; i++){
        for(int j = 0; j < y_cells_num; j++){
            if (check_collision.all(Eigen::Vector2d(cell_size*i+cell_size/2+problem.x_min, cell_size*j+cell_size/2+problem.y_min))){
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

std::vector<amp::DenseArray2D<int>> MyPotentialFunction::brushFireIdividual(){
    check_collision_problem check_collision(problem);
    int x_cells_num = (problem.x_max-problem.x_min)*cells_per_unit;
    int y_cells_num = (problem.y_max-problem.y_min)*cells_per_unit;
    int num_of_cells = x_cells_num*y_cells_num;
    double cell_size = double(1.0)/cells_per_unit;

    for(int ob_idx = 0; ob_idx < problem.obstacles.size(); ob_idx++){
        int changed_num_of_cells = 0;
        amp::DenseArray2D<int> distant_array(x_cells_num, y_cells_num);
        amp::DenseArray2D<int> gradient_array(x_cells_num, y_cells_num);

        // find obstacle
        for(int i = 0; i < x_cells_num; i++){
            for(int j = 0; j < y_cells_num; j++){
                if (check_collision.thisObstacle(ob_idx, Eigen::Vector2d(cell_size*i+cell_size/2+problem.x_min, cell_size*j+cell_size/2+problem.y_min))){
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
                                    gradient_array(i+k, j+l) = (k+2) + (l+1)*3;
                                    changed_num_of_cells += 1;
                                }
                            }
                        }

                    }
                }
            }
            current_distant += 1;
        }
        distant_arrays.push_back(distant_array);
        gradient_arrays.push_back(gradient_array);
    }
    return distant_arrays;
}

double MyPotentialFunction::attraction(const Eigen::Vector2d& q) const{
    double distant_to_goal = (problem.q_goal-q).norm();
    if(distant_to_goal<=d_star){
        return zetta*pow(distant_to_goal,2)/2;
    }
    return d_star*zetta*distant_to_goal - zetta*pow(d_star,2)/2;
}

Eigen::Vector2d MyPotentialFunction::attractionGradient(const Eigen::Vector2d& q) const{
    double distant_to_goal = (problem.q_goal-q).norm();
    if(distant_to_goal<=d_star){
        return zetta*(q - problem.q_goal);
    }
    return d_star*zetta*(q - problem.q_goal)/distant_to_goal;
}

double MyPotentialFunction::repulsive(const Eigen::Vector2d& q) const{
    double repulsive = 0;
    int i = floor((q[0]-problem.x_min)*cells_per_unit);
    int j = floor((q[1]-problem.y_min)*cells_per_unit);

    for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ob_idx++){
        double distant_to_obstacle = distant_arrays[ob_idx](i, j);
        if(distant_to_obstacle == 0){
            distant_to_obstacle = pow(10,-6);
        }
        
        if (distant_to_obstacle <= Q_star){
            repulsive += eta*pow(1/Q_star-1/distant_to_obstacle,2)/2;
        }
    }
    return repulsive;
}

Eigen::Vector2d MyPotentialFunction::repulsiveGradient(const Eigen::Vector2d& q) const{
    Eigen::Vector2d repulsive_gradient = Eigen::Vector2d(0.0, 0.0);
    Eigen::Vector2d temp_repulsive_gradient = Eigen::Vector2d(0.0, 0.0);
    int i = floor((q[0]-problem.x_min)*cells_per_unit);
    int j = floor((q[1]-problem.y_min)*cells_per_unit);

    for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ob_idx++){
        double distant_to_obstacle = distant_arrays[ob_idx](i, j);
        if(distant_to_obstacle == 0){
            distant_to_obstacle = pow(10,-6);
        }
        if (distant_to_obstacle <= Q_star){
            switch (gradient_arrays[ob_idx](i,j))
            {
            case 0:
                temp_repulsive_gradient = Eigen::Vector2d(0.0, 0.0);
                break;

            case 1:
                temp_repulsive_gradient = Eigen::Vector2d(-0.707107, -0.707107);
                break;

            case 2:
                temp_repulsive_gradient = Eigen::Vector2d(0.0, -1.0);
                /* code */
                break;
            
            case 3:
                temp_repulsive_gradient = Eigen::Vector2d(0.707107, -0.707107);
                /* code */
                break;

            case 4:
                temp_repulsive_gradient = Eigen::Vector2d(-1.0, 0.0);
                /* code */
                break;

            case 5:
                temp_repulsive_gradient = Eigen::Vector2d(0.0, 0.0);
                break;

            case 6:
                temp_repulsive_gradient = Eigen::Vector2d(1.0, 0.0);
                /* code */
                break;

            case 7:
                temp_repulsive_gradient = Eigen::Vector2d(-0.707107, 0.707107);
                /* code */
                break;

            case 8:
                temp_repulsive_gradient = Eigen::Vector2d(0.0, 1.0);
                /* code */
                break;

            case 9:
                temp_repulsive_gradient = Eigen::Vector2d(0.707107, 0.707107);
                /* code */
                break;

            default:
                ERROR(gradient_arrays[ob_idx](i,j));
                ERROR("something wrong when building distant gradient");
                break;
            }
            
            temp_repulsive_gradient = eta*(1/Q_star-1/distant_to_obstacle)*temp_repulsive_gradient/pow(distant_to_obstacle,2);
            repulsive_gradient += temp_repulsive_gradient;
        }
    }
    return repulsive_gradient;
}
