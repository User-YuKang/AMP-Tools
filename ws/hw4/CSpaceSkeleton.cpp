#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = 0; // x index of cell
    std::size_t cell_y = 0; // x index of cell
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    // std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    // MyGridCSpace2D& cspace = *cspace_ptr;

    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);
    MyGridCSpace2D& cspace = *cspace_ptr;
    double step = 2*M_PI/m_cells_per_dim;
    double half_step = step/2;
    double critical = 0.01;
    bool crash = false;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for(int i = 0; i < m_cells_per_dim; i++){
        Eigen::Vector2d state = Eigen::Vector2d(i*step+half_step, 0);
        Eigen::Vector2d joint1 = manipulator.getJointLocation(state, 1);
        crash = check_collision(env,joint1);
        if(!crash){
            crash = check_collision_between_point(env, manipulator.getBaseLocation(), joint1, critical);
        }

        for(int j = 0; j < m_cells_per_dim; j++){
            if (crash){
                cspace(i,j) = true;
                continue;
            }

            state[1] = j*step+half_step;
            Eigen::Vector2d end_effector = manipulator.getJointLocation(state, 2);
            if (check_collision(env, end_effector)){
                cspace(i,j) = true;
                continue;
            }
            cspace(i,j) = check_collision_between_point(env, joint1, end_effector, critical);
        }
        crash = false;
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

bool MyManipulatorCSConstructor::check_collision(const amp::Environment2D& env, Eigen::Vector2d point) {
    amp::Obstacle2D obstacle;
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < env.obstacles.size(); ++ob_idx) {
        obstacle = env.obstacles[ob_idx];
        

        int n = obstacle.verticesCCW().size();
        for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
            const Eigen::Vector2d& v1 = obstacle.verticesCCW()[vertix_idx];
            const Eigen::Vector2d& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
            Eigen::Vector2d edge = v1 - v2;
            Eigen::Vector2d point_to_v1 = point - v1;
            cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
            if (cross_product > 0){
                possible_collision = 0;
                break; // Point is outside of this obstacle
            }
            // std::cout << "Cross product with edge " << vertix_idx << ": " << cross_product << std::endl;
        }
        if (!possible_collision) {
            possible_collision = 1;
            continue; // Check the next obstacle
        }
        // std::cout << "Collision with obstacle " << ob_idx << std::endl;
        // std::cout << "Collision: True" << std::endl;
        return true; // Point is inside this obstacle
    }
    return false; // Point is outside all polygons
}

bool MyManipulatorCSConstructor::check_collision_between_point(const amp::Environment2D& env, Eigen::Vector2d point1, Eigen::Vector2d point2, double critical){
    Eigen::Vector2d first2second = point2 - point1;
    double previous_step = first2second.norm();
    double devide = 2;

    while(previous_step > critical){
        for(double k = 1; k < devide; k+=2){
            if (check_collision(env, point1 + first2second*(k/devide))){
                return true;
            }
        }
        devide = devide*2;
        previous_step = previous_step/2;
    }
    return false;
}
