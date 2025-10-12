#include "MyCSConstructors.h"

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

// std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
//     // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
//     std::size_t cell_x = 0; // x index of cell
//     std::size_t cell_y = 0; // x index of cell
//     return {cell_x, cell_y};
// }

// // Override this method for computing all of the boolean collision values for each cell in the cspace
// std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
//     // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
//     // Pass the constructor parameters to std::make_unique()
//     std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
//     // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
//     MyGridCSpace2D& cspace = *cspace_ptr;
//     std::cout << "Constructing C-space for manipulator" << std::endl;
//     // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
//     cspace(1, 3) = true;
//     cspace(3, 3) = true;
//     cspace(0, 1) = true;
//     cspace(1, 0) = true;
//     cspace(2, 0) = true;
//     cspace(3, 0) = true;
//     cspace(4, 1) = true;

//     // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
//     // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
//     return cspace_ptr;
// }

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    double cell_width_height = (env.x_max-env.x_min)/m_cells_per_dim;
    check_collision_enviroument check_collision(env);
    std::cout << "Constructing C-space for point agent" << std::endl;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for(int cell_n = 0; cell_n < m_cells_per_dim; cell_n++){
        for(int cell_m = 0; cell_m < m_cells_per_dim; cell_m++){
            for(int i = 0; i < 10; i++){
                double random_x = env.x_min+cell_width_height*cell_n+(rand()%98+1)/100.0*cell_width_height;
                double random_y = env.y_min+cell_width_height*cell_m+(rand()%98+1)/100.0*cell_width_height;
                if (check_collision.all(Eigen::Vector2d(random_x, random_y))){
                    cspace(cell_n, cell_m) = true;
                    break;
                }
            }
        }
    }
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    amp::DenseArray2D<int> wavefront(grid_cspace.size().first, grid_cspace.size().second);
    for(int i = 0; i < grid_cspace.size().first; i++){
        for (int j = 0; j < grid_cspace.size().second; j++){
            if(grid_cspace(i,j)){
                wavefront(i,j) = 1;
            }
        }
    }

    double cell_width_height = (grid_cspace.x0Bounds().second-grid_cspace.x0Bounds().first)/grid_cspace.size().first;
    int init_x_loc = (q_init(0)-grid_cspace.x0Bounds().first)/cell_width_height;
    int init_y_loc = (q_init(1)-grid_cspace.x1Bounds().first)/cell_width_height;
    int goal_x_loc = (q_goal(0)-grid_cspace.x0Bounds().first)/cell_width_height;
    int goal_y_loc = (q_goal(1)-grid_cspace.x1Bounds().first)/cell_width_height;

    wavefront(init_x_loc, init_y_loc) = 2;

    bool changed_cell = false;
    int current_distant = 2;
    do{
        changed_cell = false;
        for(int i = 0; i < grid_cspace.size().first*grid_cspace.size().second; i++){
            if (wavefront(goal_x_loc, goal_y_loc) != 0){
                break;
            }
            if(wavefront(i%grid_cspace.size().first, int(i/grid_cspace.size().first)) == current_distant){
                if (int(i/grid_cspace.size().first)>=1 && wavefront(i%grid_cspace.size().first, int(i/grid_cspace.size().first)-1) == 0){
                    wavefront(i%grid_cspace.size().first, int(i/grid_cspace.size().first)-1) = current_distant+1;
                    changed_cell = true;
                }
                if (int(i/grid_cspace.size().first)+1<grid_cspace.size().second && wavefront(i%grid_cspace.size().first, int(i/grid_cspace.size().first)+1) == 0) {
                    wavefront(i%grid_cspace.size().first, int(i/grid_cspace.size().first)+1) = current_distant+1;
                    changed_cell = true;
                }
                if (i%grid_cspace.size().first>=1 && wavefront(i%grid_cspace.size().first-1, int(i/grid_cspace.size().first)) == 0){
                    wavefront(i%grid_cspace.size().first-1, int(i/grid_cspace.size().first)) = current_distant+1;
                    changed_cell = true;
                }
                if (i%grid_cspace.size().first+1<grid_cspace.size().first && wavefront(i%grid_cspace.size().first+1, int(i/grid_cspace.size().first)) == 0){
                    wavefront(i%grid_cspace.size().first+1, int(i/grid_cspace.size().first)) = current_distant+1;
                    changed_cell = true;
                }
            }
        }
        current_distant += 1;
    }while(changed_cell == true);


    int x = goal_x_loc;
    int y = goal_y_loc;

    for (int distant = wavefront(goal_x_loc, goal_y_loc); distant > 3; distant--){
        if (y>=1 && wavefront(x, y-1) == distant - 1){
            y -= 1;
        }
        else if (y+1<grid_cspace.size().second && wavefront(x, y+1) == distant - 1) {
            y += 1;
        }
        else if (x>=1 && wavefront(x-1, y) == distant - 1){
            x -= 1;
        }
        else if (x+1<grid_cspace.size().first && wavefront(x+1, y) == distant - 1){
            x += 1;
        }
        path.waypoints.insert(path.waypoints.begin()+1, Eigen::Vector2d(grid_cspace.x0Bounds().first + x * cell_width_height + cell_width_height/2, grid_cspace.x1Bounds().first + y * cell_width_height + cell_width_height/2));
    }

    path.waypoints.push_back(q_goal);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(-M_PI, -M_PI);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(M_PI, M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
        bool need_to_reduce_0 = false;
        bool need_to_reduce_1 = false;
        for (int i = 0; i < path.waypoints.size(); i++){
            if(path.waypoints[i](0) > M_PI){
                need_to_reduce_0 = true;
            }
            if(path.waypoints[i](1) > M_PI){
                need_to_reduce_1 = true;
            }
        }
        if (need_to_reduce_0){
            for (int i = 0; i < path.waypoints.size(); i++){
                path.waypoints[i](0) -= 2*M_PI;
            }
        }
        if (need_to_reduce_1){
            for (int i = 0; i < path.waypoints.size(); i++){
                path.waypoints[i](1) -= 2*M_PI;
            }
        }
        
    }
    return path;
}
