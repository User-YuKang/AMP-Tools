#include "MyBugAlgorithm.h"
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug1Algorithm::plan(const amp::Problem2D& problem) {
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    // Set step size
    double step_size = 0.05;
    double step_multiply = 0.1;
    double angle_step = 0.05;
    double angle_multiply = 10;

    // Set up initial variables
    amp::Path2D path;
    Eigen::Vector2d current_position = problem.q_init;
    Eigen::Vector2d previous_position = problem.q_init;
    Eigen::Vector2d initial_forward(step_size,0);
    Eigen::Vector2d initial_forward_right = Eigen::Rotation2Dd(-M_PI/4) * Eigen::Vector2d(step_size,0);
    Eigen::Vector2d initial_right = Eigen::Rotation2Dd(-M_PI/2) * Eigen::Vector2d(step_size*1.4,0);
    Eigen::Vector2d forward;
    Eigen::Vector2d forward_right;
    Eigen::Vector2d right;
    Eigen::Vector2d step_forward;
    Eigen::Rotation2Dd rot;
    Eigen::Vector2d hit_point;
    Eigen::Vector2d leave_point;
    double old_angle;

    // Determine initial orientation
    Eigen::Vector2d to_goal = problem.q_goal - problem.q_init;
    double angle = atan2(to_goal(1),to_goal(0));
    rot = Eigen::Rotation2Dd(angle);
    forward = rot * initial_forward;
    forward_right = rot * initial_forward_right;
    right = rot * initial_right;
    step_forward = forward*step_multiply;

    // Initial location
    path.waypoints.push_back(problem.q_init);

    auto t1 = std::chrono::high_resolution_clock::now();
    while(true){
        if (check_timeout(t1)){LOG("1");return path;}
        // Move towards goal until collision
        while((!check_collision(problem, current_position + forward)) && (current_position - problem.q_goal).norm() > 0.1){
            if (check_timeout(t1)){LOG("2");return path;}
            previous_position = current_position;
            current_position += step_forward;
        }
        // Reached goal
        if ((current_position - problem.q_goal).norm() <= 0.1){
            break; // Reached goal
        }
        // Collision
        path.waypoints.push_back(current_position);
        amp::Path2D temp_path_memory;
        amp::Path2D temp_path_1;
        amp::Path2D temp_path_2;
        temp_path_1.waypoints.push_back(current_position);
        temp_path_memory.waypoints.push_back(current_position);
        // LOG("reach obstacle");

        // Reposition angle
        while(check_collision(problem, current_position + forward) || !check_collision(problem, current_position + right)){
            if (check_timeout(t1)){LOG("3");return path;}
            // LOG(check_collision(problem, current_position + forward));
            angle += angle_step;
            rot = Eigen::Rotation2Dd(angle);
            forward = rot * initial_forward;
            forward_right = rot * initial_forward_right;
            right = rot * initial_right;
            step_forward = forward*step_multiply;
        }

        // Circumnavigate obstacle
        old_angle = angle;
        hit_point = current_position;
        leave_point = current_position;
        bool right_turn = 0;
        while(abs(angle-old_angle) < 2*M_PI-1 || (current_position - hit_point).norm() > 0.1){
            if (check_timeout(t1)){LOG("4");return path;}

            previous_position = current_position;
            current_position += step_forward;
            if(!check_collision(problem, current_position + right)){
                current_position = previous_position;
                if(!right_turn){
                    path.waypoints.push_back(current_position);
                    temp_path_2.waypoints.push_back(current_position);
                    temp_path_memory.waypoints.push_back(current_position);
                }

                current_position += right + Eigen::Rotation2Dd(-angle_step*angle_multiply) * (-right);
                path.waypoints.push_back(current_position);
                temp_path_2.waypoints.push_back(current_position);
                temp_path_memory.waypoints.push_back(current_position);

                angle -= angle_step * angle_multiply;
                rot = Eigen::Rotation2Dd(angle);
                forward = rot * initial_forward;
                forward_right = rot * initial_forward_right;
                right = rot * initial_right;
                step_forward = forward*step_multiply;

                right_turn = 1;
            }
            bool first_time = 1;
            while((check_collision(problem, current_position + forward_right)||check_collision(problem, current_position + forward))|| !check_collision(problem, current_position + right)){
                if (check_timeout(t1)){LOG("5");return path;}
                // LOG("new:");
                // LOG(check_collision(problem, current_position + forward_right));
                // LOG(check_collision(problem, current_position + right));
                if(first_time){
                    path.waypoints.push_back(current_position);
                    temp_path_2.waypoints.push_back(current_position);
                    temp_path_memory.waypoints.push_back(current_position);
                    first_time = 0;
                }

                angle += angle_step;
                rot = Eigen::Rotation2Dd(angle);
                forward = rot * initial_forward;
                forward_right = rot * initial_forward_right;
                right = rot * initial_right;
                step_forward = forward*step_multiply;
                
                right_turn = 0;
            }

            if((current_position - problem.q_goal).norm() < (leave_point - problem.q_goal).norm()){
                temp_path_1.waypoints = temp_path_memory.waypoints;
                temp_path_1.waypoints.push_back(current_position);
                leave_point = current_position;
                temp_path_2.waypoints.clear();
                temp_path_2.waypoints.push_back(current_position);
            }
            
        }
        path.waypoints.push_back(current_position);

        // change direction of temp_path_2
        std::reverse(temp_path_2.waypoints.begin(), temp_path_2.waypoints.end());

        // Move to leave point
        if(temp_path_1.length() < temp_path_2.length()){
            path.waypoints.insert(path.waypoints.end(), temp_path_1.waypoints.begin(), temp_path_1.waypoints.end());
        }
        else{
            path.waypoints.insert(path.waypoints.end(), temp_path_2.waypoints.begin(), temp_path_2.waypoints.end());
        }
        current_position = leave_point;

        to_goal = problem.q_goal - leave_point;
        angle = atan2(to_goal(1),to_goal(0));
        rot = Eigen::Rotation2Dd(angle);
        forward = rot * initial_forward;
        forward_right = rot * initial_forward_right;
        right = rot * initial_right;
        step_forward = forward*step_multiply;

        if (check_collision(problem, current_position + forward)){
            LOG("Stuck in obstacle!");
            return path; // Stuck in obstacle
        }
    }

    // Get to goal
    path.waypoints.push_back(problem.q_goal);

    return path;
}

bool MyBug1Algorithm::check_reach_hit(Eigen::Vector2d hit_point, Eigen::Vector2d current_position, double old_angle, double angle) {
    if(abs(angle-old_angle) < 2*M_PI-0.1 || (current_position - hit_point).norm() > 0.05){
        return false;
    }
    return true;
}

amp::Path2D MyBug2Algorithm::plan(const amp::Problem2D& problem) {
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    const double goal_slope = (problem.q_goal(1)-problem.q_init(1))/(problem.q_goal(0)-problem.q_init(0));
    const double goal_intercept = problem.q_init(1)-goal_slope*problem.q_init(0);

    // Move towards goal until collision
    

    path.waypoints.push_back(problem.q_goal);
    return path;
}

double MyBug2Algorithm::point_to_line_distance(Eigen::Vector2d point, double slope, double intercept) {
    return std::abs(slope*point(0)-point(1)+intercept)/std::sqrt(slope*slope+1);
}

bool MyBugAlgorithm::check_timeout(std::chrono::time_point<std::chrono::high_resolution_clock> t1) {
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = t2 - t1;
    double time_pass = elapsed.count();
    if(time_pass>0.1){
        return true;
    }
    return false;
}

bool MyBugAlgorithm::check_collision(const amp::Problem2D& problem, Eigen::Vector2d point) {
    amp::Obstacle2D obstacle;
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ++ob_idx) {
        obstacle = problem.obstacles[ob_idx];
        

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