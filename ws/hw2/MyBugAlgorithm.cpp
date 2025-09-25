#include "MyBugAlgorithm.h"
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug1Algorithm::plan(const amp::Problem2D& problem) {
    // Set step size
    double forward_hand = 0.07;
    double step_multiply = 0.10;
    double angle_step = 0.05;
    double angle_multiply = 10;

    // Set up initial variables
    amp::Path2D path;
    Eigen::Vector2d current_position = problem.q_init;
    Eigen::Vector2d previous_position = problem.q_init;
    LeftTurningBugEsential left_turning_bug_esential(forward_hand, forward_hand*1.2, step_multiply, 0);
    Eigen::Vector2d hit_point;
    Eigen::Vector2d leave_point;
    double old_angle;

    // Determine initial orientation
    Eigen::Vector2d to_goal = problem.q_goal - problem.q_init;
    double initial_angle = atan2(to_goal(1),to_goal(0));
    left_turning_bug_esential.increase_angle(initial_angle);

    // Initial location
    path.waypoints.push_back(problem.q_init);

    // Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
    while(true){
        if (check_timeout(t1)){LOG("1");return path;} // Timeout

        // Move towards goal until collision
        while((!check_collision(problem, current_position + left_turning_bug_esential.forward)) 
              && (current_position - problem.q_goal).norm() > 0.1){
            if (check_timeout(t1)){LOG("2");return path;}
            previous_position = current_position;
            current_position += left_turning_bug_esential.step_forward;
        }

        // Reached goal
        if ((current_position - problem.q_goal).norm() <= 0.1){
            break; // Reached goal
        }

        // Collision
        path.waypoints.push_back(current_position);

        // Path to go back to leave point
        amp::Path2D temp_path_memory;
        amp::Path2D temp_path_1;
        amp::Path2D temp_path_2;
        temp_path_1.waypoints.push_back(current_position);
        temp_path_memory.waypoints.push_back(current_position);

        // Reposition to angle to circumnavigate obstacle
        while(check_collision(problem, current_position + left_turning_bug_esential.forward) 
              || !check_collision(problem, current_position + left_turning_bug_esential.right)){
            if (check_timeout(t1)){LOG("3");return path;}
            left_turning_bug_esential.increase_angle(angle_step);
        }

        // Circumnavigate obstacle
        old_angle = left_turning_bug_esential.angle;
        hit_point = current_position;
        leave_point = current_position;
        bool right_turn = 0;
        while(abs(left_turning_bug_esential.angle-old_angle) < 2*M_PI-1 || (current_position - hit_point).norm() > 0.1){
            if (check_timeout(t1)){LOG("4");return path;} // Timeout

            previous_position = current_position;
            current_position += left_turning_bug_esential.step_forward;

            // if did not touch obstacle, move back and turn right
            if(!check_collision(problem, current_position + left_turning_bug_esential.right)){
                current_position = previous_position;
                if(!right_turn){
                    path.waypoints.push_back(current_position);
                    temp_path_2.waypoints.push_back(current_position);
                    temp_path_memory.waypoints.push_back(current_position);
                }

                // turn around right point
                current_position += left_turning_bug_esential.right + Eigen::Rotation2Dd(-angle_step*angle_multiply) * (-left_turning_bug_esential.right);
                path.waypoints.push_back(current_position);
                temp_path_2.waypoints.push_back(current_position);
                temp_path_memory.waypoints.push_back(current_position);

                left_turning_bug_esential.increase_angle(-angle_step * angle_multiply);

                right_turn = 1;
            }

            bool first_time = 1;
            // while front or front_right is blocked, or right is free, turn left
            while(check_collision(problem, current_position + left_turning_bug_esential.forward_right)||
                  check_collision(problem, current_position + left_turning_bug_esential.forward) || 
                  !check_collision(problem, current_position + left_turning_bug_esential.right)){
                if (check_timeout(t1)){LOG("5");return path;} // Timeout

                // store path if turn left for the first time
                if(first_time){
                    path.waypoints.push_back(current_position);
                    temp_path_2.waypoints.push_back(current_position);
                    temp_path_memory.waypoints.push_back(current_position);
                    first_time = 0;
                }
                left_turning_bug_esential.increase_angle(angle_step);
                right_turn = 0;
                // LOG(left_turning_bug_esential.forward_right);
            }

            // check if it is closer to goal, and update leave point if so
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
        left_turning_bug_esential.set_angle(atan2(to_goal(1),to_goal(0)));

        if (check_collision(problem, current_position + left_turning_bug_esential.forward)){
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
    // set step size
    const double forward_hand = 0.05;
    const double step_multiply = 0.1;
    const double angle_step = 0.05;
    const double angle_multiply = 10;

    // set up initial variables
    amp::Path2D path;
    Eigen::Vector2d current_position;
    Eigen::Vector2d previous_position;
    LeftTurningBugEsential left_turning_bug_esential(forward_hand, forward_hand*1.2, step_multiply, 0);
    double old_angle;

    // Determine goal line
    const double goal_slope = (problem.q_goal(1)-problem.q_init(1))/(problem.q_goal(0)-problem.q_init(0))+1E-10; // add small value to avoid infinite slope
    const double goal_intercept = problem.q_init(1)-goal_slope*problem.q_init(0);
    Eigen::Vector2d goal_direction = (problem.q_goal - problem.q_init).normalized();
    double angle_to_goal = atan2(goal_direction(1),goal_direction(0));

    // Initial Orientation and save initial position
    const double initial_angle = angle_to_goal;
    left_turning_bug_esential.set_angle(initial_angle);
    current_position = problem.q_init;
    path.waypoints.push_back(problem.q_init);

    // start bug2 algorithm
    // Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
    while(true){
        if (check_timeout(t1)){LOG("1");return path;} // Timeout

        // Move towards goal until collision
        while((!check_collision(problem, current_position + left_turning_bug_esential.forward)) 
              && (current_position - problem.q_goal).norm() > 0.1){
            if (check_timeout(t1)){LOG("2");return path;}
            previous_position = current_position;
            current_position += left_turning_bug_esential.step_forward;
        }

        // Reached goal
        if ((current_position - problem.q_goal).norm() <= 0.1){
            break; // Reached goal
        }

        // Collision
        path.waypoints.push_back(current_position);

        // Reposition to angle to circumnavigate obstacle
        while(check_collision(problem, current_position + left_turning_bug_esential.forward) 
              || !check_collision(problem, current_position + left_turning_bug_esential.right)){
            if (check_timeout(t1)){LOG("3");return path;}
            left_turning_bug_esential.increase_angle(angle_step);
        }

        // Circumnavigate obstacle until reach goal line
        old_angle = left_turning_bug_esential.angle;
        Eigen::Vector2d hit_point = current_position;
        double distance_to_goal = (hit_point-problem.q_goal).norm();
        bool right_turn = 0;
        while(true){
            if (check_timeout(t1)){LOG("4");return path;} // Timeout

            previous_position = current_position;
            current_position += left_turning_bug_esential.step_forward;

            // if did not touch obstacle, move back and turn right
            if(!check_collision(problem, current_position + left_turning_bug_esential.right)){
                current_position = previous_position;
                if(!right_turn){
                    path.waypoints.push_back(current_position);
                }

                // turn around right point
                current_position += left_turning_bug_esential.right + Eigen::Rotation2Dd(-angle_step*angle_multiply) * (-left_turning_bug_esential.right);
                path.waypoints.push_back(current_position);

                left_turning_bug_esential.increase_angle(-angle_step * angle_multiply);

                right_turn = 1;
            }

            bool first_time = 1;
            // while front or front_right is blocked, or right is free, turn left
            while(check_collision(problem, current_position + left_turning_bug_esential.forward_right)||
                  check_collision(problem, current_position + left_turning_bug_esential.forward) || 
                  !check_collision(problem, current_position + left_turning_bug_esential.right)){
                if (check_timeout(t1)){LOG("5");return path;} // Timeout

                // store path if turn left for the first time
                if(first_time){
                    path.waypoints.push_back(current_position);
                    first_time = 0;
                }
                left_turning_bug_esential.increase_angle(angle_step);
                right_turn = 0;
                // LOG(left_turning_bug_esential.forward_right);
            }

            if(abs(left_turning_bug_esential.angle-old_angle) > 2*M_PI-1 && (current_position - hit_point).norm() < 0.1){
                path.waypoints.push_back(current_position);
                LOG("Stuck in obstacle!");
                return path; // Stuck in obstacle
            }

            if(abs(left_turning_bug_esential.angle-old_angle) > M_PI/6 && 
               point_to_line_distance(current_position, goal_slope, goal_intercept) < 0.1 &&
               (current_position-problem.q_goal).norm() < distance_to_goal){
                double x_closest = (current_position(1)+current_position(0)/goal_slope-goal_intercept)/(goal_slope+1/goal_slope);
                double y_closest = goal_slope*x_closest + goal_intercept;

                double temp_old_angle = left_turning_bug_esential.angle;
                Eigen::Vector2d temp_to_goal = (problem.q_goal - Eigen::Vector2d(x_closest, y_closest)).normalized();
                double temp_angle_to_goal = atan2(temp_to_goal(1),temp_to_goal(0));

                left_turning_bug_esential.set_angle(temp_angle_to_goal);

                if (!check_collision(problem, current_position + left_turning_bug_esential.forward)){
                    path.waypoints.push_back(current_position);
                    break;
                }
                left_turning_bug_esential.set_angle(temp_old_angle);
            }
        }
    }
    

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

LeftTurningBugEsential::LeftTurningBugEsential(double forward_hand, double right_hand, double input_step_multiply, double input_angle){
    initial_forward = Eigen::Vector2d(forward_hand, 0);
    initial_forward_right = Eigen::Rotation2Dd(-M_PI/4)*initial_forward;
    initial_right = Eigen::Vector2d(0, -right_hand);

    angle = input_angle;
    step_multiply = input_step_multiply;
    rot = Eigen::Rotation2Dd(angle);
    forward = rot * initial_forward;
    forward_right = rot * initial_forward_right;
    right = rot * initial_right;
    step_forward = forward*step_multiply;
}

void LeftTurningBugEsential::increase_angle(double incremental){
    angle += incremental;
    rot = Eigen::Rotation2Dd(angle);
    forward = rot * initial_forward;
    forward_right = rot * initial_forward_right;
    right = rot * initial_right;
    step_forward = forward*step_multiply;
}

void LeftTurningBugEsential::set_angle(double input_angle){
    angle = input_angle;
    rot = Eigen::Rotation2Dd(angle);
    forward = rot * initial_forward;
    forward_right = rot * initial_forward_right;
    right = rot * initial_right;
    step_forward = forward*step_multiply;
}
