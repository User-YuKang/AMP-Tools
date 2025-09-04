#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <Eigen/Geometry>
#include <chrono>

class LeftTurningBugEsential{
    public:
        Eigen::Vector2d forward;
        Eigen::Vector2d forward_right;
        Eigen::Vector2d right;
        Eigen::Vector2d step_forward;
        Eigen::Rotation2Dd rot;
        double angle;
        LeftTurningBugEsential(double, double, double, double);
        void increase_angle(double);
        void set_angle(double);

    private:
        Eigen::Vector2d initial_forward;
        Eigen::Vector2d initial_forward_right;
        Eigen::Vector2d initial_right;
        double step_multiply;
};

class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        bool check_collision(const amp::Problem2D& problem, Eigen::Vector2d point);
        bool check_timeout(std::chrono::time_point<std::chrono::high_resolution_clock> t1);
        // Add any other methods here...
    
    private:
        // Add any member variables here...
};

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBug1Algorithm : public MyBugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        // Add any other methods here...
    
    private:
        bool check_reach_hit(Eigen::Vector2d hit_point, Eigen::Vector2d current_position, double old_angle, double angle);
        // Add any member variables here...
};

class MyBug2Algorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        // Add any other methods here...
    
    private:
        // Add any member variables here...
        double point_to_line_distance(Eigen::Vector2d point, double slope, double intercept);
        LeftTurningBugEsential left_turning_bug_esential();
};