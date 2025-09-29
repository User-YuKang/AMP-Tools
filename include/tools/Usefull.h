#pragma once

#include "tools/Obstacle.h"
#include "tools/Environment.h"

inline bool check_collision_problem(const amp::Problem2D& problem, Eigen::Vector2d point) {
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