#include "HelpfulClass.h"

Point2DCollisionChecker::Point2DCollisionChecker(const amp::Environment2D& enviroument_)
: env(enviroument_){
    std::vector<std::pair<double,double>> bounds;
    bounds.push_back({enviroument_.x_min, enviroument_.x_max});
    bounds.push_back({enviroument_.y_min, enviroument_.y_max});
    setBounds(bounds);
}

bool Point2DCollisionChecker::isCollide(const Eigen::VectorXd& point) {
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < env.obstacles.size(); ++ob_idx) {
        if (thisObstacle(ob_idx, point)){
            return true; // Point is inside this polygons
        }
    }
    return false; // Point is outside all polygons
    // Implementation
}

bool Point2DCollisionChecker::isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_){
    Eigen::VectorXd one_2_two = point2_ - point1_;
    double distance = one_2_two.norm();
    int sec_num = 1;
    while(distance/sec_num > GAP){
        for(int i = 0; i < sec_num; i++){
            Eigen::VectorXd check_location = point1_ + one_2_two*(1+2*i)/(sec_num*2);
            if (isCollide(check_location))
            {
                return true;
            }
        }
        sec_num = sec_num * 2;
    }
    return false;
}

bool Point2DCollisionChecker::thisObstacle(int ob_idx_, const Eigen::VectorXd& point_){
    const amp::Obstacle2D& obstacle = env.obstacles[ob_idx_];
    double cross_product;

    int n = obstacle.verticesCCW().size();
    for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
        const Eigen::VectorXd& v1 = obstacle.verticesCCW()[vertix_idx];
        const Eigen::VectorXd& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
        Eigen::VectorXd edge = v1 - v2;
        Eigen::VectorXd point_to_v1 = point_ - v1;
        cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
        if (cross_product > 0){
            return false; // Point is outside of this obstacle
        }
    }
    return true; // Point is inside this obstacle
}

