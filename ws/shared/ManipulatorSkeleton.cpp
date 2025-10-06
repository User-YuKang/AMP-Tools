#include "ManipulatorSkeleton.h"

// Comparator function
bool comp(int a, int b) {
    return a > b;
}

double sum(std::vector<double> v){
    double total=0;
    for (int i = 0; i < v.size(); i++){
        total += v[i];
    }
    return total;
}

double smallestAbsuluteOfSum(std::vector<double> v){
    double smallest = 0;
    for (int i = 0; i < v.size(); i++){
        if (smallest < v[i]){
            smallest = v[i] - smallest;
        }
        else{
            smallest = smallest - v[i];
        }
    }
    return smallest;
}

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

MyManipulator2D::MyManipulator2D(const std::vector<double>& link_lengths)
    : LinkManipulator2D(link_lengths) // Pass the link lengths to the base class constructor
{}

Eigen::Transform<double, 2, Eigen::Affine> MyManipulator2D::HomogeneousTransform(double theta,const Eigen::Vector2d& tran) const {
    
    Eigen::Rotation2D<double> rotation(theta);
    Eigen::Translation<double,2> translation(tran[0], tran[1]);
    Eigen::Transform<double, 2, Eigen::Affine> transform = translation * rotation;
    return transform;
}

amp::ManipulatorState MyManipulator2D::getConfiguration2link(const Eigen::Vector2d& end_effector_location) const {
    amp::ManipulatorState joint_angles(2);
    double cos_theta2 = (pow(end_effector_location[0], 2) + pow(end_effector_location[1], 2) - pow(getLinkLengths()[0], 2) - pow(getLinkLengths()[1], 2)) / (2 * getLinkLengths()[0] * getLinkLengths()[1]);
    joint_angles[1] = acos(cos_theta2);

    double cos_theta1 = (end_effector_location[0] * (getLinkLengths()[0] + getLinkLengths()[1] * cos_theta2) + end_effector_location[1] * (getLinkLengths()[1] * sin(joint_angles[1]))) / (pow(getLinkLengths()[0] + getLinkLengths()[1] * cos_theta2, 2) + pow(getLinkLengths()[1] * sin(joint_angles[1]), 2));
    double sin_theta1 = (end_effector_location[1] * (getLinkLengths()[0] + getLinkLengths()[1] * cos_theta2) - end_effector_location[0] * (getLinkLengths()[1] * sin(joint_angles[1]))) / (pow(getLinkLengths()[0] + getLinkLengths()[1] * cos_theta2, 2) + pow(getLinkLengths()[1] * sin(joint_angles[1]), 2));
    joint_angles[0] = atan2(sin_theta1, cos_theta1);
    return joint_angles;
}

Eigen::Vector2d MyManipulator2D::possibleJointLocation(double r1, double r2, double linklength, const Eigen::Vector2d& end_effector_location) const{
    double end_base_distant = (end_effector_location - getBaseLocation()).norm();
    Eigen::Vector2d joint_location = Eigen::Vector2d(0.0, 0.0);

    bool intersect_outer_circle = ((end_base_distant <= r2 + linklength) && (end_base_distant >= abs(r2 - linklength)));
    bool intersect_inner_circle = ((end_base_distant <= r1 + linklength) && (end_base_distant >= abs(r1 - linklength)));

    // intersect with both inner outer
    if (intersect_outer_circle && intersect_inner_circle){
        // LOG("case 1");
        Eigen::Vector2d outer_intersect = circlesIntersect(getBaseLocation(), r2, end_effector_location, linklength);
        Eigen::Vector2d inner_intersect = circlesIntersect(getBaseLocation(), r1, end_effector_location, linklength);
        double theta = 0;
        Eigen::Rotation2Dd rot;

        double angle_different_1 = abs(outer_intersect[0]-inner_intersect[0]);
        double angle_different_2 = abs(outer_intersect[0]-inner_intersect[1]);

        bool angle_1_changed = false;
        bool angle_2_changed = false;

        if(2*M_PI-angle_different_1 < angle_different_1){
            angle_different_1 = 2*M_PI-angle_different_1;
            angle_1_changed = true;
        }
        if(2*M_PI-angle_different_2 < angle_different_2){
            angle_different_2 = 2*M_PI-angle_different_2;
            angle_2_changed = true;
        }


        if (angle_different_1 <= angle_different_2){
            if (angle_1_changed){
                theta = -M_PI;
            }
            theta += (outer_intersect[0] + inner_intersect[0])/2 + M_PI;
        }else{
            if (angle_2_changed){
                theta = -M_PI;
            }
            theta += (outer_intersect[0] + inner_intersect[1])/2 + M_PI;
        }
        rot = Eigen::Rotation2Dd(theta);
        
        joint_location = end_effector_location - rot*Eigen::Vector2d(linklength,0);
        return joint_location;

    }
    
    // only intersect with outer circle
    if (intersect_outer_circle){
        // LOG("case 2");
        Eigen::Vector2d end2base = (getBaseLocation() - end_effector_location).normalized();
        joint_location = end_effector_location + end2base*linklength;
        return joint_location;
    }

    // only intersect with inner circle
    if (intersect_inner_circle){
        // LOG("case 3");
        Eigen::Vector2d base2end = (end_effector_location-getBaseLocation()).normalized();
        joint_location = end_effector_location + base2end*linklength;
        return joint_location;
    }
    
    // case 1 that not work
    if (end_base_distant+linklength<r1){
        return joint_location;
    }

    // anything works
    if (end_base_distant<r2){
        // LOG("case 4");
        joint_location = end_effector_location - Eigen::Vector2d(linklength,0);
        return joint_location;
    }

    return joint_location;
}

Eigen::Vector2d MyManipulator2D::circlesIntersect(const Eigen::Vector2d& circle1, const double circle1Radius, const Eigen::Vector2d& circle2, const double circle2Radius) const {
    double d = sqrt(pow(circle1[0]-circle2[0],2) + pow(circle1[1]-circle2[1],2));
    double l = (pow(circle1Radius,2)-pow(circle2Radius,2)+pow(d,2))/(2*d);
    double h = sqrt(pow(circle1Radius,2)-pow(l,2));

    Eigen::Vector2d point1;
    point1[0] = (l/d)*(circle2[0]-circle1[0]) + (h/d)*(circle2[1]-circle1[1]) + circle1[0];
    point1[1] = (l/d)*(circle2[1]-circle1[1]) - (h/d)*(circle2[0]-circle1[0]) + circle1[1];
    Eigen::Vector2d point2;
    point2[0] = (l/d)*(circle2[0]-circle1[0]) - (h/d)*(circle2[1]-circle1[1]) + circle1[0];
    point2[1] = (l/d)*(circle2[1]-circle1[1]) + (h/d)*(circle2[0]-circle1[0]) + circle1[1];

    double theta1 = atan2(point1[1]-circle2[1],point1[0]-circle2[0]);
    double theta2 = atan2(point2[1]-circle2[1],point2[0]-circle2[0]);

    return Eigen::Vector2d(theta1,theta2);
}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    if (joint_index == 0) {
        return Eigen::Vector2d(0.0, 0.0); // Base location
    }

    std::vector<Eigen::Transform<double, 2, Eigen::Affine>> transforms;
    transforms.push_back(HomogeneousTransform(state[0], Eigen::Vector2d(0.0, 0.0)));
    for(int i = 1; i < joint_index; i++) {
        transforms.push_back(HomogeneousTransform(state[i], Eigen::Vector2d(getLinkLengths()[i-1], 0.0)));
    }

    Eigen::Vector2d joint_position(getLinkLengths()[joint_index-1], 0.0);
    for(int i = joint_index-1; i >= 0; i--) {
        joint_position = transforms[i] * joint_position;
    }

    return joint_position;
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
    amp::ManipulatorState joint_angles;
    joint_angles.setZero();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        joint_angles.resize(2);
        joint_angles = getConfiguration2link(end_effector_location);
        return joint_angles;
    } 
    else if (nLinks() == 3) {
        joint_angles.resize(3);
        double link2_or = getLinkLengths()[0] + getLinkLengths()[1];
        double link2_ir = abs(getLinkLengths()[0] - getLinkLengths()[1]);

        Eigen::Vector2d joint2_location = possibleJointLocation(link2_ir, link2_or, getLinkLengths()[2], end_effector_location);
        Eigen::VectorXd first2Configuration = getConfiguration2link(joint2_location);
        Eigen::Vector2d joint_2_end = joint2_location - end_effector_location;

        double theta = atan2(-joint_2_end[1],-joint_2_end[0]);

        joint_angles[0] = first2Configuration[0];
        joint_angles[1] = first2Configuration[1];
        joint_angles[2] = theta-first2Configuration[1]-first2Configuration[0];
        goto end_function;
  
    } 
    else {
        joint_angles.resize(nLinks());
        std::vector<double> temp_link_length = getLinkLengths();
        Eigen::Vector2d current_end_effector = end_effector_location;
        std::vector<double> theta;
        double current_final_length;
        int link_left = nLinks();

        while(link_left > 2){
            current_final_length = temp_link_length.back();
            temp_link_length.pop_back(); 
            std::vector<double> reorganize_link_length = temp_link_length;
            sort(reorganize_link_length.begin(), reorganize_link_length.end(), comp);

            double r1 = smallestAbsuluteOfSum(reorganize_link_length);
            double r2 = sum(reorganize_link_length);

            Eigen::Vector2d previous_joint_location = possibleJointLocation(r1, r2, current_final_length, current_end_effector);
            Eigen::Vector2d joint_2_end = previous_joint_location - current_end_effector;

            theta.push_back(atan2(-joint_2_end[1],-joint_2_end[0]));

            current_end_effector = previous_joint_location;
            link_left = link_left - 1;
        }
        Eigen::VectorXd first2Configuration = getConfiguration2link(current_end_effector);

        joint_angles[0] = first2Configuration[0];
        joint_angles[1] = first2Configuration[1];
        for(int i = 2; i < nLinks(); i++){
            joint_angles[i] = theta[nLinks()-1-i] - joint_angles.sum();
            joint_angles[i] = joint_angles[i] - 2*M_PI*floor((joint_angles[i] + M_PI)/(2*M_PI));
        }
        return joint_angles;
    }

    return joint_angles;

    end_function:
    for (int i = 0; i < nLinks(); i++){
        joint_angles[i] = joint_angles[i] - 2*M_PI*floor((joint_angles[i] + M_PI)/(2*M_PI));
    }
    return joint_angles;
}