#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <Eigen/Geometry>

// Include the correct homework header
#include "hw/HW4.h"

// Derive the amp::LinkManipulator2D class
class MyManipulator2D : public amp::LinkManipulator2D {
    public:
        // Default constructor
        MyManipulator2D();
        MyManipulator2D(const std::vector<double>& link_lengths);

        Eigen::Transform<double, 2, Eigen::Affine> HomogeneousTransform(double theta,const Eigen::Vector2d& translation) const;
        amp::ManipulatorState getConfiguration2link(const Eigen::Vector2d& end_effector_location) const;
        Eigen::Vector2d circlesIntersect(const Eigen::Vector2d& circle1, const double circle1Radius, const Eigen::Vector2d& circle2, const double circle2Radius) const;

        // Override this method for implementing forward kinematics
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        // Override this method for implementing inverse kinematics
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
};