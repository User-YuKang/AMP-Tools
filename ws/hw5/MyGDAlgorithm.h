#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "tools/Usefull.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta, int cells_per_unit) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta),
			cells_per_unit(cells_per_unit) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
		amp::DenseArray2D<int> brush_fire_distance(const amp::Problem2D& problem);
	private:
		double d_star, zetta, Q_star, eta;
		int cells_per_unit;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
            return q[0] * q[0] + q[1] * q[1];
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
            return Eigen::Vector2d(q[0] * q[0],  q[1] * q[1]);
        }
};