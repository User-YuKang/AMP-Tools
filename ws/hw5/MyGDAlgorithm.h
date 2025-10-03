#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "tools/Usefull.h"
#include <cmath>
#include <chrono>
#include <time.h>

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
	private:
		double d_star, zetta, Q_star, eta;
		int cells_per_unit;
		bool checkTimeout(const std::chrono::time_point<std::chrono::high_resolution_clock> t1, double time_allow);
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		MyPotentialFunction(double d_star, double zetta, double Q_star, double eta, int cells_per_unit, const amp::Problem2D& problem);
			
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
            // return q[0] * q[0] + q[1] * q[1];
			return attraction(q) + repulsive(q);
			// return attraction(q);
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
            // return Eigen::Vector2d(q[0] * q[0],  q[1] * q[1]);
            // return Eigen::Vector2d(q[0],  1);
			return attractionGradient(q) + repulsiveGradient(q);
			// return repulsiveGradient(q);
			// return attractionGradient(q);
        }
		amp::DenseArray2D<int> brushFireAll();

	private:
		double d_star, zetta, Q_star, eta;
		int cells_per_unit;
		const amp::Problem2D& problem;
		std::vector<amp::DenseArray2D<int>> distant_arrays;
		std::vector<amp::DenseArray2D<std::vector<int>>> gradient_arrays;

		std::vector<amp::DenseArray2D<int>> brushFireIdividual();

		double attraction(const Eigen::Vector2d& q) const;
		Eigen::Vector2d attractionGradient(const Eigen::Vector2d& q) const;
		
		double repulsive(const Eigen::Vector2d& q) const;
		Eigen::Vector2d repulsiveGradient(const Eigen::Vector2d& q) const;
};