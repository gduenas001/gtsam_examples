

#pragma once

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/Scenario.h>
#include <fstream>


#include "Counters.h"
#include "LIR.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;




// -------------------------------------------------------
struct Residual{
	double value= 0;
	int num_factors= 0;
};


// -------------------------------------------------------
class Residuals{

public:
	Residuals(){};

	Residual odom;
	Residual gps;
	Residual lidar;
	Residual prior_pose;
	Residual prior_vel;
	Residual prior_bias;
	Residual marginalized_prior;
	Residual sum;
};


// -------------------------------------------------------
class Solution{
	/*
	Object that stores one solution of the solver.
	It can also print the solution and write it to a file
	*/

public:
	Solution(const gtsam::IncrementalFixedLagSmoother &fixed_lag_smoother,
			 const gtsam::Values &result, 
			 const Counters &counters,
			 const ConstantTwistScenario &scenario,
			 const std::string &workspace);

	// time of the solution
	double time;

	// estimate state (15dof)
	gtsam::NavState nav_state;
	gtsam::imuBias::ConstantBias imu_bias;

	// true state (15dof)
	gtsam::NavState true_nav_state;

	// error (15 dof)
	Eigen::Matrix<double, 15, 1> error;

	// residuals
	Residuals residuals;

	// LIR
	LIR lir;

	/*
	writes the solution to this file
	*/
	bool write_to_file(const std::string &workspace);

};



