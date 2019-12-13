

#pragma once

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/Scenario.h>


#include "Counters.h"

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
	Residual sum;
};



// -------------------------------------------------------
class Solution{

public:
	Solution(const gtsam::IncrementalFixedLagSmoother &fixed_lag_smoother,
			 const gtsam::Values &result, 
			 const Counters &counters,
			 const ConstantTwistScenario &scenario);

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

};



