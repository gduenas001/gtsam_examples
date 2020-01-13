

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
struct Residual {
	double value= 0;
	int num_factors= 0;
};

// -------------------------------------------------------
class Variance {
public:
	Variance() {
		this->value= -1;
	}

	Variance(double variance) {
		this->value= variance;
	};

	double std() {
		return std::sqrt(this->value);
	}
	double value= -1;
};

// -------------------------------------------------------
class Residuals {

public:
	Residuals() {};

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
class Variances {
public:
	Variances() {};
	Variances(const gtsam::Matrix &P_x, 
			  const gtsam::Matrix &P_v, 
			  const gtsam::Matrix &P_b){
	  /*
	  The covariance matrices for:
	  - P_x: position (6x6)
	  - P_v: linear velocity (3x3)
	  - P_b: IMU bias (6x6)
	  */

	  // pose
	  this->roll.value= P_x(0,0);
	  this->pitch= Variance(P_x(1,1));
	  this->yaw= Variance(P_x(2,2));
	  this->x= Variance(P_x(3,3));
	  this->y= Variance(P_x(4,4));
	  this->z= Variance(P_x(5,5));
	  // velocity
	  this->v_x= Variance(P_v(0,0));
	  this->v_y= Variance(P_v(1,1));
	  this->v_z= Variance(P_v(2,2));
	  // IMU biases
	  this->b_accel_x= Variance(P_b(0,0));
	  this->b_accel_y= Variance(P_b(1,1));
	  this->b_accel_z= Variance(P_b(2,2));
	  this->b_gyro_x= Variance(P_b(3,3));
	  this->b_gyro_y= Variance(P_b(4,4));
	  this->b_gyro_z= Variance(P_b(5,5)); 
	};

	Variance roll;
	Variance pitch;
	Variance yaw;
	Variance x;
	Variance y;
	Variance z;
	Variance v_x;
	Variance v_y;
	Variance v_z;
	Variance b_accel_x;
	Variance b_accel_y;
	Variance b_accel_z;
	Variance b_gyro_x;
	Variance b_gyro_y;
	Variance b_gyro_z;
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
			 const std::string &workspace,
			 const LIR &lir);

	// time of the solution
	double time;

	// variance
	Variances variances;

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



