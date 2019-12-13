

#include "Solution.h"

using namespace std;
using namespace gtsam;


// -------------------------------------------------------
Solution::Solution(const gtsam::IncrementalFixedLagSmoother &fixed_lag_smoother,
				   const gtsam::Values &result, 
				   const Counters &counters,
				   const ConstantTwistScenario &scenario,
				   const string &workspace){

	// time
	this->time= counters.current_time;

	// estimate state
	this->nav_state= NavState(
					    result.at<Pose3>  (X(counters.current_factor)), 
                        result.at<Vector3>(V(counters.current_factor)));
	this->imu_bias= result.at<imuBias::ConstantBias>(B(counters.current_factor));

	// true state
	this->true_nav_state= NavState(
					scenario.pose(counters.current_time),
					scenario.velocity_n(counters.current_time) );
	
	// error (15dof)
	this->error.segment<3>(0)= 
			abs(nav_state.pose().translation() - true_nav_state.pose().translation());
	this->error.segment<3>(3)= 
			abs(nav_state.pose().rotation().rpy() - true_nav_state.pose().rotation().rpy());
	this->error.segment<3>(6)= 
			abs(nav_state.v() - true_nav_state.v());
	this->error.segment<6>(9)= 
			abs(this->imu_bias.vector());

	// get the factor graph & Jacobian from isam
	NonlinearFactorGraph factor_graph= fixed_lag_smoother.getFactors();

	// save the sum of residuals per type
	int factor_count= -1;
	for (auto factor : factor_graph){
		++factor_count;
        if (!factor) {continue;}

        double factor_error= 2 * factor->error(result);
        double factor_dim= factor->dim();
      
		if (counters.types[factor_count] == "odom"){
			this->residuals.odom.value += factor_error;
			this->residuals.odom.num_factors++;
		} else if (counters.types[factor_count] == "gps"){
			this->residuals.gps.value += factor_error;
			this->residuals.gps.num_factors++;
		} else if (counters.types[factor_count] == "lidar"){
			this->residuals.lidar.value += factor_error;
			this->residuals.lidar.num_factors++;
		}


		// TODO: check that this is the same as the error for the graph
		this->residuals.sum.value += factor_error;
		this->residuals.sum.num_factors++;

		// // print the factor residual info
		// cout<< "factor # "<< factor_count<< "\t"
		// 	<< "type: "<< counters.types[factor_count]<< "\t\t"
		// 	<< "dim: "<< factor_dim<< "\t"
		// 	<< "error: "<< factor_error<< "\t";
		// factor->printKeys();
    }

    // write solution to file
    this->write_to_file(workspace);
}


// -------------------------------------------------------
bool Solution::write_to_file(const string &workspace){

	// initialize stream
	fstream stream;
	string filename= "";

	// write estimated state (15dof) to a file
	filename= workspace + "/estimate_state.csv";
	stream.open(filename.c_str(), fstream::app);
	stream << this->time << "," 
		   << this->nav_state.pose().x() << "," 
    	   << this->nav_state.pose().y() << "," 
    	   << this->nav_state.pose().z() << "," 
    	   << this->nav_state.pose().rotation().roll() << "," 
    	   << this->nav_state.pose().rotation().pitch() << "," 
    	   << this->nav_state.pose().rotation().yaw() << "," 
    	   << Point3(this->imu_bias.accelerometer()).x() << "," 
    	   << Point3(this->imu_bias.accelerometer()).y() << "," 
    	   << Point3(this->imu_bias.accelerometer()).z() << "," 
    	   << Point3(this->imu_bias.gyroscope()).x() << "," 
    	   << Point3(this->imu_bias.gyroscope()).y() << "," 
    	   << Point3(this->imu_bias.gyroscope()).z() << "," 
    	   << endl;
	stream.close();


	return true;
}











