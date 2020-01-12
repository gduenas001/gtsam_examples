
#include "helpers.h"
#include "Solution.h"

using namespace std;
using namespace gtsam;


// -------------------------------------------------------
Solution::Solution(const gtsam::IncrementalFixedLagSmoother &fixed_lag_smoother,
				   const gtsam::Values &result, 
				   const Counters &counters,
				   const ConstantTwistScenario &scenario,
				   const string &workspace,
				   const LIR &lir){

	// time
	this->time= counters.current_time;

	// copy LIR
	this->lir= lir;

	// get current variance
	this->variance= get_variances_for_last_pose(
							fixed_lag_smoother,
      			            counters);
	this->variances_new= get_variances_for_last_pose_new(
							fixed_lag_smoother,
      			            counters);
	
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
		    (nav_state.pose().rotation() * Rot3(true_nav_state.pose().rotation().transpose())).rpy();
	this->error.segment<3>(3)= 
			nav_state.pose().translation() - true_nav_state.pose().translation();
	this->error.segment<3>(6)= 
			nav_state.bodyVelocity() - true_nav_state.bodyVelocity();
	this->error.segment<6>(9)= 
			this->imu_bias.vector();

	// get the factor graph & Jacobian from isam
	NonlinearFactorGraph 
	factor_graph= fixed_lag_smoother.getFactors();

	// save the sum of residuals per type
	int factor_count= -1;
	for (auto factor : factor_graph){
		++factor_count;
        if (!factor) {continue;}

        double factor_error= 2 * factor->error(result);
        double factor_dim= factor->dim();
      
		if (counters.types[factor_count] == "odom"){
			this->residuals.odom.value += factor_error;
			++this->residuals.odom.num_factors;
		} else if (counters.types[factor_count] == "gps"){
			this->residuals.gps.value += factor_error;
			++this->residuals.gps.num_factors;
		} else if (counters.types[factor_count] == "lidar"){
			this->residuals.lidar.value += factor_error;
			++this->residuals.lidar.num_factors;
		} else if (counters.types[factor_count] == "prior_pose"){
			this->residuals.prior_pose.value += factor_error;
			++this->residuals.prior_pose.num_factors;
		} else if (counters.types[factor_count] == "prior_vel"){
			this->residuals.prior_vel.value += factor_error;
			++this->residuals.prior_vel.num_factors;
		} else if (counters.types[factor_count] == "prior_bias"){
			this->residuals.prior_bias.value += factor_error;
			++this->residuals.prior_bias.num_factors;
		} else if (counters.types[factor_count] == "marginalized_prior"){
			this->residuals.marginalized_prior.value += factor_error;
			++this->residuals.marginalized_prior.num_factors;
		} else {
			cout<< "Type " + counters.types[factor_count] + " not added"<< endl;
		}

		// // print the factor info
		// cout<< "factor # "<< factor_count<< "\t"
		//     << "type: "<< counters.types[factor_count]<< "\t\t"
		//     << "dim: "<< factor_dim<< "\t"
		//     << "error: "<< factor_error<< "\t";
		//     factor->printKeys();

		// TODO: check that this is the same as the error for the graph
		this->residuals.sum.value += factor_error;
		++this->residuals.sum.num_factors;

    }

    // write solution to file
    this->write_to_file(workspace);
}


// -------------------------------------------------------
bool Solution::write_to_file(const string &workspace){

	// initialize stream
	fstream stream;
	string filename= "";

	// write time + estimated state (15dof) to a file
	filename= workspace + "/estimated_states.csv";
	stream.open(filename.c_str(), fstream::app);
	stream << this->time << "," 
    	   << this->nav_state.pose().rotation().roll() << "," 
    	   << this->nav_state.pose().rotation().pitch() << "," 
    	   << this->nav_state.pose().rotation().yaw() << "," 
    	   << this->nav_state.pose().x() << "," 
    	   << this->nav_state.pose().y() << "," 
    	   << this->nav_state.pose().z() << "," 
    	   << this->nav_state.bodyVelocity()[0] << ","
    	   << this->nav_state.bodyVelocity()[1] << ","
    	   << this->nav_state.bodyVelocity()[2] << ","
    	   << Point3(this->imu_bias.accelerometer()).x() << "," 
    	   << Point3(this->imu_bias.accelerometer()).y() << "," 
    	   << Point3(this->imu_bias.accelerometer()).z() << "," 
    	   << Point3(this->imu_bias.gyroscope()).x() << "," 
    	   << Point3(this->imu_bias.gyroscope()).y() << "," 
    	   << Point3(this->imu_bias.gyroscope()).z()
    	   << endl;
	stream.close();
	
	// write time + true state (15dof) to a file
	filename= workspace + "/true_states.csv";
	stream.open(filename.c_str(), fstream::app);
	stream << this->time << "," 
    	   << this->true_nav_state.pose().rotation().roll() << "," 
    	   << this->true_nav_state.pose().rotation().pitch() << "," 
    	   << this->true_nav_state.pose().rotation().yaw() << "," 
    	   << this->true_nav_state.pose().x() << "," 
    	   << this->true_nav_state.pose().y() << "," 
    	   << this->true_nav_state.pose().z() << "," 
    	   << this->true_nav_state.bodyVelocity()[0] << ","
    	   << this->true_nav_state.bodyVelocity()[1] << ","
    	   << this->true_nav_state.bodyVelocity()[2]
    	   << endl;
	stream.close();
	
	// write time + residuals to a file
	filename= workspace + "/variance.csv";
	stream.open(filename.c_str(), fstream::app);
	stream << this->time << "," 
		   << this->variance["roll"] << "," 
    	   << this->variance["pitch"] << "," 
    	   << this->variance["yaw"] << "," 
    	   << this->variance["x"] << "," 
    	   << this->variance["y"] << "," 
    	   << this->variance["z"] << "," 
    	   << this->variance["vx"] << "," 
    	   << this->variance["vy"] << "," 
    	   << this->variance["vz"] << "," 
    	   << this->variance["b_accel_x"] << "," 
    	   << this->variance["b_accel_y"] << "," 
    	   << this->variance["b_accel_z"] << "," 
    	   << this->variance["b_gyro_x"] << "," 
    	   << this->variance["b_gyro_y"] << "," 
    	   << this->variance["b_gyro_z"]
    	   << endl;
	stream.close();

	// write time + residuals to a file
	filename= workspace + "/variances_new.csv";
	stream.open(filename.c_str(), fstream::app);
	stream << this->time << "," 
		   << this->variances_new.roll.value << "," 
    	   << this->variances_new.pitch.value << "," 
    	   << this->variances_new.yaw.value << "," 
    	   << this->variances_new.x.value << "," 
    	   << this->variances_new.y.value << "," 
    	   << this->variances_new.z.value << "," 
    	   << this->variances_new.v_x.value << "," 
    	   << this->variances_new.v_y.value << "," 
    	   << this->variances_new.v_z.value << "," 
    	   << this->variances_new.b_accel_x.value << "," 
    	   << this->variances_new.b_accel_y.value << "," 
    	   << this->variances_new.b_accel_z.value << "," 
    	   << this->variances_new.b_gyro_x.value << "," 
    	   << this->variances_new.b_gyro_y.value << "," 
    	   << this->variances_new.b_gyro_z.value
    	   << endl;
	stream.close();


	// write time + residuals to a file
	filename= workspace + "/residuals.csv";
	stream.open(filename.c_str(), fstream::app);
	stream << this->time << "," 
		   << this->residuals.odom.value << "," 
    	   << this->residuals.gps.value << "," 
    	   << this->residuals.lidar.value << "," 
    	   << this->residuals.prior_pose.value << ","
    	   << this->residuals.prior_vel.value << ","
    	   << this->residuals.prior_bias.value << ","
    	   << this->residuals.marginalized_prior.value << "," 
    	   << this->residuals.sum.value << "," 
    	   << this->residuals.odom.num_factors << "," 
    	   << this->residuals.gps.num_factors << "," 
    	   << this->residuals.lidar.num_factors << "," 
    	   << this->residuals.prior_pose.num_factors << "," 
    	   << this->residuals.prior_vel.num_factors << "," 
    	   << this->residuals.marginalized_prior.num_factors << "," 
    	   << this->residuals.sum.num_factors
    	   << endl;
	stream.close();
	
	// write time + error (15dof) to a file
	filename= workspace + "/errors.csv";
	stream.open(filename.c_str(), fstream::app);
	stream<< this->time;
	for (int i = 0; i < 15; ++i) {
		stream<< ", "<< this->error[i];
	}
	stream << '\n';
	stream.close();
	
	// write time + LIR_x + LIR_y + LIR_z to a file
	filename= workspace + "/lir.csv";
	stream.open(filename.c_str(), fstream::app);
	stream<< this->time << ",";
	stream // null
		  << this->lir.null.x << ","
		  << this->lir.null.y << ","
		  << this->lir.null.z << ","
		   // lidar
		  << this->lir.lidar.x << ","
		  << this->lir.lidar.y << ","
		  << this->lir.lidar.z << ","
		   // gps
		  << this->lir.gps.x << ","
		  << this->lir.gps.y << ","
		  << this->lir.gps.z;
		  
	stream<< '\n';
	stream.close();
	

	return true;
}











