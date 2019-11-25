
#pragma once

#include "Params.h"

using namespace std;
using namespace gtsam;

typedef std::vector< std::pair<int, double> > pair_vector;

class Counters{
	/*
	Counters class that contains variables related to time
	number of elements. It also keeps track of the added and
	removed rows in the Jacobian A.
	*/

public:
	Counters(Params const &params){
		this->current_time= 0;
		this->gps_time_accum= 0;
		// this->num_imu_epochs= params.sim_time / params.dt_imu;
		this->current_factor= 0;
		this->prev_factor= -1;

		this->dt_imu= params.dt_imu;
		this->dt_gps= params.dt_gps;

		// initialize A_rows with all msmt types
		pair_vector empty_pair_vector;
		this->A_rows.insert({"prior_pose", empty_pair_vector});
		this->A_rows.insert({"prior_vel", empty_pair_vector});
		this->A_rows.insert({"prior_bias", empty_pair_vector});
		this->A_rows.insert({"lidar", empty_pair_vector});
		this->A_rows.insert({"odom", empty_pair_vector});
		this->A_rows.insert({"gps", empty_pair_vector});

		this->num_A_rows= 0;

		// initialize factor types vector
		std::vector<std::string> temp_vec;
		this->types= temp_vec;
	};

	double current_time;
	double gps_time_accum;
	// double num_imu_epochs;
	double dt_imu;
	double dt_gps;

	int current_factor;
	int prev_factor;

	// to keep track of the msmts in the Jacobian (A)
	std::map<std::string, pair_vector> A_rows;
	int num_A_rows;

	// to keep track of factor types
	std::vector<std::string> types;


	/*
	increase time by dt_imu
	*/
	void increase_time(){
		this->current_time   += this->dt_imu;
		this->gps_time_accum += this->dt_imu;
	}


	/*
	set the accumulation time to zero
	This is done after the update
	*/
	void reset_timer(){
		this->gps_time_accum= 0;
	}

	/*
	increase the number of factor by one
	*/
	void increase_factors_count(){
		this->current_factor += 1;
		this->prev_factor += 1;
	}


	/*
	add an estry with the name of the factor to 
	"types" field
	*/
	void add_factor(std::string type);


	/*
	removes rows corresponding to factors that were
	factorized
	*/
	void update_A_rows(const double lag,
					   const bool is_verbose);
};



