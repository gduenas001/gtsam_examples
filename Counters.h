
#pragma once

#include "optionsParser.h"


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

	// ----------------------------------------------------
	void increase_time(){
		this->current_time   += this->dt_imu;
		this->gps_time_accum += this->dt_imu;
	}

	// ----------------------------------------------------
	void reset_timer(){
		this->gps_time_accum= 0;
	}

	// ----------------------------------------------------
	void increase_factors_count(){
		this->current_factor += 1;
		this->prev_factor += 1;
	}

	// ----------------------------------------------------
	void update_A_rows(double lag){
		// if there has not being marginalization -> return
		if (this->current_time <= lag){ 
			return; 
		}

		// time at the beggining of the sliding window
		double time_threshold= this->current_time - lag;

		// loop through types
		for (map<string, pair_vector>::iterator
	  					it_types= this->A_rows.begin(); 
  						it_types != this->A_rows.end(); 
  						++it_types){

			// type of the factor
			string type= it_types->first;

			// vector with: (A_rows & time)
			pair_vector &row_time= this->A_rows[type];

			// for each row of this type included
			pair_vector::iterator it_rows= row_time.begin();
			while ( it_rows <= row_time.end() ) {
				double msmt_time= it_rows->second;

				// remove all factors before the sw
				if (msmt_time < time_threshold){
					it_rows= row_time.erase(it_rows);
				// XXXXXXXXXXXXXXXXXXXXXXXXXXXX
				// SOME SEGMENTATION FAULT HERE
				// XXXXXXXXXXXXXXXXXXXXXXXXXXXX
				// remove the last odom factor
				}else if (type == "odom" && 
						  msmt_time < time_threshold - this->dt_gps){
					it_rows= row_time.erase(it_rows);
				}else{
					++it_rows;
				}
			}
		}
	}
};



