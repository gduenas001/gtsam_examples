
#pragma once

#include "optionsParser.h"

class Counters{
public:
	Counters(Params const &params){
		this->current_time= 0;
		this->gps_time_accum= 0;
		// this->num_imu_epochs= params.sim_time / params.dt_imu;
		this->current_factor= 0;
		this->prev_factor= -1;

		this->dt_imu= params.dt_imu;
		this->dt_gps= params.dt_gps;
	};

	double current_time;
	double gps_time_accum;
	// double num_imu_epochs;
	double dt_imu;
	double dt_gps;

	int current_factor;
	int prev_factor;

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


};



