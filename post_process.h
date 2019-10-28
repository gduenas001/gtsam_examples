
#pragma once

#include "helpers.h"
#include "Counters.h"
#include "optionsParser.h"


/*
Post-processing function that extract the factor graph from
the iSAM2 solver and calcualtes the Jacobian A to do all 
the calculations. It loops over the hypotheses to compute the 
LIR for x, y, and z coordinates in inertial frame.

NOTE: This function is work in progress and is messy.
*/
void post_process(
	gtsam::Values result,
    gtsam::ISAM2Result isam_result,
    gtsam::ISAM2 isam,
	std::map<std::string, std::vector<int>> A_rows_per_type,
  	Counters &counters,
	Params &params);