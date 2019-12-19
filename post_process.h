
#pragma once

#include "helpers.h"
#include "Counters.h"
#include "Params.h"


/*
Post-processing function that extract the factor graph from
the iSAM2 solver and calcualtes the Jacobian A to do all 
the calculations. It loops over the hypotheses to compute the 
LIR for x, y, and z coordinates in inertial frame.

NOTE: This function is work in progress and is messy.
*/
void post_process(
	gtsam::Values result_fl,
    gtsam::FixedLagSmoother::Result isam_result_fl,
    gtsam::IncrementalFixedLagSmoother &fixed_lag_smoother,
  	Counters &counters,
	Params &params);
