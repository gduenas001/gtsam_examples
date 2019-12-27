
#pragma once

#include "helpers.h"
#include "Counters.h"
#include "Params.h"
#include "LIR.h"


/*
*/
LIR calculate_LIR(
	const gtsam::Values result,
    const gtsam::IncrementalFixedLagSmoother &fixed_lag_smoother,
  	Counters &counters,
	const Params &params);
