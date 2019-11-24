

#pragma once

#include <gtsam/navigation/Scenario.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <vector>
#include <string>

#include "Counters.h"
#include "RangeBearingFactorMap.h"
#include "RangeBearingMeasurement.h"
#include "helpers.h"



// Shorthand for velocity and pose variables
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;



/*
Add a prior factors in:
- pose (X)
- velocity (V) 
- IMU bias (B)
the priors are sampled from their corresponding
noise distributions is set in the parameters.
*/
int add_prior_factor(
         gtsam::NonlinearFactorGraph &new_graph, 
         gtsam::FixedLagSmoother::KeyTimestampMap &new_timestamps,
         gtsam::Values &initial_estimate,
         const gtsam::Scenario &scenario,
         std::default_random_engine &noise_generator, 
         std::map<string, vector<int>> &A_rows_per_type,
         Counters &counters,
         Params &params);


/*
Add lidar factor to factor graph.
It also keep A_rows_per_type updated.
*/
void add_lidar_factor(
      gtsam::NonlinearFactorGraph &newgraph,
      RangeBearingFactorMap &range_bearing_factor,
      std::map<string, vector<int>> &A_rows_per_type, 
      int &A_rows_count,
      Counters &counters,
      bool is_verbose);



/*
Add GPS factor to factor graph.
It also keep A_rows_per_type updated.
*/
void add_gps_factor(
      gtsam::NonlinearFactorGraph &newgraph,
      GPSFactor &gps_factor,
      std::map<string, vector<int>> &A_rows_per_type, 
      int &A_rows_count,
      Counters &counters,
      bool is_verbose);




/*
Add IMU factor to factor graph.
It also keep A_rows_per_type updated.
*/
void addOdomFactor(
      gtsam::NonlinearFactorGraph &newgraph,
      gtsam::CombinedImuFactor &imufac,
      std::map<string, vector<int>> &A_rows_per_type, 
      int &A_rows_count,
      Counters &counters,
      bool is_verbose);


