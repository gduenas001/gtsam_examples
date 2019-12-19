
#include "add_factors.h"

using namespace std;
using namespace gtsam;

// -------------------------------------------------------
// -------------------------------------------------------
void add_prior_factor(NonlinearFactorGraph &new_graph, 
                     FixedLagSmoother::KeyTimestampMap &new_timestamps,
                     Values &initial_estimate,
                     const Scenario &scenario,
                     default_random_engine &noise_generator, 
                     Counters &counters,
                     Params &params){

  if (params.is_verbose) {cout<< "Adding prior factors..."<< '\n';}

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr
  pose_noise_model= 
  noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.01), 
                                Vector3::Constant(0.1) ).finished() );

  // simulate a prior pose measurement
  Rot3 prior_orientation_msmt= scenario.pose(0).rotation();
  Point3 prior_position_msmt= scenario.pose(0).translation();
  if (params.is_noisy["prior"]){
    Point3 prior_orientation_noise= 
    generate_random_point(noise_generator, 
                          params.noise_dist["prior_orientation"]);
    prior_orientation_msmt.RzRyRx(prior_orientation_noise);

    Point3 prior_position_noise= 
    generate_random_point(noise_generator, 
                          params.noise_dist["prior_position"]);
    prior_position_msmt= prior_position_msmt + prior_position_noise;
    
  }
  Pose3 prior_pose_msmt= Pose3(prior_orientation_msmt,
                               prior_position_msmt);

  // add prior pose factor
  PriorFactor<Pose3> pose_prior(X(0), 
                                prior_pose_msmt,
                                params.prior_pose_cov);
  new_graph.add(pose_prior);
  initial_estimate.insert(X(0), prior_pose_msmt);
  new_timestamps[X(0)]= counters.current_time;
  
  // new way of keeping track
  vector<int> rows= returnIncrVector(0,6);
  for (int i = 0; i < rows.size(); ++i){
    counters.A_rows["prior_pose"].push_back(
            make_pair(rows[i], counters.current_time));
  }
  counters.num_A_rows += 6;

  // keep track of the factor types inserted
  counters.add_factor("prior_pose");
  
  // add velocity prior to graph and init values
  Vector prior_vel_msmt(3); // needs to be a dynamically allocated vector (I don't know why)
  prior_vel_msmt= scenario.velocity_n(0);
  if (params.is_noisy["prior"]){
    Point3 prior_vel_noise= generate_random_point(noise_generator,
                                    params.noise_dist["prior_vel"]);
    prior_vel_msmt= prior_vel_msmt + prior_vel_noise;
  }

  PriorFactor<Vector> vel_prior_factor(V(0), 
                                       prior_vel_msmt, 
                                       params.prior_vel_cov);
  new_graph.add(vel_prior_factor);
  initial_estimate.insert(V(0), prior_vel_msmt);
  new_timestamps[V(0)]= counters.current_time;

  // new way of keeping track
  rows= returnIncrVector(counters.num_A_rows, 3);
  for (int i = 0; i < rows.size(); ++i){
    counters.A_rows["prior_vel"].push_back(
            make_pair(rows[i], counters.current_time));
  }
  counters.num_A_rows += 3;

  // keep track of the factor types inserted
  counters.add_factor("prior_vel");

  // Add bias priors to graph and init values
  imuBias::ConstantBias prior_bias_msmt= imuBias::ConstantBias();
  if (params.is_noisy["prior"]){
    Point3 prior_bias_acc_noise= 
    generate_random_point(noise_generator,
                          params.noise_dist["prior_bias_acc"]);
    Point3 prior_bias_gyro_noise= 
    generate_random_point(noise_generator,
                          params.noise_dist["prior_bias_gyro"]);

    imuBias::ConstantBias prior_bias_noise= 
    imuBias::ConstantBias(prior_bias_acc_noise.vector(), 
                          prior_bias_gyro_noise.vector() );

    prior_bias_msmt= prior_bias_msmt + prior_bias_noise;
  }

  PriorFactor<imuBias::ConstantBias> 
  prior_bias_factor(B(0), 
                    prior_bias_msmt,
                    params.prior_bias_cov);
  new_graph.add(prior_bias_factor);
  initial_estimate.insert(B(0), prior_bias_msmt);
  new_timestamps[B(0)]= counters.current_time;

  // new way of keeping track
  rows= returnIncrVector(counters.num_A_rows, 6);
  for (int i = 0; i < rows.size(); ++i){
    counters.A_rows["prior_bias"].push_back(
            make_pair(rows[i], counters.current_time));
  }
  counters.num_A_rows += 6;

  // keep track of the factor types inserted
  counters.add_factor("prior_bias");
  
  if (params.is_verbose) {cout<< "...prior factor added"<< '\n';}

  return;
}


// -------------------------------------------------------
// -------------------------------------------------------
void add_lidar_factor(NonlinearFactorGraph &newgraph,
					RangeBearingFactorMap &range_bearing_factor,
					Counters &counters,
          bool is_verbose){

  if (is_verbose) {cout<< "Adding lidar factor..."<< '\n';}

  newgraph.add(range_bearing_factor);

  // new way of keeping track
  vector<int> rows= returnIncrVector(counters.num_A_rows, 3);
  for (int i = 0; i < rows.size(); ++i){
    counters.A_rows["lidar"].push_back(
            make_pair(rows[i], counters.current_time));
  }
  counters.num_A_rows += 3;

  // keep track of the factor types inserted
  counters.add_factor("lidar");

  if (is_verbose) {cout<< "...lidar factor added"<< '\n';}
}


// -------------------------------------------------------
// -------------------------------------------------------
void add_gps_factor(NonlinearFactorGraph &newgraph,
			   GPSFactor &gps_factor,
         Counters &counters,
         bool is_verbose) {

  if (is_verbose) {cout<< "Adding GPS factor..."<< '\n';}

	newgraph.add(gps_factor);

  // new way of keeping track
  vector<int> rows= returnIncrVector(counters.num_A_rows, 3);
  for (int i = 0; i < rows.size(); ++i){
    counters.A_rows["gps"].push_back(
            make_pair(rows[i], counters.current_time));
  }
  counters.num_A_rows += 3;

  // keep track of the factor types inserted
  counters.add_factor("gps");
  
  if (is_verbose) {cout<< "...GPS factor added"<< '\n';}
}


// -------------------------------------------------------
// -------------------------------------------------------
void add_imu_factor(NonlinearFactorGraph &newgraph,
		    CombinedImuFactor &imufac,
        Counters &counters,
        bool is_verbose) {

  if (is_verbose) {cout<< "Adding imu factor"<< '\n';}

	newgraph.add(imufac);

   // new way of keeping track
  vector<int> rows= returnIncrVector(counters.num_A_rows, 15);
  for (int i = 0; i < rows.size(); ++i){
    counters.A_rows["odom"].push_back(
            make_pair(rows[i], counters.current_time));
  }
  counters.num_A_rows += 15;

  // keep track of the factor types inserted
  counters.add_factor("odom");
  
  if (is_verbose) {cout<< "...imu factor added"<< '\n';}
}









