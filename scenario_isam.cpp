// TODO 
// - need to figure out how the imu noise is specified: continuous - discrete time conversion.
// - data association for landmarks
// - different frequencies for GPS and lidar
// - substitute boost::optional. I don't think this is the use
// - Why? Odom M matrix is rank 6 because of the actual number of measurements
// - seems that dof of a odom factor is 12, not 6, but M is still rank 6...
// - Change naming convention of functions -> use underscores, not capital letters
// - save_data to support fixed-lag smoother
// - predict the initial estimate for the bias from the previous state (currently using a zero bias as init state)
// - use proto for Params class and read from external .pbtxt file
// - in fixed-lag smoothing, all hypotheses are rank-deficient
// - change results_fl to results
// - add option for the python plot
// - remove random seed and check that r values coincide with dof


#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <typeinfo>
#include <boost/math/distributions/chi_squared.hpp>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>


#include "helpers.h"
#include "post_process.h"
#include "optionsParser.h"
#include "Counters.h"
#include "add_factors.h"

using namespace std;
using namespace gtsam;


/* ************************************************************************* */
int main(int argc, char** argv) {

  // parse the options
  Params params;
  optionsParser(argc, argv, params);

  // build variable from params
  build_variables(params);

  // noise generator
  std::default_random_engine noise_generator;
  noise_generator.seed(params.seed);

  // landmarks
  vector<Point3> landmarks= createLandmarks(params.scenario_radius);

  // scenario to simulate measurements and ground truth
  ConstantTwistScenario scenario= createConstantTwistScenario(
                                      params.scenario_radius,
                                      params.scenario_linear_vel);

  // Create factor graph & timestamps to update ISAM2 fixed-lag smoother
  NonlinearFactorGraph newgraph;
  FixedLagSmoother::KeyTimestampMap new_timestamps;

  // Create the initial estimate to the solution
  Values initial_estimate, result_fl; 
  
  // fixed-lag smoother
  IncrementalFixedLagSmoother fixed_lag_smoother(params.lag, params.fl_isam_params);


  // initialize variables
  Counters counters(params);
  NavState prev_state, predict_state;
  imuBias::ConstantBias prev_bias;
  vector<Point3> true_positions;
  vector<Pose3> online_error; // error when computed online
  true_positions.push_back( scenario.pose(0).translation() );
  FixedLagSmoother::Result isam_result_fl;
  map<string, vector<int>> A_rows_per_type; // stores wich msmts to which hypothesis
  A_rows_per_type.insert( pair<string, vector<int>> ("lidar", {}) );
  A_rows_per_type.insert( pair<string, vector<int>> ("odom", {}) );
  A_rows_per_type.insert( pair<string, vector<int>> ("gps", {}) );

  // add prior factor
  int A_rows_count= add_prior_factor(newgraph,
                                  	new_timestamps,
                            		    initial_estimate, 
                              		  scenario,
                                  	noise_generator,
                              		  A_rows_per_type,
                                  	counters,
                                  	params);

  // solve the graph once
  fixed_lag_smoother.update(newgraph, initial_estimate, new_timestamps);
  result_fl= fixed_lag_smoother.calculateEstimate();
  newgraph= NonlinearFactorGraph();
  initial_estimate.clear();
  new_timestamps.clear();

  // Simulate poses and imu measurements, adding them to the factor graph
  while (counters.current_time < params.sim_time){
    counters.increase_time();
      
    // Simulate acceleration and gyro measurements in (actual) body frame
    Vector3 msmt_acc= sim_imu_acc(scenario,
                                  noise_generator,
                                  params.noise_dist["acc"],
                                  params.imu_params->n_gravity,
                                  counters.current_time,
                                  params.is_noisy["imu"]);

    Vector3 msmt_w= sim_imu_w(scenario.omega_b(counters.current_time),
                              noise_generator,
                              params.noise_dist["gyro"],
                              params.is_noisy["imu"]);

    // Preintegrate IMU msmts
    params.accum.integrateMeasurement(msmt_acc, 
                                      msmt_w, 
                                      params.dt_imu);

    // GPS update
    if (counters.gps_time_accum > params.dt_gps) {

      counters.increase_factors_count();
      new_timestamps[X(counters.current_factor)]= counters.current_time;
      new_timestamps[V(counters.current_factor)]= counters.current_time;
      new_timestamps[B(counters.current_factor)]= counters.current_time;

      // save the current position
      true_positions.push_back( scenario.pose(counters.current_time).translation() );

      // predict from IMU accumulated msmts
      prev_state= NavState(result_fl.at<Pose3>  (X(counters.prev_factor)), 
                           result_fl.at<Vector3>(V(counters.prev_factor)));
      prev_bias= result_fl.at<imuBias::ConstantBias>(B(counters.prev_factor));
      predict_state= params.accum.predict(prev_state, prev_bias);

      // predicted init values
      initial_estimate.insert(X(counters.current_factor), predict_state.pose());
      initial_estimate.insert(V(counters.current_factor), predict_state.velocity());
      initial_estimate.insert(B(counters.current_factor), imuBias::ConstantBias());  

      // Add Imu Factor
      CombinedImuFactor imu_factor(X(counters.prev_factor),    V(counters.prev_factor), 
                                   X(counters.current_factor), V(counters.current_factor), 
                                   B(counters.prev_factor),    B(counters.current_factor), 
                                   params.accum);

      addOdomFactor(newgraph,
                    imu_factor,
                    A_rows_per_type, 
                    A_rows_count,
                    counters);

   
      // Adding GPS factor
      Point3 gps_msmt= sim_gps_msmt(
                         scenario.pose(counters.current_time).translation(),
                         noise_generator, 
                         params.noise_dist["range"],
                         params.is_noisy["gps"] );

      GPSFactor gps_factor(X(counters.current_factor), 
                          gps_msmt, 
                          params.gps_cov);

      add_gps_factor(newgraph,
                   gps_factor,
                   A_rows_per_type,
                   A_rows_count,
                   counters);

      // lidar measurements
      for (int j = 0; j < landmarks.size(); ++j) {
        
        RangeBearingMeasurement 
        range_bearing_msmt= sim_lidar_msmt(scenario,
                                           landmarks[j],
                                           counters.current_time,
                                           params,
                                           noise_generator,
                                           params.is_noisy["lidar"]);

        // range-bearing factor
        RangeBearingFactorMap 
        range_bearing_factor(X(counters.current_factor), 
                             range_bearing_msmt.range,
                             range_bearing_msmt.bearing,
                             landmarks[j], 
                             params.lidar_cov);

        add_lidar_factor(newgraph,
                       range_bearing_factor,
                       A_rows_per_type, 
                       A_rows_count,
                       counters);
      }      
      
      // Incremental solution
      isam_result_fl= fixed_lag_smoother.update(newgraph, 
                                                initial_estimate, 
                                                new_timestamps);
      for (int i = 0; i < 3; ++i) {
        isam_result_fl= fixed_lag_smoother.update();
      }

      result_fl= fixed_lag_smoother.calculateEstimate();

      // compute error
      online_error.push_back(compute_error(scenario.pose(counters.current_time),
                            result_fl.at<Pose3>(X(counters.current_factor)) ));

      // if there's been marginalization -> add factor
      if (counters.current_time  > params.lag){
        counters.add_factor("marginalized_prior");
      }

	    // reset variables
      counters.update_A_rows(params.lag);
      newgraph= NonlinearFactorGraph();
      params.accum.resetIntegration();
      initial_estimate.clear();
      new_timestamps.clear();
      counters.reset_timer();
    }
  } // end for loop

  // save the data TODO: give option to save in different folder
  save_data(result_fl,
           true_positions,
           landmarks,
           online_error);
  
  // post process data showing each hypothesis
  post_process(result_fl,
               isam_result_fl,
               fixed_lag_smoother,
               A_rows_per_type,
               counters,
               params);

  return 0;
}
