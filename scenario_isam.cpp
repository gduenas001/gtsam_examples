// TODO 
// - need to figure out how the imu noise is specified: continuous - discrete time conversion.
// - data association for landmarks
// - different frequencies for GPS and lidar
// - use sliding window filter
// - Why? Odom M matrix is rank 6 because of the actual number of measurements
// - substitute boost::optional. I don't think this is the use
// - check quantily vs complement in the inv cdf function
// - seems that dof of a odom factor is 12, not 6, but M is still rank 6...


#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <typeinfo>
#include <boost/math/distributions/chi_squared.hpp>

#include "helpers.h"
#include "postProcess.h"
#include "optionsParser.h"
#include "Counters.h"

using namespace std;
using namespace gtsam;


/* ************************************************************************* */
int main(int argc, char** argv) {

  // parse the options
  Params params;
  optionsParser(argc, argv, params);

  // build variable from params
  build_variables(params);

  std::default_random_engine noise_generator;    // noise generator
  vector<Point3> landmarks = createLandmarks(params.scenario_radius);

  // scenario to simulate measurements and ground truth
  ConstantTwistScenario scenario = createConstantTwistScenario(
                                      params.scenario_radius,
                                      params.scenario_linear_vel);

  // Create a factor graph &  ISAM2
  NonlinearFactorGraph newgraph;
  ISAM2 isam(params.isam_params);
  Values initialEstimate, result; // Create the initial estimate to the solution

  // initialize variables
  Counters counters(params);
  NavState prev_state, predict_state;
  imuBias::ConstantBias prev_bias;
  vector<Point3> true_positions;
  vector<Pose3> online_error; // error when computed online
  true_positions.push_back( scenario.pose(0).translation() );
  ISAM2Result isam_result;
  map<string, vector<int>> A_rows_per_type; // stores wich msmts to which hypothesis
  A_rows_per_type.insert( pair<string, vector<int>> ("lidar", {}) );
  A_rows_per_type.insert( pair<string, vector<int>> ("odom", {}) );
  A_rows_per_type.insert( pair<string, vector<int>> ("gps", {}) );

  // solve the graph once
  int A_rows_count= addNoiselessPriorFactor(newgraph,
                        		  initialEstimate, 
                          		scenario,
                          		A_rows_per_type);
  isam.update(newgraph, initialEstimate);
  result = isam.calculateEstimate();
  newgraph = NonlinearFactorGraph();
  initialEstimate.clear();

  // Simulate poses and imu measurements, adding them to the factor graph
  while (counters.current_time < params.sim_time){
    counters.increase_time();
      
    // Predict acceleration and gyro measurements in (actual) body frame

    
    Point3 acc_noise = generate_random_point( noise_generator, params.noise_dist["acc"] );
    Vector3 msmt_acc = scenario.acceleration_b(counters.current_time) -
                          scenario.rotation(counters.current_time).transpose() * params.imu_params->n_gravity +
                          acc_noise.vector();
    Point3 gyro_noise = generate_random_point( noise_generator, params.noise_dist["gyro"] );                      
    Vector3 msmt_w = scenario.omega_b(counters.current_time) + gyro_noise.vector();
    params.accum.integrateMeasurement(msmt_acc, msmt_w, params.dt_imu);

    // GPS update
    if (counters.gps_time_accum > params.dt_gps) {

      counters.increase_factors_count();

      // save the current position
      true_positions.push_back( scenario.pose(counters.current_time).translation() );

      // predict from IMU accumulated msmts
      prev_state = NavState(result.at<Pose3>  (X(counters.prev_factor)), 
                            result.at<Vector3>(V(counters.prev_factor)));
      prev_bias = result.at<imuBias::ConstantBias>(B(counters.prev_factor));
      predict_state = params.accum.predict(prev_state, prev_bias);

      // predicted init values
      initialEstimate.insert(X(counters.current_factor), predict_state.pose());
      initialEstimate.insert(V(counters.current_factor), predict_state.velocity());
      initialEstimate.insert(B(counters.current_factor), imuBias::ConstantBias());  

      // Add Imu Factor
      CombinedImuFactor imu_factor(X(counters.prev_factor),    V(counters.prev_factor), 
                                   X(counters.current_factor), V(counters.current_factor), 
                                   B(counters.prev_factor),    B(counters.current_factor), 
                                   params.accum);

      addOdomFactor(newgraph,
                    imu_factor,
                    A_rows_per_type, 
                    A_rows_count);
   
      // Adding GPS factor
      Point3 gps_msmt= sim_gps_msmt(scenario.pose(counters.current_time).translation(),
                     noise_generator, 
                     params.noise_dist["range"],
                     params.is_noisy["gps"]);

      GPSFactor gps_factor(X(counters.current_factor), gps_msmt, params.gps_cov);
      addGPSFactor(newgraph,
                   gps_factor,
                   A_rows_per_type,
                   A_rows_count);

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

        addLidarFactor(newgraph,
                       range_bearing_factor,
                       A_rows_per_type, 
                       A_rows_count);
      }      
      
      // Incremental solution
      isam_result = isam.update(newgraph, initialEstimate);
      result = isam.calculateEstimate();

      // compute error
      online_error.push_back(compute_error(scenario.pose(counters.current_time),
                            result.at<Pose3>(X(counters.current_factor)) ));

	    // reset variables
      newgraph = NonlinearFactorGraph();
      params.accum.resetIntegration();
      initialEstimate.clear();
      counters.reset_timer();
    }
  } // end for loop  



// post process data showing each hypothesis
postProcess(result,
           isam_result,
           isam,
           A_rows_per_type,
           counters,
           params);

  // save the data TODO: give option to save in different folder
saveData(result,
         true_positions,
         landmarks,
         online_error);

  return 0;
}
