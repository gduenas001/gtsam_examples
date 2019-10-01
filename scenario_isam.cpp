// TODO 
// - need to figure out how the imu noise is specified: continuous - discrete time conversion.
// - data association for landmarks
// - different frequencies for GPS and lidar
// - use sliding window filter
// - odom M matrix is rank 6 because of the actual number of measurements
// - move as much as possible of the constractio outside main
// - record the covariance after solving
// - create function to generate lidar measurements

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <typeinfo>
#include <boost/math/distributions/chi_squared.hpp>

#include "helpers.h"
#include "postProcess.h"
#include "optionsParser.h"

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
  ConstantTwistScenario scenario = createConstantTwistScenario(params.scenario_radius, params.scenario_linear_vel);

  // Create a factor graph &  ISAM2
  NonlinearFactorGraph newgraph;
  ISAM2 isam(params.isam_params);
  Values initialEstimate, result; // Create the initial estimate to the solution

  // initialize variables
  double gps_time_accum = 0.0,
         current_time = 0.0,
         num_imu_epochs = params.sim_time / params.dt_imu;
  int pose_factor_count = 1;
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
  vector<string> factor_types; // stores the factor types (odom, GPS, lidar)
  int A_rows_count = addNoiselessPriorFactor(newgraph,
                 		          factor_types,
                        		  initialEstimate, 
                          		scenario,
                          		A_rows_per_type);
  isam.update(newgraph, initialEstimate);
  result = isam.calculateEstimate();
  newgraph = NonlinearFactorGraph();
  initialEstimate.clear();

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 1; i < num_imu_epochs; ++i) {

    current_time = i * params.dt_imu;
    gps_time_accum += params.dt_imu;
      
    // Predict acceleration and gyro measurements in (actual) body frame
    Point3 acc_noise = generate_random_point( noise_generator, params.noise_dist["acc"] );
    Vector3 measuredAcc = scenario.acceleration_b(current_time) -
                          scenario.rotation(current_time).transpose() * params.imu_params->n_gravity;
                          // acc_noise.vector();
    Point3 gyro_noise = generate_random_point( noise_generator, params.noise_dist["gyro"] );                      
    Vector3 measuredOmega = scenario.omega_b(current_time); // + gyro_noise.vector();
    params.accum.integrateMeasurement(measuredAcc, measuredOmega, params.dt_imu);

    // GPS update
    if (gps_time_accum > params.dt_gps) {

      // save the current position
      true_positions.push_back( scenario.pose(current_time).translation() );

      // predict from IMU accumulated msmts
      prev_state = NavState(result.at<Pose3>(X(pose_factor_count-1)), result.at<Vector3>(V(pose_factor_count-1)));
      prev_bias = result.at<imuBias::ConstantBias>(B(pose_factor_count-1));
      predict_state = params.accum.predict(prev_state, prev_bias);

      // predicted init values
      initialEstimate.insert(X(pose_factor_count), predict_state.pose());
      initialEstimate.insert(V(pose_factor_count), predict_state.velocity());
      initialEstimate.insert(B(pose_factor_count), imuBias::ConstantBias());  

      // Add Imu Factor
      CombinedImuFactor imufac(X(pose_factor_count - 1), V(pose_factor_count - 1), 
                               X(pose_factor_count),     V(pose_factor_count), 
                               B(pose_factor_count - 1), B(pose_factor_count), 
                               params.accum);
      addOdomFactor(newgraph,
                    imufac,
                    factor_types, 
                    A_rows_per_type, 
                    A_rows_count);
   
      // Adding GPS factor TODO: generate gps msmt with function
      Point3 gps_noise(generate_random_point(noise_generator, params.noise_dist["gps"]));
      Point3 gps_msmt = scenario.pose(current_time).translation() + gps_noise;
      GPSFactor gps_factor(X(pose_factor_count), gps_msmt, params.gps_cov);
      addGPSFactor(newgraph,
                   gps_factor,
                   factor_types,
                   A_rows_per_type,
                   A_rows_count);

      // lidar measurements
      for (int j = 0; j < landmarks.size(); ++j) {
        
        RangeBearingMeasurement range_bearing_msmt= sim_lidar_msmt(scenario,
                                           landmarks[j],
                                           current_time,
                                           params,
                                           noise_generator);
        

        // range-bearing factor
        RangeBearingFactorMap range_bearing_factor(X(pose_factor_count), 
                                                   range_bearing_msmt.range,
                                                   range_bearing_msmt.bearing,
                                                   landmarks[j], 
                                                   params.lidar_cov);

        addLidarFactor(newgraph,
                       range_bearing_factor,
                       factor_types, 
                       A_rows_per_type, 
                       A_rows_count);
      }      
      
      // Incremental solution
      isam_result = isam.update(newgraph, initialEstimate);
      result = isam.calculateEstimate();


      // compute error
      online_error.push_back(compute_error(scenario.pose(current_time),
                            result.at<Pose3>(X(pose_factor_count)) ));

	    // reset variables
      newgraph = NonlinearFactorGraph();
      params.accum.resetIntegration();
      initialEstimate.clear();
      gps_time_accum = 0.0;
      pose_factor_count++;  // increase the factor count
    }
  } // end for loop  


// post process data showing each hypothesis
postProcess(result,
           isam_result,
           isam,
           A_rows_per_type);

  // save the data TODO: give option to save in different folder
saveData(result,
         true_positions,
         landmarks,
         online_error);

  return 0;
}
