// TODO 
// - need to figure out how the imu noise is specified: continuous - discrete time conversion.
// - data association for landmarks
// - different frequencies for GPS and lidar
// - use sliding window filter
// - odom M matrix is rank 6 because of the actual number of measurements

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <typeinfo>
#include <boost/math/distributions/chi_squared.hpp>

#include <helpers.h>
#include <postProcess.h>
#include <optionsParser.h>

using namespace std;
using namespace gtsam;

const double kGravity = 9.81;





/* ************************************************************************* */
int main(int argc, char** argv) {

  // parse the options
  Params params;
  optionsParser(argc, argv, params);

  // create parameters
  Matrix33 measured_acc_cov = I_3x3 * pow(params.accel_noise_sigma,2);
  Matrix33 measured_omega_cov = I_3x3 * pow(params.gyro_noise_sigma,2);
  Matrix33 bias_acc_cov = I_3x3 * pow(params.accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = I_3x3 * pow(params.gyro_bias_rw_sigma,2);
  Matrix33 integration_error_cov = I_3x3 * 1e-20; // error committed in integrating position from velocities, 8 
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-20; // error in the bias used for preintegration, 5

  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_params = 
                              PreintegratedCombinedMeasurements::Params::MakeSharedU(kGravity);
  imu_params->setAccelerometerCovariance(measured_acc_cov);
  imu_params->setGyroscopeCovariance(measured_omega_cov);
  imu_params->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  imu_params->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  imu_params->biasAccOmegaInt = bias_acc_omega_int;
  imu_params->setIntegrationCovariance(integration_error_cov);
  imu_params->setUse2ndOrderCoriolis(false);
  imu_params->setOmegaCoriolis(Vector3(0, 0, 0));
  PreintegratedCombinedMeasurements accum(imu_params);
  std::default_random_engine noise_generator;    // noise generator
  std::normal_distribution<double> accel_noise_dist(0, params.accel_noise_sigma);
  std::normal_distribution<double> gyro_noise_dist(0, params.gyro_noise_sigma);
  std::normal_distribution<double> range_noise_dist(0, params.range_noise_sigma);
  std::normal_distribution<double> bearing_noise_dist(0, params.bearing_noise_sigma);
  std::normal_distribution<double> gps_noise_dist(0, params.gps_noise_sigma);
  noiseModel::Diagonal::shared_ptr gps_cov = noiseModel::Isotropic::Sigma(3, params.gps_noise_sigma); // GPS covariance is constant
  ConstantTwistScenario scenario = createConstantTwistScenario(params.scenario_radius, params.scenario_linear_vel);
  noiseModel::Diagonal::shared_ptr lidar_cov = noiseModel::Diagonal::Sigmas( (Vector(3) 
            << params.bearing_noise_sigma, params.bearing_noise_sigma, params.range_noise_sigma).finished() );
  vector<Point3> landmarks = createLandmarks(params.scenario_radius);
  ISAM2Params isam_params;
  isam_params.evaluateNonlinearError = true;
 

  // Create a factor graph &  ISAM2
  NonlinearFactorGraph newgraph;
  ISAM2 isam(isam_params);
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
    Point3 acc_noise = generate_random_point( noise_generator, accel_noise_dist );
    Vector3 measuredAcc = scenario.acceleration_b(current_time) -
                          scenario.rotation(current_time).transpose() * imu_params->n_gravity;
                          // acc_noise.vector();
    Point3 gyro_noise = generate_random_point( noise_generator, gyro_noise_dist );                      
    Vector3 measuredOmega = scenario.omega_b(current_time); // + gyro_noise.vector();
    accum.integrateMeasurement(measuredAcc, measuredOmega, params.dt_imu);

    // GPS update
    if (gps_time_accum > params.dt_gps) {

      // save the current position
      true_positions.push_back( scenario.pose(current_time).translation() );

      // predict from IMU accumulated msmts
      prev_state = NavState(result.at<Pose3>(X(pose_factor_count-1)), result.at<Vector3>(V(pose_factor_count-1)));
      prev_bias = result.at<imuBias::ConstantBias>(B(pose_factor_count-1));
      predict_state = accum.predict(prev_state, prev_bias);

      // predicted init values
      initialEstimate.insert(X(pose_factor_count), predict_state.pose());
      initialEstimate.insert(V(pose_factor_count), predict_state.velocity());
      initialEstimate.insert(B(pose_factor_count), imuBias::ConstantBias());  

      // Add Imu Factor
      CombinedImuFactor imufac(X(pose_factor_count - 1), V(pose_factor_count - 1), 
                               X(pose_factor_count),     V(pose_factor_count), 
                               B(pose_factor_count - 1), B(pose_factor_count), accum);
      addOdomFactor(newgraph,
                    imufac,
                    factor_types, 
                    A_rows_per_type, 
                    A_rows_count);
   
      // Adding GPS factor
      Point3 gps_noise(generate_random_point(noise_generator, gps_noise_dist));
      Point3 gps_msmt = scenario.pose(current_time).translation() + gps_noise;
      GPSFactor gps_factor(X(pose_factor_count), gps_msmt, gps_cov);
      addGPSFactor(newgraph,
                   gps_factor,
                   factor_types,
                   A_rows_per_type,
                   A_rows_count);

      // lidar measurements
      for (int j = 0; j < landmarks.size(); ++j) {
        Eigen::MatrixXd range_jacobian;
        Eigen::MatrixXd bearing_jacobian;
        
        // range
        double range = scenario.pose(current_time).range(landmarks[j], range_jacobian);
        double range_noise = range_noise_dist(noise_generator);
        range = range + range_noise;

        // bearing
        Unit3 bearing = scenario.pose(current_time).bearing(landmarks[j], bearing_jacobian);
        Rot3 bearing_noise = Rot3::RzRyRx(bearing_noise_dist(noise_generator),
                                          bearing_noise_dist(noise_generator),
                                          bearing_noise_dist(noise_generator));
        bearing = bearing_noise.rotate(bearing);

        // range-bearing factor
        RangeBearingFactorMap range_bearing_factor(X(pose_factor_count), 
                                                   range,
                                                   bearing,
                                                   landmarks[j], 
                                                   lidar_cov);

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
      accum.resetIntegration();
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
