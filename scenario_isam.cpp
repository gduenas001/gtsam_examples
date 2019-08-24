// TODO 
// - need to figure out how the imu noise is specified: continuous - discrete time conversion.
// - data association for landmarks
// - different frequencies for GPS and lidar
// - use sliding window filter
// - check residuals

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <typeinfo>
// #include <fstream>

#include <helpers.h>
#include <RangeBearingFactorMap.h>

using namespace std;
using namespace gtsam;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // run time options
  float sim_time;
  if (argc == 2){
    sim_time= atof(argv[1]);
  } else {
    sim_time= 30; // seconds
  }

  // input parameters
  double accel_noise_sigma = 0.1; // what units?? 0003924
  double gyro_noise_sigma = 0.1; //000205689024915
  double accel_bias_rw_sigma = 0.005; //1e-20; //004905
  double gyro_bias_rw_sigma = 0.000001454441043; //1e-20; //000001454441043
  double gps_noise_sigma = 1.0; // meters
  double dt_imu = 1.0 / 100, // default 125HZ -- makes for 10 degrees per step (1.0 / 18)
         dt_gps = 1.0, // seconds
         scenario_radius = 30, // meters
         scenario_linear_vel = 50 / 3.6, // m/s
         range_noise_sigma = 0.50, // range standard deviation
         bearing_noise_sigma = 5 * M_PI / 180; // bearing standard dev

  // create parameters
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma,2);
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma,2);
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
  std::normal_distribution<double> accel_noise_dist(0, accel_noise_sigma);
  std::normal_distribution<double> gyro_noise_dist(0, gyro_noise_sigma);
  std::normal_distribution<double> range_noise_dist(0, range_noise_sigma);
  std::normal_distribution<double> bearing_noise_dist(0, bearing_noise_sigma);
  std::normal_distribution<double> gps_noise_dist(0, gps_noise_sigma);
  noiseModel::Diagonal::shared_ptr gps_cov = noiseModel::Isotropic::Sigma(3, gps_noise_sigma); // GPS covariance is constant
  ConstantTwistScenario scenario = createConstantTwistScenario(scenario_radius, scenario_linear_vel);
  noiseModel::Diagonal::shared_ptr lidar_cov = noiseModel::Diagonal::Sigmas( (Vector(3) << bearing_noise_sigma, bearing_noise_sigma, range_noise_sigma).finished() );
  std::vector<Point3> landmarks = createLandmarks(scenario_radius);

  // Create a factor graph &  ISAM2
  NonlinearFactorGraph newgraph, complete_graph;
  ISAM2 isam;
  Values initialEstimate, result; // Create the initial estimate to the solution

  // initialize variables
  double gps_time_accum = 0.0,
         current_time = 0.0,
         num_imu_epochs = sim_time / dt_imu;
  int pose_factor_count = 1;
  NavState prev_state, predict_state;
  imuBias::ConstantBias prev_bias;
  std::vector<Point3> true_positions;
  std::vector<Pose3> online_error; // error when computed online
  true_positions.push_back( scenario.pose(0).translation() );
  ISAM2Result isam_result;

  // solve the graph once
  addNoiselessPriorFactor(newgraph, complete_graph, initialEstimate, scenario);
  isam.update(newgraph, initialEstimate);
  result = isam.calculateEstimate();
  newgraph = NonlinearFactorGraph();
  initialEstimate.clear();  

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 1; i < num_imu_epochs; ++i) {

    current_time = i * dt_imu;
    gps_time_accum += dt_imu;
      
    // Predict acceleration and gyro measurements in (actual) body frame
    Point3 acc_noise = generate_random_point( noise_generator, accel_noise_dist );
    Vector3 measuredAcc = scenario.acceleration_b(current_time) -
                          scenario.rotation(current_time).transpose() * imu_params->n_gravity;
                          // acc_noise.vector();
    Point3 gyro_noise = generate_random_point( noise_generator, gyro_noise_dist );                      
    Vector3 measuredOmega = scenario.omega_b(current_time); // + gyro_noise.vector();
    accum.integrateMeasurement(measuredAcc, measuredOmega, dt_imu);

    // GPS update 
    if (gps_time_accum > dt_gps) {

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
      newgraph.add(imufac);
      complete_graph.add(imufac);

      // // Adding GPS factor
      Point3 gps_noise(generate_random_point(noise_generator, gps_noise_dist));
      Point3 gps_msmt = scenario.pose(current_time).translation() + gps_noise;
      GPSFactor gps_factor(X(pose_factor_count), gps_msmt, gps_cov);
      newgraph.add(gps_factor);
      complete_graph.add(gps_factor);

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
                                                   lidar_cov);;
        newgraph.add(range_bearing_factor);
      }      
      
      // Incremental solution
      isam_result = isam.update(newgraph, initialEstimate);
      result = isam.calculateEstimate();

      
      // compute error
      Rot3 rotation_error = scenario.pose(current_time).rotation() *
      	   Rot3( result.at<Pose3>(X(pose_factor_count)).rotation().transpose() );
      Point3 translation_error = scenario.pose(current_time).translation() - 
      							 result.at<Pose3>(X(pose_factor_count)).translation();
      online_error.push_back( Pose3( rotation_error, translation_error ));

	  // reset variables
      newgraph = NonlinearFactorGraph();
      accum.resetIntegration();
      initialEstimate.clear();
      gps_time_accum = 0.0;
      pose_factor_count++;  // increase the factor count

    }
  } // end for loop
  
  // check residuals
  double residual = isam.error(isam.getDelta());
  // double error_before = isam_result.errorBefore;
  // cout<< "error before: "<< error_before << endl;
  // cout<< "error after: "<< isam_result.errorAfter()<< endl;
  cout<< "the residual is: "<< residual<< endl;

  // save the data TODO: give option to save in different folder
  saveData(result,
           true_positions,
           landmarks,
           online_error);

  // print path with python
  string command = "python ../python_plot.py";
  system(command.c_str());


  // save factor graph as graphviz dot file
  // ofstream os("isam_example.dot");
  // complete_graph.saveGraph(os, result);

  // GTSAM_PRINT(result);


  return 0;
}
/* ************************************************************************* */
