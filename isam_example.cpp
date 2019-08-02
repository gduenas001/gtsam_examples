// TODO 
// corrupts the IMU & gps msmts with noise and use that same noise in the filtering
// for that i need to figure out how the imu noise is specified: continuous - discrete time conversion.
// compare the estimate with the actual scenario pose in plots


#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <typeinfo>
#include <fstream>
#include <vector>
#include <random>


using namespace std;
using namespace gtsam;

// Shorthand for velocity and pose variables
using symbol_shorthand::V;
using symbol_shorthand::X;
using symbol_shorthand::B;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // parameters
  int num_imu_epochs;
  if (argc == 2){
    num_imu_epochs= atoi(argv[1]);
  } else {
    num_imu_epochs= 100;
  }

  // IMU preintegrator
  double accel_noise_sigma = 0.0003924; // what units??
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = I_3x3 * 1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration

  // create parameters
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

  // Start with a camera on x-axis looking at origin (only use pose_0 from here to generate the scenario)
  double radius = 30;
  const Point3 up(0, 0, 1), target(0, 0, 0);
  const Point3 position(radius, 0, 0);
  const SimpleCamera camera = SimpleCamera::Lookat(position, target, up);
  const Pose3 pose_0 = camera.pose();

  // Now, create a constant-twist scenario that makes the camera orbit the origin
  double angular_velocity = M_PI,  // rad/sec
         delta_t = 1.0 / 18;          // makes for 10 degrees per step
  Vector3 angular_velocity_vector(0, -angular_velocity, 0);
  Vector3 linear_velocity_vector(radius * angular_velocity, 0, 0);
  ConstantTwistScenario scenario(angular_velocity_vector, linear_velocity_vector, pose_0);

  // Create a factor graph &  ISAM2
  NonlinearFactorGraph newgraph, complete_graph;
  ISAM2 isam;

  // Create the initial estimate to the solution
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 0.1 rad std on roll, pitch, yaw, 30cm std on x,y,z.
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  PriorFactor<Pose3> pose_prior(X(0), pose_0, pose_noise);
  newgraph.add(PriorFactor<Pose3>(X(0), pose_0, pose_noise));
  complete_graph.add(PriorFactor<Pose3>(X(0), pose_0, pose_noise));
  initialEstimate.insert(X(0), pose_0);

  // add velocity prior to graph and init values
  Vector vel_prior(3); // needs to be a dynamically allocated vector (I don't know why)
  // vel_prior << 0, angular_velocity * radius, 0;
  // cout<< "initial velocity "<< scenario.velocity_n(0)<< endl;
  vel_prior= scenario.velocity_n(0);
  noiseModel::Diagonal::shared_ptr vel_noise = noiseModel::Diagonal::Sigmas( Vector3::Constant(0.01) );
  PriorFactor<Vector> vel_prior_factor(V(0), vel_prior, vel_noise);
  newgraph.add(vel_prior_factor);
  complete_graph.add(vel_prior_factor);  
  initialEstimate.insert(V(0), vel_prior);

  // Add bias priors to graph and init values
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
  PriorFactor<imuBias::ConstantBias> bias_prior_factor(B(0), imuBias::ConstantBias(), bias_noise);
  newgraph.add(bias_prior_factor); 
  complete_graph.add(bias_prior_factor); 
  initialEstimate.insert(B(0), imuBias::ConstantBias());

  // solve the graph once
  isam.update(newgraph, initialEstimate);
  result = isam.calculateEstimate();
  newgraph = NonlinearFactorGraph();
  initialEstimate.clear();

   // noise generator
  std::default_random_engine noise_generator;
  std::normal_distribution<double> accel_noise_dist(0, accel_noise_sigma);
  std::normal_distribution<double> gyro_noise_dist(0, gyro_noise_sigma);

  // initialize variables
  noiseModel::Diagonal::shared_ptr gps_cov = noiseModel::Isotropic::Sigma(3,0.5); // GPS covariance is constant
  NavState prev_state, predict_state;
  imuBias::ConstantBias prev_bias;
  std::vector<Point3> true_positions;
  true_positions.push_back( scenario.pose(0).translation() );

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 1; i < num_imu_epochs; ++i) {

    double current_time = i * delta_t;
      
    // save the current position
    true_positions.push_back( scenario.pose(current_time).translation() );

    // Predict acceleration and gyro measurements in (actual) body frame
    Vector3 measuredAcc = scenario.acceleration_b(current_time) -
                          scenario.rotation(current_time).transpose() * imu_params->n_gravity;
    Vector3 measuredOmega = scenario.omega_b(current_time);
    accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t);
    prev_state = NavState(result.at<Pose3>(X(i-1)), result.at<Vector3>(V(i-1)));
    prev_bias = result.at<imuBias::ConstantBias>(B(i-1));
    predict_state = accum.predict(prev_state, prev_bias);

    // Known init values
    Pose3 pose_i = scenario.pose(current_time);
    initialEstimate.insert(X(i), scenario.pose(current_time) );
    initialEstimate.insert(V(i), scenario.velocity_n(current_time) );
    initialEstimate.insert(B(i), imuBias::ConstantBias() );

    // // predicted init values
    // initialEstimate.insert(X(i), predict_state.pose());
    // initialEstimate.insert(V(i), predict_state.velocity());
    // initialEstimate.insert(B(i), imuBias::ConstantBias());  

    // Add Imu Factor
    CombinedImuFactor imufac(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), accum);
    newgraph.add(imufac);
    complete_graph.add(imufac);

    // // Adding GPS factor
    if (i < 6)
    {
      GPSFactor gps_factor(X(i), scenario.pose(current_time).translation(), gps_cov);
      newgraph.add(gps_factor);
      complete_graph.add(gps_factor);  
    }
    
    // Incremental solution
    isam.update(newgraph, initialEstimate);
    result = isam.calculateEstimate();
    newgraph = NonlinearFactorGraph();
    accum.resetIntegration();
    initialEstimate.clear();
  } // end for loop
  
  // write poses into file
  string filename = "estimated_positions.csv";
  fstream stream(filename.c_str(), fstream::out);
  for(const Values::ConstKeyValuePair& key_value: result) {
    if ( typeid(key_value.value) == typeid(gtsam::GenericValue<gtsam::Pose3>) ) {
      const Pose3& write_pose = key_value.value.cast<Pose3>();
      stream << write_pose.x() << "," << write_pose.y() << "," << write_pose.z() << endl;
    }
  }
  stream.close();

  filename = "true_positions.csv";
  stream.open(filename.c_str(), fstream::out);
  for (std::vector<Point3>::iterator it = true_positions.begin() ; it != true_positions.end(); ++it) {
    stream << it->x() << "," << it->y() << "," << it->z() << endl;
  }
  stream.close();

  // print path with python
  string command = "python ../python_plot.py";
  system(command.c_str());


  // save factor graph as graphviz dot file
  // ofstream os("isam_example.dot");
  // complete_graph.saveGraph(os, result);

  GTSAM_PRINT(result);


  return 0;
}
/* ************************************************************************* */
