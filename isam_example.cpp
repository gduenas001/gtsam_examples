// TODO 
// corrupts the IMU & gps msmts with noise and use that same noise in the filtering
// for that i need to figure out how the imu noise is specified: continuous - discrete time conversion.



#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <typeinfo>
#include <fstream>
#include <vector>
#include <random>
// #include <system>


using namespace std;
using namespace gtsam;

// Shorthand for velocity and pose variables
using symbol_shorthand::V;
using symbol_shorthand::X;
using symbol_shorthand::B;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

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

  // Create a factor graph
  NonlinearFactorGraph newgraph, complete_graph;

  // Create (incremental) ISAM2 solver
  ISAM2 isam;

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 0.1 rad std on roll, pitch, yaw, 30cm std on x,y,z.
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  PriorFactor<Pose3> pose_prior(X(0), pose_0, pose_noise);
  newgraph.add(PriorFactor<Pose3>(X(0), pose_0, pose_noise));
  complete_graph.add(PriorFactor<Pose3>(X(0), pose_0, pose_noise));
  initialEstimate.insert(X(0), pose_0);

  // add velocity prior to graph and init values
  Vector velocity_prior(3); // needs to be a dynamically allocated vector (I don't know why)
  velocity_prior << 0, angular_velocity * radius, 0;
  noiseModel::Diagonal::shared_ptr velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  PriorFactor<Vector> vel_prior(V(0), velocity_prior, velnoise);
  newgraph.add(vel_prior);
  complete_graph.add(vel_prior);  
  initialEstimate.insert(V(0), velocity_prior);

  // Add bias priors to graph and init values
  noiseModel::Diagonal::shared_ptr biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
  PriorFactor<imuBias::ConstantBias> biasprior(B(0), imuBias::ConstantBias(), biasnoise);
  newgraph.add(biasprior); 
  complete_graph.add(biasprior); 
  initialEstimate.insert(B(0), imuBias::ConstantBias());

  // solve the graph once
  isam.update(newgraph, initialEstimate);
  result = isam.calculateEstimate();
  newgraph = NonlinearFactorGraph();
  initialEstimate.clear();

   // noise generator
  std::default_random_engine noise_generator;
  std::normal_distribution<double> distribution(5.0,2.0);

  // initialize variables
  noiseModel::Diagonal::shared_ptr gps_cov = noiseModel::Isotropic::Sigma(3,1.0); // GPS covariance is constant
  NavState prev_state, predict_state;
  imuBias::ConstantBias prev_bias;

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 1; i < 10; ++i) {

    double current_time = i * delta_t;
      
    // Predict acceleration and gyro measurements in (actual) body frame
    Vector3 measuredAcc = scenario.acceleration_b(current_time) -
                          scenario.rotation(current_time).transpose() * imu_params->n_gravity;
    Vector3 measuredOmega = scenario.omega_b(current_time);
    accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t);
    prev_state = NavState(result.at<Pose3>(X(i-1)), result.at<Vector3>(V(i-1)));
    prev_bias = result.at<imuBias::ConstantBias>(B(i-1));
    predict_state = accum.predict(prev_state, prev_bias);

    // insert init values
    Pose3 pose_i = scenario.pose(current_time);
    initialEstimate.insert(X(i), predict_state.pose());
    initialEstimate.insert(V(i), predict_state.velocity());
    initialEstimate.insert(B(i), imuBias::ConstantBias());  

    // Add Imu Factor
    CombinedImuFactor imufac(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), accum);
    newgraph.add(imufac);
    complete_graph.add(imufac);

    // Adding GPS factor
    GPSFactor gps_factor(X(i), scenario.pose(current_time).translation(), gps_cov);
    newgraph.add(gps_factor);
    complete_graph.add(gps_factor);  

    // Incremental solution
    isam.update(newgraph, initialEstimate);
    result = isam.calculateEstimate();
    newgraph = NonlinearFactorGraph();
    accum.resetIntegration();
    initialEstimate.clear();
  } // end for loop
  
  // save factor graph as graphviz dot file
  ofstream os("isam_example.dot");
  complete_graph.saveGraph(os, result);
  
  // Also print out to console
  // newgraph.saveGraph(cout, result);

  // GTSAM_PRINT(result);

  string command = "python ../python_plot_example.py";
  system(command.c_str());
  return 0;
}
/* ************************************************************************* */
