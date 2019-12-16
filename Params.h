
#pragma once

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <random>


using namespace gtsam;
using namespace std;


struct Params{
  /*
  Struc that contains the parameters for the simulation.
  The default parameters are included here and can be changed at runtime 
  e.g. if you want to change the uncertainty in the IMU accelerometer
  "--accel_noise_sigma 0.2"
  */

  // constant -- cannot be changed with the options parser
  const double k_gravity = 9.81;

  // -----------------------------------
  // params that are build later
  gtsam::noiseModel::Diagonal::shared_ptr lidar_cov;
  gtsam::noiseModel::Diagonal::shared_ptr gps_cov;
  gtsam::noiseModel::Diagonal::shared_ptr prior_pose_cov;
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_cov;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_cov;

  gtsam::Matrix33 measured_acc_cov;
  gtsam::Matrix33 measured_omega_cov;
  gtsam::Matrix33 bias_acc_cov;
  gtsam::Matrix33 bias_omega_cov;
  gtsam::Matrix33 integration_error_cov;
  gtsam::Matrix66 bias_acc_omega_int;
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_params;
  gtsam::PreintegratedCombinedMeasurements accum;
  std::map<std::string, std::normal_distribution<double>> noise_dist;
  gtsam::ISAM2Params isam_params, fl_isam_params;
  std::map<std::string, bool> is_noisy;

  // -----------------------------------
  // options with default values 

  // for debug
  bool is_verbose= false;

  // IMU noise specs
  // TODO: investigate what this units are. 
  double accel_noise_sigma= 0.0003924; //0.1; // default: 0003924
  double gyro_noise_sigma= 0.000205689024915; // 0.1; // default: 000205689024915
  double accel_bias_rw_sigma = 0.004905; // default: 0.004905
  double gyro_bias_rw_sigma = 0.000001454441043; // default: 0.000001454441043

  // GPS position noise spec
  double gps_noise_sigma = 1.0; // meters

  // IMU freq
  double dt_imu = 1.0 / 200; // default 125HZ -- makes for 10 degrees per step (1.0 / 18)

  // GPS freq (at the moment GPS and lidar are sync)
  // TODO: add functionality to have asynchronous GPS and lidar measurements
  double dt_gps = 1.0; // seconds

  // The current scenario simulate a robot moving in circles,
  // this is the radius of such movement & linear velocity
  double scenario_radius = 30; // meters
  double scenario_linear_vel = 50 / 3.6; // m/s

  // lidar feature detections noise specs
  double range_noise_sigma = 0.20; // meters, range standard deviation
  double bearing_noise_sigma = 3 * M_PI / 180; // rads,  bearing standard dev

  // default simulation time (to increase the simulation time: "--sim_time <time_in_secs>")
  double sim_time= 2; // seconds

  // deprecated
  bool evaluate_nonlinear_error= true; // for the residuals

  // prob allocation for the upper bound of lambda
  double P_lambda= 1e-5; 

  // Alert limits
  double AL_x= 1; // meters, alert limit in x-coord
  double AL_y= 1; // meters, alert limit in y-coord
  double AL_z= 1; // meters, alert limit in z-coord

  // if set to false, the measurements are perfect
  bool is_noisy_gps= true; // if the GPS measurements are noisy
  bool is_noisy_lidar= true; // if the lidar measurements are noisy
  bool is_noisy_imu= true; // if the IMU measurements are noisy
  bool is_noisy_prior= true; // if the prior pose is noisy

  // prior position noise
  double prior_position_noise_sigma= 0.01; //meters

  // prior orientation noise
  double prior_orientation_noise_sigma= 0.01; //rads

  // prior velocity noise
  double prior_vel_noise_sigma= 0.01; // m/s

  // prior IMU bias noise
  double prior_bias_acc_noise_sigma= 0.05; // m/s2
  double prior_bias_gyro_noise_sigma= 1 * M_PI / 180; // rad

  // noise generator seed
  int seed= 0;

  // fixed-lag smoother lag time
  double lag= 3;

  // workspace folder
  std::string workspace;

  // landmarks
  std::vector<gtsam::Point3> landmarks;
};


