
#pragma once


#include <iostream>
#include <getopt.h>
#include <cmath>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <random>
#include <gtsam/nonlinear/ISAM2.h>


using namespace gtsam;
using namespace std;


struct Params{
  // constant -- cannot be changed with the options parser
  const double k_gravity = 9.81;

  // params that are build later
  gtsam::noiseModel::Diagonal::shared_ptr lidar_cov;
  gtsam::noiseModel::Diagonal::shared_ptr gps_cov;
  gtsam::Matrix33 measured_acc_cov;
  gtsam::Matrix33 measured_omega_cov;
  gtsam::Matrix33 bias_acc_cov;
  gtsam::Matrix33 bias_omega_cov;
  gtsam::Matrix33 integration_error_cov;
  gtsam::Matrix66 bias_acc_omega_int;
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_params;
  gtsam::PreintegratedCombinedMeasurements accum;
  std::map<string, std::normal_distribution<double>> noise_dist;
  ISAM2Params isam_params;


  // options -- can be changed at run time
  double accel_noise_sigma = 0.1; // what units?? 0003924
  double gyro_noise_sigma = 0.1; //000205689024915
  double accel_bias_rw_sigma = 0.005; //1e-20; //004905
  double gyro_bias_rw_sigma = 0.000001454441043; //1e-20; //000001454441043
  double gps_noise_sigma = 1.0; // meters
  double dt_imu = 1.0 / 200; // default 125HZ -- makes for 10 degrees per step (1.0 / 18)
  double dt_gps = 1.0; // seconds
  double scenario_radius = 30; // meters
  double scenario_linear_vel = 50 / 3.6; // m/s
  double range_noise_sigma = 0.10; // range standard deviation
  double bearing_noise_sigma = 2 * M_PI / 180; // bearing standard dev
  double sim_time= 2; // seconds -- time to run the simulation
  bool   evaluate_nonlinear_error= true; // for the residuals
  double P_lambda= 1e-5; // prob allocation for the upper bound of lambda
  double AL_x= 1; // alert limit in x-coord
  double AL_y= 1; // alert limit in y-coord
  double AL_z= 1; // alert limit in z-coord
  

};

int optionsParser (int argc, char **argv, Params &params);

void build_variables(Params &params);

