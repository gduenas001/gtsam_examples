/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file imuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in conjunction with GPS
 *  - you can test imuFactor (resp. combinedImuFactor) by commenting (resp. uncommenting)
 *  the line #define USE_COMBINED (few lines below)
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
    N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
    linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A row starting with "1" is a gps correction formatted with
    N, E, D, qX, qY, qZ, qW
 *  Note that for GPS correction, we're only using the position not the rotation. The
 *  rotation is provided in the file for ground truth comparison.
 */

// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>


using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

const string output_filename = "../data/results/results.csv";

typedef noiseModel::Diagonal::shared_ptr noiseDiagShared;
typedef Eigen::Matrix<double, 3,3> Matrix33;

int main(int argc, char* argv[])
{
  string data_filename= "../data/imuAndGPSdata.csv";   
  
  // Set up output file for plotting errors
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

  // Begin parsing the CSV file.  Input the first line for initialization.
  // From there, we'll iterate through the file and we'll preintegrate the IMU
  // or add in the GPS given the input.
  ifstream file(data_filename.c_str());
  string value;

  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD) -- I don't think so - guillermo
  // Format is (N,E,D, velN,velE,velD, qX,qY,qZ,qW) - guillermo
  Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();
  getline(file, value, ','); // i, the first line of the file is the initial position
  for (int i=0; i<9; i++) {
    getline(file, value, ',');
    initial_state(i) = atof(value.c_str());
  }
  getline(file, value, '\n');
  initial_state(9) = atof(value.c_str());
  cout << "initial state:\n" << initial_state.transpose() << "\n\n";

  // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(9), initial_state(6), initial_state(7), initial_state(8));
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.segment<3>(3));
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  // set initial values
  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(0), prior_pose); // this returns ```Symbol('x',correction_count)```
  initial_values.insert(V(0), prior_velocity);
  initial_values.insert(B(0), prior_imu_bias);

  // Assemble prior noise model and add it the graph.
  noiseDiagShared pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseDiagShared velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // [m/s], 3x3 matrix with 0.1 in the diagonal
  noiseDiagShared bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(0), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(0), prior_velocity, velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(0), prior_imu_bias, bias_noise_model));

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924; // what units??
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3) * 1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration

  // initialize the IMU gravity readings to zero (this is simulated data and gravity does not play a role)
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> preintegratedParams = 
                            PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  preintegratedParams->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  preintegratedParams->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  preintegratedParams->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  preintegratedParams->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  preintegratedParams->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  preintegratedParams->biasAccOmegaInt = bias_acc_omega_int;

  // If GTSAM_TANGENT_PREINTEGRATION is ON (which is default): typedef TangentPreintegration PreintegrationType
  // PreintegrationType *imu_preintegrated = new PreintegratedCombinedMeasurements(preintegratedParams, prior_imu_bias);
  PreintegratedCombinedMeasurements *imu_preintegrated = new PreintegratedCombinedMeasurements(preintegratedParams, prior_imu_bias);

  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  // NavState prop_state = prev_state;
  NavState prop_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.005;  // The real system has noise, but here, results are nearly
                      // exactly the same, so keeping this for simplicity.

  // All priors have been set up, now iterate through the data file.
  while (file.good()) {

    // Parse out first value
    getline(file, value, ',');
    int type = atoi(value.c_str());

    if (type == 0) { // IMU measurement
      Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
      for (int i=0; i<5; ++i) {
        getline(file, value, ',');
        imu(i) = atof(value.c_str());
      }
      getline(file, value, '\n');
      imu(5) = atof(value.c_str());

      // Adding the IMU preintegration.
      imu_preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

    } else if (type == 1) { // GPS measurement
      Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
      for (int i=0; i<6; ++i) {
        getline(file, value, ',');
        gps(i) = atof(value.c_str());
      }
      getline(file, value, '\n');
      gps(6) = atof(value.c_str());

      correction_count++; // a gps msmt is a correction --  guillermo

      // Adding IMU factor
      CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                   X(correction_count  ), V(correction_count  ),
                                   B(correction_count-1), B(correction_count  ),
                                   *imu_preintegrated);
      graph->add(imu_factor);

      // Adding GPS factor
      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,1.0); 
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),  // N,
                                  gps(1),  // E,
                                  gps(2)), // D,
                           correction_noise);
      graph->add(gps_factor);

      // predict the pose (NavPose) from the IMU msmts until the GPS
      prop_state = imu_preintegrated->predict(prev_state, prev_bias);

      // add the IMU prediction to initial values before optimization with GPS
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v()); // same as ```prop_state.velocity()```;
      initial_values.insert(B(correction_count), prev_bias);

      // optimize factor graph
      LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
      Values result = optimizer.optimize();

      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset the preintegration object (this only contains the IMU measurements that links one pose with the next one)
      imu_preintegrated->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 gtsam_position = prev_state.pose().translation();
      Vector3 position_error = gtsam_position - gps.head<3>();
      current_position_error = position_error.norm();

      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      Quaternion quat_error = gtsam_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3 euler_angle_error(quat_error.x()*2,
                                 quat_error.y()*2,
                                 quat_error.z()*2);
      current_orientation_error = euler_angle_error.norm();

      // display statistics
      cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";

      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2),
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

      output_time += 1.0;

    } // end GPS msmt
    else {
      cerr << "ERROR parsing file\n";
      return 1;
    }
  }
  fclose(fp_out);
  cout << "Complete, results written to " << output_filename << "\n\n";;
  return 0;
}
