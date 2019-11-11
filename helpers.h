
/*
Helper functions for the main code.
*/


#pragma once


#include <random>
#include <fstream>

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/navigation/ImuBias.h>
#include <vector>
#include <string>


#include "RangeBearingFactorMap.h"
#include "optionsParser.h"
#include "RangeBearingMeasurement.h"
#include "Counters.h"

using namespace gtsam;
using namespace std;

// Shorthand for velocity and pose variables
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;




/*
Saves the following .csv files in the results folder:
- estimated_positions
- true_positions
- errors
- average_errors
- landmarks
Note that each time the simulation is run, this files get
overwritten.
*/
void save_data(gtsam::Values result,
              std::vector<Point3> true_positions,
              std::vector<Point3> landmarks,
              std::vector<Pose3> online_error);


/*
Generate a random 3D point from the given distribution
*/
gtsam::Point3 
generate_random_point(
              std::default_random_engine &generator, 
							std::normal_distribution<double> &distribution);

/*
Compute average of vector of poses is a 6D vector
(roll, pitch, yaw, x, y, z)
*/
gtsam::Vector6 error_average(std::vector<Pose3> poses);

/*
Add a prior factors in:
- pose (X)
- velocity (V) 
- IMU bias (B)
the priors are sampled from their corresponding
noise distributions is set in the parameters.
*/
int add_prior_factor(
         gtsam::NonlinearFactorGraph &new_graph, 
         gtsam::FixedLagSmoother::KeyTimestampMap &new_timestamps,
         gtsam::Values &initial_estimate,
         const gtsam::Scenario &scenario,
         std::default_random_engine &noise_generator, 
         std::map<string, vector<int>> &A_rows_per_type,
         Counters &counters,
         Params &params);

/*
Return int vector with increasing values
e.g. returnIncrVector(3, 2) --> (3,4)
*/
std::vector<int> returnIncrVector(int start, 
                                  int num_elem);

/*
Creates the ground truth from where we simulate the
measurements. The simulated robot moves in circles at a
constant speed.
*/
gtsam::ConstantTwistScenario 
createConstantTwistScenario(double radius = 30, 
                            double linear_velocity = 25);

/*
Creates the map of landmarks and stores them in a vector
of 3D points. 
TODO: read landmarks from text file.
*/
std::vector<Point3> createLandmarks(double radius);


/*
Extracts the rows from the matrix and return a new matrix
TODO: check if there is an inbuilt function
*/
gtsam::Matrix 
extractMatrixRows(gtsam::Matrix &A, 
                  std::vector<int> &row_inds);

/*
Extracts the columns from the matrix and return a new matrix
TODO: check if there is an inbuilt function
*/
gtsam::Matrix 
extractMatrixColumns(gtsam::Matrix &A, 
                     std::vector<int> &col_inds);

/*
Extract matrix rows and columns and return matrix
*/
gtsam::Matrix 
extractMatrixRowsAndColumns(
                   gtsam::Matrix &A, 
                   std::vector<int> &row_inds, 
                   std::vector<int> &col_inds);

/*
Simple function to print vector of ints in terminal
*/
void printIntVector(std::vector<int> v);

/*
Add lidar factor to factor graph.
It also keep A_rows_per_type updated.
*/
void add_lidar_factor(
      gtsam::NonlinearFactorGraph &newgraph,
      RangeBearingFactorMap &range_bearing_factor,
      std::map<string, vector<int>> &A_rows_per_type, 
      int &A_rows_count,
      Counters &counters);

/*
Add GPS factor to factor graph.
It also keep A_rows_per_type updated.
*/
void add_gps_factor(
      gtsam::NonlinearFactorGraph &newgraph,
      GPSFactor &gps_factor,
      std::map<string, vector<int>> &A_rows_per_type, 
      int &A_rows_count,
      Counters &counters);


/*
Add IMU factor to factor graph.
It also keep A_rows_per_type updated.
*/
void addOdomFactor(
      gtsam::NonlinearFactorGraph &newgraph,
      gtsam::CombinedImuFactor &imufac,
      std::map<string, vector<int>> &A_rows_per_type, 
      int &A_rows_count,
      Counters &counters);

/*
Simple function to print matrix with nice format
*/
void printMatrix(gtsam::Matrix A);

/*
Compute error between two poses
convention: error = true - estimated
TODO: check if the rotation follows this convention
*/
gtsam::Pose3 
compute_error(gtsam::Pose3 true_pose, 
              gtsam::Pose3 estimated_pose);

/*
Calculate the effective number of measurements in the
graph. This is trying to deal with the fact that the 
effective number of measurements for each IMU factor in 
less than 15, which is the number of measurements in 
each IMU factor.
*/
double 
get_dof_from_graph(const gtsam::NonlinearFactorGraph &graph);

/*
Build t vector. This is the vector that extracts the state
of interest.
*/
std::map<std::string, gtsam::Vector> 
buildt_vector(int size);

/*
Retruns the variances for the last estimated pose in the
graph as a map.
e.g. var["x"] or var["roll"]
TODO: I think that this are the variances in x-y-z
in the inertial frame. Check if they actually are the
variances in lateral-longitudinal-vertical. If not, 
calculate the variances in lateral-longitudinal-vertical.
*/
std::map<std::string, double> 
getVariancesForLastPose(gtsam::ISAM2 &isam,
                        Counters &counters);
  


/*
Returns the variances for the last estimated pose:
"roll" - "pitch" - "yaw" - "x" - "y" - "z"
*/
std::map<string,double> get_variances_for_last_pose(
                  gtsam::IncrementalFixedLagSmoother fixed_lag_smoother,
                  Counters &counters);

/*
Generate lidar (range & bearing) measurements to all
landmarks.
*/
RangeBearingMeasurement 
sim_lidar_msmt(gtsam::ConstantTwistScenario &scenario,
               gtsam::Point3 &landmark,
               double time,
               Params &params,
               std::default_random_engine noise_generator,
               bool is_noisy);


/*
Simulate GPS measurement of the position of the robot.
*/
gtsam::Point3 
sim_gps_msmt(const gtsam::Point3 &true_position,
             std::default_random_engine &noise_generator, 
             std::normal_distribution<double> &gps_distribution,
             bool is_noisy);

/*
Simulate IMU acceleration measurement. This takes care of the gravity.
*/
gtsam::Vector3 sim_imu_acc(
      gtsam::ConstantTwistScenario &scenario,
      std::default_random_engine &noise_generator, 
      std::normal_distribution<double> &imu_acc_dist,
      gtsam::Vector3 g,
      double time,
      bool is_noisy);


/*
Simulate gyro measurement.
*/
gtsam::Vector3 sim_imu_w(
         gtsam::Vector3 true_imu_w,
         std::default_random_engine &noise_generator, 
         std::normal_distribution<double> &imu_gyro_dist,
         bool is_noisy);





// -------------------------------------------------------
/*
deprecated
*/
// -------------------------------------------------------
void eliminateFactorsByType_old(
          boost::shared_ptr<GaussianFactorGraph> &lin_graph,
          vector<string> factor_types,
          string type);

Matrix eliminateFactorsByType(Matrix &A,
              map<string, vector<int>> &A_rows_per_type,
              string type);


Matrix extractJacobianRows(Matrix &A, vector<int> &row_inds);


/*
Calculate the effective number of measurements in the
graph. This is trying to deal with the fact that the 
effective number of measurements for each IMU factor in 
less than 15, which is the number of measurements in 
each IMU factor.
*/
double getDOFfromGraph(
      std::map<string, std::vector<int>> &A_rows_per_type);

/*
Calculate the effective number of measurements in all 
factors of the type. See getDOFfromGraph.
*/
double getDOFfromFactorType(int dim, std::string type);
