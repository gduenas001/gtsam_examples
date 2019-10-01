
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
#include <gtsam/navigation/ImuBias.h>
#include <vector>
#include <string>


#include "RangeBearingFactorMap.h"
#include "optionsParser.h"
#include "RangeBearingMeasurement.h"

using namespace gtsam;
using namespace std;

// Shorthand for velocity and pose variables
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;




// save data
void saveData(Values result,
              std::vector<Point3> true_positions,
              std::vector<Point3> landmarks,
              std::vector<Pose3> online_error);


// generate a random 3D point
Point3 generate_random_point(std::default_random_engine &generator, 
							 std::normal_distribution<double> &distribution);

// TBD
Vector6 errorAverage(std::vector<Pose3> poses);

// add a noiseless prior factor
int addNoiselessPriorFactor(NonlinearFactorGraph &new_graph, 
                             std::vector<std::string> &factor_types,
							 Values &initial_estimate,
                             const Scenario &scenario,
                             map<string, vector<int>> &A_rows_per_type);


// return int vector with increasing value
std::vector<int> returnIncrVector(int start, int num_elem);


// creates the ground truth from where we simulate the measurements and measure the error
ConstantTwistScenario createConstantTwistScenario(double radius = 30, double linear_velocity = 25);


// creates the map of landmarks and stores them in a vector of 3D points
std::vector<Point3>  createLandmarks(double radius);

// eliminate all factors of type "type"
void eliminateFactorsByType_old(
          boost::shared_ptr<GaussianFactorGraph> &lin_graph,
          vector<string> factor_types,
          string type);

Matrix eliminateFactorsByType(Matrix &A,
              map<string, vector<int>> &A_rows_per_type,
              string type);

Matrix extractJacobianRows(Matrix &A, vector<int> row_inds);

void printIntVector(vector<int> v);

void addLidarFactor(NonlinearFactorGraph &newgraph,
                    RangeBearingFactorMap &range_bearing_factor,
                    vector<string> &factor_types, 
                    map<string, vector<int>> &A_rows_per_type, 
                    int &A_rows_count);

void addGPSFactor(NonlinearFactorGraph &newgraph,
                  GPSFactor &gps_factor,
                  vector<string> &factor_types, 
                  map<string, vector<int>> &A_rows_per_type, 
                  int &A_rows_count);

void addOdomFactor(NonlinearFactorGraph &newgraph,
                   CombinedImuFactor &imufac,
                   vector<string> &factor_types, 
                   map<string, vector<int>> &A_rows_per_type, 
                   int &A_rows_count);

void printMatrix(gtsam::Matrix A);


Pose3 compute_error(Pose3 true_pose, Pose3 estimated_pose);


RangeBearingMeasurement sim_lidar_msmt(ConstantTwistScenario &scenario,
                    Point3 &landmark,
                    double time,
                    Params &params,
                    std::default_random_engine noise_generator);


