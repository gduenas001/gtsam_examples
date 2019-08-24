
#pragma once


#include <random>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/ImuBias.h>


using namespace gtsam;
using namespace std;

// Shorthand for velocity and pose variables
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;


// generate a random 3D point
Point3 generate_random_point(std::default_random_engine &generator, 
							 std::normal_distribution<double> &distribution);

// TBD
Vector6 errorAverage(std::vector<Pose3> poses);

// add a noiseless prior factor
void addNoiselessPriorFactor(NonlinearFactorGraph &new_graph, 
							 NonlinearFactorGraph &complete_graph, 
							 Values &initial_estimate,
                             const Scenario &scenario);

// creates the ground truth from where we simulate the measurements and measure the error
ConstantTwistScenario createConstantTwistScenario(double radius = 30, double linear_velocity = 25);


// creates the map of landmarks and stores them in a vector of 3D points
std::vector<Point3>  createLandmarks(double radius);
