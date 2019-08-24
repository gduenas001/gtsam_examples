
#include <helpers.h>

using namespace std;
using namespace gtsam;

// generate a random 3D point
Point3 generate_random_point(std::default_random_engine &generator, std::normal_distribution<double> &distribution) {
  return Point3(distribution(generator),distribution(generator),distribution(generator));
}

// compute average of vector of poses
Pose3 Pose3_vector_average(std::vector<Pose3> poses){
  // Point3 ave_translation
  for (std::vector<Pose3>::iterator it = poses.begin() ; it != poses.end(); ++it) {

  }
  return Pose3();
}


// add a noiseless prior factor
void addNoiselessPriorFactor(NonlinearFactorGraph &new_graph, NonlinearFactorGraph &complete_graph, Values &initial_estimate,
                             const Scenario &scenario) {
  // Add a prior on pose x0. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.0), Vector3::Constant(0.0)).finished());
  PriorFactor<Pose3> pose_prior(X(0), scenario.pose(0), pose_noise);
  new_graph.add(PriorFactor<Pose3>(X(0), scenario.pose(0), pose_noise));
  complete_graph.add(PriorFactor<Pose3>(X(0), scenario.pose(0), pose_noise));
  initial_estimate.insert(X(0), scenario.pose(0));

  // add velocity prior to graph and init values
  Vector vel_prior(3); // needs to be a dynamically allocated vector (I don't know why)
  vel_prior= scenario.velocity_n(0);
  noiseModel::Diagonal::shared_ptr vel_noise = noiseModel::Diagonal::Sigmas( Vector3::Constant(0.0) ); // default 0.01
  PriorFactor<Vector> vel_prior_factor(V(0), vel_prior, vel_noise);
  new_graph.add(vel_prior_factor);
  complete_graph.add(vel_prior_factor);  
  initial_estimate.insert(V(0), vel_prior);

  // Add bias priors to graph and init values
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.0)); // default 0.1
  PriorFactor<imuBias::ConstantBias> bias_prior_factor(B(0), imuBias::ConstantBias(), bias_noise);
  new_graph.add(bias_prior_factor); 
  complete_graph.add(bias_prior_factor); 
  initial_estimate.insert(B(0), imuBias::ConstantBias());

}


// creates the ground truth from where we simulate the measurements and measure the error
ConstantTwistScenario createConstantTwistScenario(double radius, double linear_velocity) {
  // Start with a camera on x-axis looking at origin (only use pose_0 from here to generate the scenario)
  const Point3 up(0, 0, 1), target(0, 0, 0);
  const Point3 position(radius, 0, 0);
  const SimpleCamera camera = SimpleCamera::Lookat(position, target, up);
  const Pose3 pose_0 = camera.pose();

  // Now, create a constant-twist scenario that makes the camera orbit the origin
  double angular_velocity = linear_velocity / radius;  // rad/sec
  Vector3 angular_velocity_vector(0, -angular_velocity, 0);
  Vector3 linear_velocity_vector(linear_velocity, 0, 0);
  return ConstantTwistScenario(angular_velocity_vector, linear_velocity_vector, pose_0);
}

// creates the map of landmarks and stores them in a vector of 3D points
std::vector<Point3>  createLandmarks(double radius){
  double distance = radius + radius/10;
  std::vector<Point3> landmarks;
  landmarks.push_back( Point3(distance, 0, 0) );  
  landmarks.push_back( Point3(0, distance, 0) );
  landmarks.push_back( Point3(-distance, 0, 0) );
  landmarks.push_back( Point3(0, -distance, 0) );
  
  return landmarks;
}


