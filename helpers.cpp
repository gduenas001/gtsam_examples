
#include <helpers.h>

using namespace std;
using namespace gtsam;

// save data
void saveData(Values result,
              std::vector<Point3> true_positions,
              std::vector<Point3> landmarks,
              std::vector<Pose3> online_error){
  // initialize variables
  string filename = "";
  fstream stream;

  // write estimated poses into file
  filename = "../results/estimated_positions.csv";
  stream.open(filename.c_str(), fstream::out);
  stream << "--------------- Estimated Positions ---------------"<< endl;
  for(const Values::ConstKeyValuePair& key_value: result) {
    if ( typeid(key_value.value) == typeid(gtsam::GenericValue<gtsam::Pose3>) ) {
      const Pose3& write_pose = key_value.value.cast<Pose3>();
      stream << write_pose.x() << "," << write_pose.y() << "," << write_pose.z() << endl;
    }
  }
  stream.close();

  // write true poses into a file
  filename = "../results/true_positions.csv";
  stream.open(filename.c_str(), fstream::out);
  stream << "--------------- True Positions ---------------"<< endl;
  for (std::vector<Point3>::iterator it = true_positions.begin() ; it != true_positions.end(); ++it) {
    stream << it->x() << "," << it->y() << "," << it->z() << endl;
  }
  stream.close();

  // write landmark into a file
  filename = "../results/landmarks.csv";
  stream.open(filename.c_str(), fstream::out);
  for (std::vector<Point3>::iterator it = landmarks.begin() ; it != landmarks.end(); ++it) {
    stream << it->x() << "," << it->y() << "," << it->z() << endl;
  }
  stream.close();

// write errors into a file
  filename = "../results/errors.csv";
  stream.open(filename.c_str(), fstream::out);
  stream << "--------------- Errors ---------------"<< endl;
  stream << "x[m] \t y[m] \t z[m] \t rho[deg] \t pitch[deg] \t yaw[deg]"<< endl;
  for (std::vector<Pose3>::iterator it = online_error.begin() ; it != online_error.end(); ++it) {
    stream << it->translation().x() <<","
           << it->translation().y() <<","
           << it->translation().z() <<","
           << (it->rotation()).roll() <<"," 
           << (it->rotation()).pitch() << "," 
           << (it->rotation()).yaw() << endl;
  }
  stream.close();

  // write the average errors
  filename = "../results/average_errors.csv";
  stream.open(filename.c_str(), fstream::out);
  stream << "--------------- Average Error ---------------"<< endl;
  Vector6 ave_error = errorAverage(online_error);
  stream << ave_error[0] <<","<< ave_error[1] <<","<< ave_error[2]<<","
         << ave_error[3] * 180/M_PI <<","<< ave_error[4] * 180/M_PI <<","<< ave_error[5] * 180/M_PI<< endl;
  stream.close();
}


// generate a random 3D point
Point3 generate_random_point(std::default_random_engine &generator, std::normal_distribution<double> &distribution) {
  return Point3(distribution(generator),distribution(generator),distribution(generator));
}


// compute average of vector of poses
Vector6 errorAverage(std::vector<Pose3> poses){
  Vector3 ave_translation, ave_rotation; // deault constructor -> zero translation
  for (std::vector<Pose3>::iterator it = poses.begin() ; it != poses.end(); ++it) {
    ave_translation += abs( it->translation() );
    ave_rotation += abs(Vector3( it->rotation().roll(),
                                 it->rotation().pitch(),
                                 it->rotation().yaw() ));
  }
  return (Vector(6) << ave_translation / poses.size(), 
                       ave_rotation / poses.size() ).finished();
}


// add a noiseless prior factor
void addNoiselessPriorFactor(NonlinearFactorGraph &new_graph, 
                             vector<string> &factor_types,
                             Values &initial_estimate,
                             const Scenario &scenario) {
  // Add a prior on pose x0. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.1)).finished() );
  PriorFactor<Pose3> pose_prior(X(0), scenario.pose(0), pose_noise);
  new_graph.add(PriorFactor<Pose3>(X(0), scenario.pose(0), pose_noise));
  initial_estimate.insert(X(0), scenario.pose(0));
  factor_types.push_back("prior_pose");

  // add velocity prior to graph and init values
  Vector vel_prior(3); // needs to be a dynamically allocated vector (I don't know why)
  vel_prior= scenario.velocity_n(0);
  noiseModel::Diagonal::shared_ptr vel_noise = noiseModel::Diagonal::Sigmas( 
        Vector3::Constant(0.01) ); // default 0.01
  PriorFactor<Vector> vel_prior_factor(V(0), vel_prior, vel_noise);
  new_graph.add(vel_prior_factor);
  initial_estimate.insert(V(0), vel_prior);
  factor_types.push_back("prior_vel");

  // Add bias priors to graph and init values
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Diagonal::Sigmas(
        Vector6::Constant(0.01)); // default 0.1
  PriorFactor<imuBias::ConstantBias> bias_prior_factor(B(0), imuBias::ConstantBias(), bias_noise);
  new_graph.add(bias_prior_factor); 
  initial_estimate.insert(B(0), imuBias::ConstantBias());
  factor_types.push_back("prior_bias");
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


