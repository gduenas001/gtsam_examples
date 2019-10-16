
#include "helpers.h"

using namespace std;
using namespace gtsam;

// -------------------------------------------------------
void saveData(Values result,
              std::vector<Point3> true_positions,
              std::vector<Point3> landmarks,
              std::vector<Pose3> online_error){
  /* 
  save data
  */

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


// -------------------------------------------------------
Point3 generate_random_point(std::default_random_engine &generator, std::normal_distribution<double> &distribution) {
  /*
  generate a random 3D point
  */
  return Point3(distribution(generator),distribution(generator),distribution(generator));
}


// -------------------------------------------------------
Vector6 errorAverage(std::vector<Pose3> poses){
  /*
  compute average of vector of poses
  */

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


// -------------------------------------------------------
int addNoiselessPriorFactor(NonlinearFactorGraph &new_graph, 
                             Values &initial_estimate,
                             const Scenario &scenario,
                             map<string, vector<int>> &A_rows_per_type) {
  /*
  add a noiseless prior factor
  */

  // initialize the count on the rows of A
  int A_rows_count = 0;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.1)).finished() );
  PriorFactor<Pose3> pose_prior(X(0), scenario.pose(0), pose_noise);
  new_graph.add(PriorFactor<Pose3>(X(0), scenario.pose(0), pose_noise));
  initial_estimate.insert(X(0), scenario.pose(0));
  A_rows_per_type.insert(pair<string, vector<int>> 
          ("prior_pose", returnIncrVector(0,6)));
  A_rows_count += 6;
  
  // add velocity prior to graph and init values
  Vector vel_prior(3); // needs to be a dynamically allocated vector (I don't know why)
  vel_prior= scenario.velocity_n(0);
  noiseModel::Diagonal::shared_ptr vel_noise = noiseModel::Diagonal::Sigmas( 
        Vector3::Constant(0.01) ); // default 0.01
  PriorFactor<Vector> vel_prior_factor(V(0), vel_prior, vel_noise);
  new_graph.add(vel_prior_factor);
  initial_estimate.insert(V(0), vel_prior);
  A_rows_per_type.insert(pair<string, vector<int>> 
          ("prior_vel", returnIncrVector(A_rows_count,3)));
  A_rows_count += 3;

  // Add bias priors to graph and init values
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Diagonal::Sigmas(
        Vector6::Constant(0.01)); // default 0.1
  PriorFactor<imuBias::ConstantBias> bias_prior_factor(B(0), imuBias::ConstantBias(), bias_noise);
  new_graph.add(bias_prior_factor); 
  initial_estimate.insert(B(0), imuBias::ConstantBias());
  A_rows_per_type.insert(pair<string, vector<int>> 
          ("prior_bias", returnIncrVector(A_rows_count,6)));
  A_rows_count += 6;

  // return the count of the rows of A
  return A_rows_count;
}

// -------------------------------------------------------
ConstantTwistScenario createConstantTwistScenario(double radius, double linear_velocity) {
  /*
  creates the ground truth from where we simulate the measurements and measure the error
  */

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


// -------------------------------------------------------
std::vector<Point3>  createLandmarks(double radius){
  /*
  creates the map of landmarks and stores them in a vector of 3D points
  */

  double distance = radius + radius/10;
  std::vector<Point3> landmarks;
  landmarks.push_back( Point3(distance, 0, 0) );  
  landmarks.push_back( Point3(0, distance, 0) );
  landmarks.push_back( Point3(-distance, 0, 0) );
  landmarks.push_back( Point3(0, -distance, 0) );
  
  return landmarks;
}


// -------------------------------------------------------
std::vector<int> returnIncrVector(int start, int num_elem){
  /*
  return int vector with increasing values
  */

  vector<int> v(num_elem);
  iota (begin(v), end(v), start);

  return v;
}

// -------------------------------------------------------
void eliminateFactorsByType_old(
          boost::shared_ptr<GaussianFactorGraph> &lin_graph,
          vector<string> factor_types,
          string type){
  /*
  eliminate all factors with this type
  */

  typedef FastVector<boost::shared_ptr<GaussianFactor>>::iterator 
                              sharedGaussianFactorIterator;

  sharedGaussianFactorIterator it = lin_graph->begin();
  int num_elim_factors = 0;
  for (int i = 0; i < factor_types.size(); ++i){
    cout<< "factor "<< i<< " is of type "<< factor_types[i]<< endl;
    if (factor_types[i] == type) {
      lin_graph->erase(it + i - num_elim_factors);
      cout<< "erase factor of type "<< type<< endl;
      ++num_elim_factors;
    }
  }
}


// -------------------------------------------------------
Matrix eliminateFactorsByType(Matrix &M,
              map<string, vector<int>> &A_rows_per_type,
              string type){
  /*
  extract the matrix corresponding to the measurements of this type
  */
  return extractJacobianRows(M, A_rows_per_type[type]);
}


// -------------------------------------------------------
Matrix extractJacobianRows(Matrix &M, vector<int> &row_inds){
  /*
  extracte rows from Jacobians
  */

  Matrix h_M( row_inds.size(), row_inds.size() );
  for (int i = 0; i < row_inds.size(); ++i){
    for (int j = 0; j < row_inds.size(); ++j){
      h_M(i,j) = M(row_inds[i], row_inds[j]);
    }
  }
  return h_M;
}


// -------------------------------------------------------
Matrix extractMatrixRows(Matrix &A, vector<int> &row_inds){
  /*
  extracts the rows from the matrix
  */

  Matrix B(row_inds.size(), A.cols());
  for (int i = 0; i < row_inds.size(); ++i){
    for (int j = 0; j < B.cols(); ++j){
      B(i,j)= A(row_inds[i], j);      
    }
  }
  return B;
}


// -------------------------------------------------------
Matrix extractMatrixColumns(Matrix &A, vector<int> &col_inds){
  /*
  extracts the rows from the matrix
  */

  Matrix B(A.rows(), col_inds.size());
  for (int j = 0; j < col_inds.size(); ++j){
    for (int i = 0; i < B.rows(); ++i){
      B(i,j)= A(i, col_inds[j]);      
    }
  }
  return B;
}

// -------------------------------------------------------
void printIntVector(vector<int> v){
  /*
  Print vector of ints
  */

  for (auto i = v.begin(); i != v.end(); ++i)
    cout << *i << ' ';
  cout<< '\n';
}


// -------------------------------------------------------
void addLidarFactor(NonlinearFactorGraph &newgraph,
					RangeBearingFactorMap &range_bearing_factor,
					map<string, vector<int>> &A_rows_per_type, 
					int &A_rows_count){
  /*
  add lidar factor
  */

  newgraph.add(range_bearing_factor);
  vector<int> lidar_rows= returnIncrVector(A_rows_count, 3);
  A_rows_per_type["lidar"].insert( A_rows_per_type["lidar"].end(),
        	  lidar_rows.begin(), lidar_rows.end() );
  A_rows_count += 3;
}


// -------------------------------------------------------
void addGPSFactor(NonlinearFactorGraph &newgraph,
			   GPSFactor &gps_factor,
				 map<string, vector<int>> &A_rows_per_type, 
				 int &A_rows_count) {
  /*
  add GPS factor
  */

	newgraph.add(gps_factor);
	vector<int> gps_rows=  returnIncrVector(A_rows_count, 3);
	A_rows_per_type["gps"].insert( A_rows_per_type["gps"].end(),
				gps_rows.begin(), gps_rows.end() );
	A_rows_count += 3;
}


// -------------------------------------------------------
void addOdomFactor(NonlinearFactorGraph &newgraph,
		    CombinedImuFactor &imufac,
			  map<string, vector<int>> &A_rows_per_type, 
			  int &A_rows_count) {
  /*
  add odometry factor
  */

	newgraph.add(imufac);
	vector<int> odom_rows=  returnIncrVector(A_rows_count, 15);
	A_rows_per_type["odom"].insert( A_rows_per_type["odom"].end(),
	     	    odom_rows.begin(), odom_rows.end() );
	A_rows_count += 15;
}


// -------------------------------------------------------
void printMatrix(Matrix A){
  /*
  print matrix
  */
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
  cout<< A.format(CleanFmt)<< endl;
}


// -------------------------------------------------------
Pose3 compute_error(Pose3 true_pose,
					Pose3 estimated_pose){
  /*
  compute error
  */
	Rot3 rotation_error = true_pose.rotation() *
	  	   Rot3( estimated_pose.rotation().transpose() );
	Point3 translation_error = true_pose.translation() - 
							estimated_pose.translation();
	return Pose3( rotation_error, translation_error );
}


// -------------------------------------------------------
RangeBearingMeasurement sim_lidar_msmt(ConstantTwistScenario &scenario,
                    Point3 &landmark,
                    double time,
                    Params &params,
                    std::default_random_engine noise_generator){
  /*
  generate lidar msmts
  */
  
  // range
  double range = scenario.pose(time).range(landmark);
  double range_noise = params.noise_dist["range"](noise_generator);
  range = range + range_noise;

  // bearing
  Unit3 bearing = scenario.pose(time).bearing(landmark);
  Rot3 bearing_noise = Rot3::RzRyRx(params.noise_dist["bearing"](noise_generator),
                                    params.noise_dist["bearing"](noise_generator),
                                    params.noise_dist["bearing"](noise_generator));
  bearing = bearing_noise.rotate(bearing);

  RangeBearingMeasurement msmt(range, bearing);

  return msmt;
}


