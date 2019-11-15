

#include "helpers.h"

using namespace std;
using namespace gtsam;


// -------------------------------------------------------
void save_data(Values result,
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
  Vector6 ave_error = error_average(online_error);
  stream << ave_error[0] * 180/M_PI <<","<< ave_error[1] * 180/M_PI <<","<< ave_error[2] * 180/M_PI<<","
         << ave_error[3] <<","<< ave_error[4] <<","<< ave_error[5]<< endl;
         
  stream.close();
}


// -------------------------------------------------------
Point3 generate_random_point(std::default_random_engine &generator, std::normal_distribution<double> &distribution) {
  return Point3(distribution(generator),
                distribution(generator),
                distribution(generator));
}


// -------------------------------------------------------
// -------------------------------------------------------
Vector6 error_average(std::vector<Pose3> poses){
 
  Vector3 ave_translation, ave_rotation; // deault constructor -> zero translation
  for (std::vector<Pose3>::iterator it = poses.begin() ; it != poses.end(); ++it) {
    ave_translation += abs( it->translation() );
    ave_rotation += abs(Vector3( it->rotation().roll(),
                                 it->rotation().pitch(),
                                 it->rotation().yaw() ));
  }
  return (Vector(6) << ave_rotation / poses.size(),
                       ave_translation / poses.size() ).finished();
}





// -------------------------------------------------------
// -------------------------------------------------------
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


// -------------------------------------------------------
// -------------------------------------------------------
std::vector<Point3>  createLandmarks(double radius){

  double distance = radius + radius/10;
  std::vector<Point3> landmarks;
  landmarks.push_back( Point3(distance, 0, 0) );  
  landmarks.push_back( Point3(0, distance, 0) );
  landmarks.push_back( Point3(-distance, 0, 0) );
  landmarks.push_back( Point3(0, -distance, 0) );
  
  return landmarks;
}


// -------------------------------------------------------
// -------------------------------------------------------
std::vector<int> returnIncrVector(int start, int num_elem){

  vector<int> v(num_elem);
  iota (begin(v), end(v), start);

  return v;
}



// -------------------------------------------------------
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
// -------------------------------------------------------
Matrix extractMatrixRows(Matrix &A, vector<int> &row_inds){

  Matrix B(row_inds.size(), A.cols());
  for (int i = 0; i < row_inds.size(); ++i){
    for (int j = 0; j < B.cols(); ++j){
      B(i,j)= A(row_inds[i], j);      
    }
  }
  return B;
}


// -------------------------------------------------------
// -------------------------------------------------------
Matrix extractMatrixColumns(Matrix &A, vector<int> &col_inds){

  Matrix B(A.rows(), col_inds.size());
  for (int j = 0; j < col_inds.size(); ++j){
    for (int i = 0; i < B.rows(); ++i){
      B(i,j)= A(i, col_inds[j]);      
    }
  }
  return B;
}


// -------------------------------------------------------
// -------------------------------------------------------
Matrix extractMatrixRowsAndColumns(Matrix &A, 
								   vector<int> &row_inds, 
								   vector<int> &col_inds){

  Matrix B( row_inds.size(), col_inds.size() );
  for (int i = 0; i < row_inds.size(); ++i){
    for (int j = 0; j < col_inds.size(); ++j){
      B(i,j) = A(row_inds[i], col_inds[j]);
    }
  }
  return B;
}


// -------------------------------------------------------
// -------------------------------------------------------
void printIntVector(vector<int> v){

  for (auto i = v.begin(); i != v.end(); ++i)
    cout << *i << ' ';
  cout<< '\n';
}



// -------------------------------------------------------
// -------------------------------------------------------
void printMatrix(Matrix A){
  
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
  cout<< A.format(CleanFmt)<< endl;
}


// -------------------------------------------------------
// -------------------------------------------------------
Pose3 compute_error(Pose3 true_pose,
					Pose3 estimated_pose){
  
	Rot3 rotation_error = true_pose.rotation() *
	  	   Rot3( estimated_pose.rotation().transpose() );
	Point3 translation_error = true_pose.translation() - 
							estimated_pose.translation();
	return Pose3( rotation_error, translation_error );
}


// -------------------------------------------------------
// -------------------------------------------------------
RangeBearingMeasurement sim_lidar_msmt(ConstantTwistScenario &scenario,
                    Point3 &landmark,
                    double time,
                    Params &params,
                    std::default_random_engine noise_generator,
                    bool is_noisy){
  
  // range
  double range = scenario.pose(time).range(landmark);
  if (is_noisy){
    double range_noise= params.noise_dist["range"](noise_generator);
    range= range + range_noise;
  }

  // bearing
  Unit3 bearing = scenario.pose(time).bearing(landmark);
  if (is_noisy){
    Rot3 bearing_noise = Rot3::RzRyRx(params.noise_dist["bearing"](noise_generator),
                                    params.noise_dist["bearing"](noise_generator),
                                    params.noise_dist["bearing"](noise_generator));
    bearing = bearing_noise.rotate(bearing);
  }
  
  // create measurement object
  RangeBearingMeasurement msmt(range, bearing);
  return msmt;
}


// -------------------------------------------------------
// -------------------------------------------------------
double get_dof_from_graph(const NonlinearFactorGraph &graph){
  int dim= 0;
  for (auto factor : graph){
    if (!factor) {continue;}

    if (factor->dim() == 15){
      dim += 12;
    }else{
      dim += factor->dim();
    }
  }
  return dim;
}


// -------------------------------------------------------
// -------------------------------------------------------
map<string, Vector> buildt_vector(int size){

	map<string, Vector> t_vector;
	t_vector["x"]= Vector::Zero(size);
	t_vector["y"]= Vector::Zero(size);
	t_vector["z"]= Vector::Zero(size);
	t_vector["x"](size-10)= 1;
	t_vector["y"](size-9)= 1;
	t_vector["z"](size-8)= 1;

	return t_vector;
}


// -------------------------------------------------------
// -------------------------------------------------------
map<string,double> getVariancesForLastPose(ISAM2 &isam,
										   Counters &counters){
	map<string,double> var;
	Matrix P_x= isam.marginalCovariance(X(counters.current_factor));
	var["roll"]= P_x(0,0);
	var["pitch"]= P_x(1,1);
	var["yaw"]= P_x(2,2);
	var["x"]= P_x(3,3);
	var["y"]= P_x(4,4);
	var["z"]= P_x(5,5);

	return var;
}


// -------------------------------------------------------
// -------------------------------------------------------
map<string,double> get_variances_for_last_pose(
                  IncrementalFixedLagSmoother fixed_lag_smoother,
                  // BatchFixedLagSmoother fixed_lag_smoother,
                  Counters &counters){

  map<string,double> var;
  Matrix P_x= fixed_lag_smoother.marginalCovariance(X(counters.current_factor));
  var["roll"]= P_x(0,0);
  var["pitch"]= P_x(1,1);
  var["yaw"]= P_x(2,2);
  var["x"]= P_x(3,3);
  var["y"]= P_x(4,4);
  var["z"]= P_x(5,5);

  return var;
}


// -------------------------------------------------------
// -------------------------------------------------------
gtsam::Point3
sim_gps_msmt(const Point3 &true_position,
             std::default_random_engine &noise_generator, 
             std::normal_distribution<double> &gps_distribution,
             bool is_noisy){

  if (is_noisy){
    Point3 gps_noise(generate_random_point(noise_generator, gps_distribution));
    return true_position + gps_noise;
  }else{
    return true_position;
  }
}


// -------------------------------------------------------
// -------------------------------------------------------
Vector3 sim_imu_acc(ConstantTwistScenario &scenario,
             std::default_random_engine &noise_generator, 
             std::normal_distribution<double> &imu_acc_dist,
             gtsam::Vector3 g,
             double time,
             bool is_noisy){

  Vector3 msmt_acc = scenario.acceleration_b(time) -
                     scenario.rotation(time).transpose() * g;
  if (is_noisy){
    Point3 acc_noise = generate_random_point( noise_generator, imu_acc_dist );
    msmt_acc= msmt_acc + acc_noise.vector();
  }

  return msmt_acc;
}


// -------------------------------------------------------
// -------------------------------------------------------
Vector3 sim_imu_w(Vector3 true_imu_w,
             std::default_random_engine &noise_generator, 
             std::normal_distribution<double> &imu_gyro_dist,
             bool is_noisy){

  if (is_noisy){
    Point3 gyro_noise = generate_random_point( noise_generator, imu_gyro_dist );                      
    return true_imu_w + gyro_noise.vector();
  }else{
    return true_imu_w;
  }
}


// -------------------------------------------------------
// -------------------------------------------------------
int return_first_element(const pair<int, double> &p){
    return p.first;
}




// -------------------------------------------------------
/*
deprecated
*/
// -------------------------------------------------------


// -------------------------------------------------------
// -------------------------------------------------------
void eliminateFactorsByType_old(
          boost::shared_ptr<GaussianFactorGraph> &lin_graph,
          vector<string> factor_types,
          string type){

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
// -------------------------------------------------------
double getDOFfromGraph(map<string, vector<int>> &A_rows_per_type){

  double dof= 0;
  for (map<string, vector<int>>::iterator it= A_rows_per_type.begin(); 
       it != A_rows_per_type.end(); 
       ++it){

    string factor_type = it->first;
    double factor_n= (it->second).size();
    dof += getDOFfromFactorType(factor_n, factor_type);
  }
  return dof;
}


// -------------------------------------------------------
// -------------------------------------------------------
double getDOFfromFactorType(int n, string type){
  if (type == "odom"){
    int num_odom_factors= n / 15;
      return n - (num_odom_factors * 3);
    }else{
      return n;
    }
}
