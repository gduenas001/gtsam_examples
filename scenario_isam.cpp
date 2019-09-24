// TODO 
// - need to figure out how the imu noise is specified: continuous - discrete time conversion.
// - data association for landmarks
// - different frequencies for GPS and lidar
// - use sliding window filter
// - Obtain S matrix
// - odom M matrix is rank 6 because of the actual number of measurements
// - use a parser for the options

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <typeinfo>
#include <boost/math/distributions/chi_squared.hpp>


#include <helpers.h>


using namespace std;
using namespace gtsam;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // run time options
  float sim_time;
  if (argc == 2){
    sim_time= atof(argv[1]);
  } else {
    sim_time= 5; // seconds
  }

  // input parameters
  double accel_noise_sigma = 0.1; // what units?? 0003924
  double gyro_noise_sigma = 0.1; //000205689024915
  double accel_bias_rw_sigma = 0.005; //1e-20; //004905
  double gyro_bias_rw_sigma = 0.000001454441043; //1e-20; //000001454441043
  double gps_noise_sigma = 1.0; // meters
  double dt_imu = 1.0 / 200, // default 125HZ -- makes for 10 degrees per step (1.0 / 18)
         dt_gps = 1.0, // seconds
         scenario_radius = 30, // meters
         scenario_linear_vel = 50 / 3.6, // m/s
         range_noise_sigma = 0.50, // range standard deviation
         bearing_noise_sigma = 5 * M_PI / 180; // bearing standard dev

  // create parameters
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma,2);
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma,2);
  Matrix33 integration_error_cov = I_3x3 * 1e-20; // error committed in integrating position from velocities, 8 
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-20; // error in the bias used for preintegration, 5

  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_params = 
                              PreintegratedCombinedMeasurements::Params::MakeSharedU(kGravity);
  imu_params->setAccelerometerCovariance(measured_acc_cov);
  imu_params->setGyroscopeCovariance(measured_omega_cov);
  imu_params->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  imu_params->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  imu_params->biasAccOmegaInt = bias_acc_omega_int;
  imu_params->setIntegrationCovariance(integration_error_cov);
  imu_params->setUse2ndOrderCoriolis(false);
  imu_params->setOmegaCoriolis(Vector3(0, 0, 0));
  PreintegratedCombinedMeasurements accum(imu_params);
  std::default_random_engine noise_generator;    // noise generator
  std::normal_distribution<double> accel_noise_dist(0, accel_noise_sigma);
  std::normal_distribution<double> gyro_noise_dist(0, gyro_noise_sigma);
  std::normal_distribution<double> range_noise_dist(0, range_noise_sigma);
  std::normal_distribution<double> bearing_noise_dist(0, bearing_noise_sigma);
  std::normal_distribution<double> gps_noise_dist(0, gps_noise_sigma);
  noiseModel::Diagonal::shared_ptr gps_cov = noiseModel::Isotropic::Sigma(3, gps_noise_sigma); // GPS covariance is constant
  ConstantTwistScenario scenario = createConstantTwistScenario(scenario_radius, scenario_linear_vel);
  noiseModel::Diagonal::shared_ptr lidar_cov = noiseModel::Diagonal::Sigmas( (Vector(3) << bearing_noise_sigma, bearing_noise_sigma, range_noise_sigma).finished() );
  vector<Point3> landmarks = createLandmarks(scenario_radius);
  ISAM2Params isam_params;
  isam_params.evaluateNonlinearError = true;
 

  // Create a factor graph &  ISAM2
  NonlinearFactorGraph newgraph;
  ISAM2 isam(isam_params);
  Values initialEstimate, result; // Create the initial estimate to the solution

  // initialize variables
  double gps_time_accum = 0.0,
         current_time = 0.0,
         num_imu_epochs = sim_time / dt_imu;
  int pose_factor_count = 1;
  NavState prev_state, predict_state;
  imuBias::ConstantBias prev_bias;
  vector<Point3> true_positions;
  vector<Pose3> online_error; // error when computed online
  true_positions.push_back( scenario.pose(0).translation() );
  ISAM2Result isam_result;
  map<string, vector<int>> A_rows_per_type; // stores wich msmts to which hypothesis
  A_rows_per_type.insert( pair<string, vector<int>> ("lidar", {}) );
  A_rows_per_type.insert( pair<string, vector<int>> ("odom", {}) );
  A_rows_per_type.insert( pair<string, vector<int>> ("gps", {}) );

  // solve the graph once
  vector<string> factor_types; // stores the factor types (odom, GPS, lidar)
  int A_rows_count = addNoiselessPriorFactor(newgraph,
                 		          factor_types,
                        		  initialEstimate, 
                          		  scenario,
                          		  A_rows_per_type);
  isam.update(newgraph, initialEstimate);
  result = isam.calculateEstimate();
  newgraph = NonlinearFactorGraph();
  initialEstimate.clear();

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 1; i < num_imu_epochs; ++i) {

    current_time = i * dt_imu;
    gps_time_accum += dt_imu;
      
    // Predict acceleration and gyro measurements in (actual) body frame
    Point3 acc_noise = generate_random_point( noise_generator, accel_noise_dist );
    Vector3 measuredAcc = scenario.acceleration_b(current_time) -
                          scenario.rotation(current_time).transpose() * imu_params->n_gravity;
                          // acc_noise.vector();
    Point3 gyro_noise = generate_random_point( noise_generator, gyro_noise_dist );                      
    Vector3 measuredOmega = scenario.omega_b(current_time); // + gyro_noise.vector();
    accum.integrateMeasurement(measuredAcc, measuredOmega, dt_imu);

    // GPS update
    if (gps_time_accum > dt_gps) {

      // save the current position
      true_positions.push_back( scenario.pose(current_time).translation() );

      // predict from IMU accumulated msmts
      prev_state = NavState(result.at<Pose3>(X(pose_factor_count-1)), result.at<Vector3>(V(pose_factor_count-1)));
      prev_bias = result.at<imuBias::ConstantBias>(B(pose_factor_count-1));
      predict_state = accum.predict(prev_state, prev_bias);

      // predicted init values
      initialEstimate.insert(X(pose_factor_count), predict_state.pose());
      initialEstimate.insert(V(pose_factor_count), predict_state.velocity());
      initialEstimate.insert(B(pose_factor_count), imuBias::ConstantBias());  

      // Add Imu Factor
      CombinedImuFactor imufac(X(pose_factor_count - 1), V(pose_factor_count - 1), 
                               X(pose_factor_count),     V(pose_factor_count), 
                               B(pose_factor_count - 1), B(pose_factor_count), accum);
      addOdomFactor(newgraph,
                    imufac,
                    factor_types, 
                    A_rows_per_type, 
                    A_rows_count);
   
      // Adding GPS factor
      Point3 gps_noise(generate_random_point(noise_generator, gps_noise_dist));
      Point3 gps_msmt = scenario.pose(current_time).translation() + gps_noise;
      GPSFactor gps_factor(X(pose_factor_count), gps_msmt, gps_cov);
      addGPSFactor(newgraph,
                   gps_factor,
                   factor_types,
                   A_rows_per_type,
                   A_rows_count);

      // lidar measurements
      for (int j = 0; j < landmarks.size(); ++j) {
        Eigen::MatrixXd range_jacobian;
        Eigen::MatrixXd bearing_jacobian;
        
        // range
        double range = scenario.pose(current_time).range(landmarks[j], range_jacobian);
        double range_noise = range_noise_dist(noise_generator);
        range = range + range_noise;

        // bearing
        Unit3 bearing = scenario.pose(current_time).bearing(landmarks[j], bearing_jacobian);
        Rot3 bearing_noise = Rot3::RzRyRx(bearing_noise_dist(noise_generator),
                                          bearing_noise_dist(noise_generator),
                                          bearing_noise_dist(noise_generator));
        bearing = bearing_noise.rotate(bearing);

        // range-bearing factor
        RangeBearingFactorMap range_bearing_factor(X(pose_factor_count), 
                                                   range,
                                                   bearing,
                                                   landmarks[j], 
                                                   lidar_cov);

        addLidarFactor(newgraph,
                       range_bearing_factor,
                       factor_types, 
                       A_rows_per_type, 
                       A_rows_count);
      }      
      
      // Incremental solution
      isam_result = isam.update(newgraph, initialEstimate);
      result = isam.calculateEstimate();


      // compute error
      online_error.push_back(compute_error(scenario.pose(current_time),
                            result.at<Pose3>(X(pose_factor_count)) ));

	    // reset variables
      newgraph = NonlinearFactorGraph();
      accum.resetIntegration();
      initialEstimate.clear();
      gps_time_accum = 0.0;
      pose_factor_count++;  // increase the factor count
    }
  } // end for loop
  



  // check residuals
  boost::optional<double> error_after = isam_result.errorAfter;
  cout<< "error after: "<< error_after.value_or(-1)<< endl;
  

  // print the error for all the factor 
  NonlinearFactorGraph factor_graph = isam.getFactorsUnsafe();
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph = factor_graph.linearize(result);
  Matrix A = (lin_graph->jacobian()).first;
  cout<< "Jacobian matrix, A size = "<< A.rows()<< " x "<< A.cols()<< endl;


  cout<< "----------- Hypothesis 0 ----------"<< "\n\n";
  // number of measurements and states
  double n = A.rows(); double m = A.cols();
  cout<< "n = "<< n<< "\nm = "<< m<< endl;

  // check matrix M before elimination
  Matrix Lambda = (lin_graph->hessian()).first;
  Matrix S = Lambda.inverse() * A.transpose();
  Matrix M = (Eigen::MatrixXd::Identity(n, n) - A*S);
  Eigen::FullPivLU<Matrix> M_lu(M);
  M_lu.setThreshold(1e-5);
  cout<< "size of M = "<< M.rows() << " x "<< M.cols()<< endl;
  cout << "rank of M is " << M_lu.rank() << endl;


  // loop over hypotheses
  for (map<string, vector<int>>::iterator it= A_rows_per_type.begin(); 
       it != A_rows_per_type.end(); 
       ++it){
    string type = it->first;

    cout<< "----------- Hypothesis "<< type <<" ----------"<< "\n\n";

    cout<< "Rows to be extracted: "<< endl; 
    printIntVector( it->second );

    Matrix h_M = extractJacobianRows(M, it->second);
    Eigen::FullPivLU<Matrix> h_M_lu(h_M);
    h_M_lu.setThreshold(1e-5);
    cout<< "size of M is = "<< h_M.rows() << " x "<< h_M.cols()<< endl;
    cout << "rank of M is " << h_M_lu.rank() << endl; 
  }
 




  // Vector error_vector = factor_graph.at(0)->unwhitenedError(result);
  // Vector error_vector = factor->unwhitenedError(result);
  // cout<< "error in selected factor: "<< factor->error(result)<< endl;
  // cout<< error_vector<< endl;
  


  boost::math::chi_squared_distribution<> chi2_dist(4);
  cout<< "cdf value: "<< boost::math::quantile(chi2_dist, 1-1e-7)<< endl;


  // save the data TODO: give option to save in different folder
  saveData(result,
           true_positions,
           landmarks,
           online_error);


  // // print path with python
  // string command = "python ../python_plot.py";
  // system(command.c_str());


  // save factor graph as graphviz dot file
  // Use this to convert to png image
  // dot -Tpng -Gdpi=1000 isam_example.dot -o isam_example.png
  ofstream os("isam_example.dot");
  factor_graph.saveGraph(os, result);


  // GTSAM_PRINT(result);


  return 0;
}
/* ************************************************************************* */
