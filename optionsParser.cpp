
#include "optionsParser.h"

using namespace std;
using namespace gtsam;

struct option long_opt[] =
{
  {"accel_noise_sigma", required_argument, NULL, 'a'},
  {"gyro_noise_sigma", required_argument, NULL, 'b'},
  {"accel_bias_rw_sigma", required_argument, NULL, 'c'},
  {"gyro_bias_rw_sigma", required_argument, NULL, 'd'},
  {"gps_noise_sigma", required_argument, NULL, 'e'},
  {"dt_imu", required_argument, NULL, 'f'},
  {"dt_gps", required_argument, NULL, 'g'},
  {"scenario_radius", required_argument, NULL, 'h'},
  {"scenario_linear_vel", required_argument, NULL, 'i'},
  {"range_noise_sigma", required_argument, NULL, 'j'},
  {"bearing_noise_sigma", required_argument, NULL, 'k'},
  {"sim_time", required_argument, NULL, 'l'},
  {"evaluateNonlinearError", required_argument, NULL, 'm'},
  {"P_lambda", required_argument, NULL, 'n'},
  {"AL_x", required_argument, NULL, 'o'},
  {"AL_y", required_argument, NULL, 'p'},
  {"AL_z", required_argument, NULL, 'q'},
  {"is_noisy_gps", required_argument, NULL, 'r'},
  {"is_noisy_lidar", required_argument, NULL, 's'},
  {"is_noisy_imu", required_argument, NULL, 't'},
  {"prior_position_noise_sigma", required_argument, NULL, 'u'},
  {"prior_orientation_noise_sigma", required_argument, NULL, 'v'},
  {"is_noisy_prior", required_argument, NULL, 'w'},
  {"seed", required_argument, NULL, 'x'},


  {NULL,   0,                 NULL, 0  }
};



int optionsParser (int argc, char **argv, Params &params){
  int opt;
  const char* short_opt = "a:b:c:d:e:f:g:h:i:j:k:";


  while((opt = getopt_long(argc, argv, short_opt, long_opt, NULL)) != -1)
  {
    switch(opt)
    {
       case -1:       /* no more arguments */
       case 0:        /* long options toggles */
       break;

       case 'a':
         printf("you entered \"%s\" for the variable 'accel_noise_sigma'\n", optarg);
         params.accel_noise_sigma= atof(optarg);
       break;

       case 'b':
         printf("you entered \"%s\" for the variable 'gyro_noise_sigma'\n", optarg);
         params.gyro_noise_sigma= atof(optarg);
       break;

       case 'c':
         printf("you entered \"%s\" for the variable 'accel_bias_rw_sigma'\n", optarg);
         params.accel_bias_rw_sigma= atof(optarg);
       break;

       case 'd':
         printf("you entered \"%s\" for the variable 'gyro_bias_rw_sigma'\n", optarg);
         params.gyro_bias_rw_sigma= atof(optarg);
       break;

       case 'e':
         printf("you entered \"%s\" for the variable 'gps_noise_sigma'\n", optarg);
         params.gps_noise_sigma= atof(optarg);
       break;

       case 'f':
         printf("you entered \"%s\" for the variable 'dt_imu'\n", optarg);
         params.dt_imu= atof(optarg);
       break;

       case 'g':
         printf("you entered \"%s\" for the variable 'dt_gps'\n", optarg);
         params.dt_gps= atof(optarg);
       break;

       case 'h':
         printf("you entered \"%s\" for the variable 'scenario_radius'\n", optarg);
         params.scenario_radius= atof(optarg);
       break;

       case 'i':
         printf("you entered \"%s\" for the variable 'scenario_linear_vel'\n", optarg);
         params.scenario_linear_vel= atof(optarg);
       break;

       case 'j':
         printf("you entered \"%s\" for the variable 'range_noise_sigma'\n", optarg);
         params.range_noise_sigma= atof(optarg);
       break;

       case 'k':
         printf("you entered \"%s\" for the variable 'bearing_noise_sigma'\n", optarg);
         params.bearing_noise_sigma= atof(optarg);
       break;

       case 'l':
         printf("you entered \"%s\" for the variable 'sim_time'\n", optarg);
         params.sim_time= atof(optarg);
       break;

       case 'm':
         printf("you entered \"%s\" for the variable 'evaluateNonlinearError'\n", optarg);
         params.evaluate_nonlinear_error= optarg;
       break;

       case 'n':
         printf("you entered \"%s\" for the variable 'P_lambda'\n", optarg);
         params.P_lambda= atof(optarg);
       break;

       case 'o':
         printf("you entered \"%s\" for the variable 'AL_x'\n", optarg);
         params.AL_x= atof(optarg);
       break;

       case 'p':
         printf("you entered \"%s\" for the variable 'AL_y'\n", optarg);
         params.AL_y= atof(optarg);
       break;

       case 'q':
         printf("you entered \"%s\" for the variable 'AL_z'\n", optarg);
         params.AL_z= atof(optarg);
       break;

       case 'r':
         printf("you entered \"%s\" for the variable 'is_noisy_gps'\n", optarg);
         if ( strcmp(optarg, "false") == 0 ){
           params.is_noisy_gps= false;
         }
       break;

       case 's':
         printf("you entered \"%s\" for the variable 'is_noisy_lidar'\n", optarg);
         params.is_noisy_lidar= atof(optarg);
       break;

       case 't':
         printf("you entered \"%s\" for the variable 'is_noisy_imu'\n", optarg);
         params.is_noisy_imu= atof(optarg);
       break;

       case 'u':
         printf("you entered \"%s\" for the variable 'prior_position_noise_sigma'\n", optarg);
         params.prior_position_noise_sigma= atof(optarg);
       break;

       case 'v':
         printf("you entered \"%s\" for the variable 'prior_orientation_noise_sigma'\n", optarg);
         params.prior_orientation_noise_sigma= atof(optarg);
       break;

       case 'w':
         printf("you entered \"%s\" for the variable 'is_noisy_prior'\n", optarg);
         if ( strcmp(optarg, "false") == 0 ){
          params.is_noisy_prior= false;
         }
       break;

       case 'x':
         printf("you entered \"%s\" for the variable 'seed'\n", optarg);
         params.seed= atoi(optarg);
       break;

       // default error
       default:
       fprintf(stderr, "%s: invalid option -- %c\n", argv[0], opt);
       return (-2);
    };
  };

  return (0);
}





// create varibles from parameters
void build_variables(Params &params){
  
  // create parameters
  params.lidar_cov= noiseModel::Diagonal::Sigmas( (Vector(3)<< 
               params.bearing_noise_sigma, 
               params.bearing_noise_sigma, 
               params.range_noise_sigma).finished() );

  params.gps_cov=   noiseModel::Diagonal::Sigmas( (Vector(3)<<
               params.gps_noise_sigma,
               params.gps_noise_sigma,
               params.gps_noise_sigma).finished() );

  params.prior_pose_cov= noiseModel::Diagonal::Sigmas( (Vector(6) << 
               Vector3::Constant(params.prior_orientation_noise_sigma), 
               Vector3::Constant(params.prior_position_noise_sigma) ).finished() );

  params.prior_vel_cov= noiseModel::Diagonal::Sigmas( (Vector(3) << 
               Vector3::Constant(params.prior_vel_noise_sigma) ).finished() );

  params.prior_bias_cov= noiseModel::Diagonal::Sigmas( (Vector(6) << 
               Vector3::Constant(params.prior_bias_acc_noise_sigma), 
               Vector3::Constant(params.prior_bias_gyro_noise_sigma) ).finished() );


  params.measured_acc_cov = I_3x3 * pow(params.accel_noise_sigma,2);
  params.measured_omega_cov = I_3x3 * pow(params.gyro_noise_sigma,2);
  params.bias_acc_cov = I_3x3 * pow(params.accel_bias_rw_sigma,2);
  params.bias_omega_cov = I_3x3 * pow(params.gyro_bias_rw_sigma,2);
  params.integration_error_cov = I_3x3 * 1e-20; // error committed in integrating position from velocities, 8 
  params.bias_acc_omega_int = Matrix::Identity(6,6) * 1e-20; // error in the bias used for preintegration, 5


  params.imu_params= PreintegratedCombinedMeasurements::Params::MakeSharedU(params.k_gravity);
  params.imu_params->setAccelerometerCovariance(params.measured_acc_cov);
  params.imu_params->setGyroscopeCovariance(params.measured_omega_cov);
  params.imu_params->biasAccCovariance = params.bias_acc_cov; // acc bias in continuous
  params.imu_params->biasOmegaCovariance = params.bias_omega_cov; // gyro bias in continuous
  params.imu_params->biasAccOmegaInt = params.bias_acc_omega_int;
  params.imu_params->setIntegrationCovariance(params.integration_error_cov);
  params.imu_params->setUse2ndOrderCoriolis(false);
  params.imu_params->setOmegaCoriolis(Vector3(0, 0, 0));
  PreintegratedCombinedMeasurements accum_temp(params.imu_params); // TODO: clean this
  params.accum= accum_temp;


  params.noise_dist["acc"]= std::normal_distribution<double>(0, params.accel_noise_sigma);
  params.noise_dist["gyro"]= std::normal_distribution<double>(0, params.gyro_noise_sigma);
  params.noise_dist["range"]= std::normal_distribution<double>(0, params.range_noise_sigma);
  params.noise_dist["bearing"]= std::normal_distribution<double>(0, params.bearing_noise_sigma);
  params.noise_dist["gps"]= std::normal_distribution<double>(0, params.gps_noise_sigma);
  params.noise_dist["prior_position"]= std::normal_distribution<double>(0, params.prior_position_noise_sigma);
  params.noise_dist["prior_orientation"]= std::normal_distribution<double>(0, params.prior_orientation_noise_sigma);
  params.noise_dist["prior_vel"]= std::normal_distribution<double>(0, params.prior_vel_noise_sigma);
  params.noise_dist["prior_bias_acc"]= std::normal_distribution<double>(0, params.prior_bias_acc_noise_sigma);
  params.noise_dist["prior_bias_gyro"]= std::normal_distribution<double>(0, params.prior_bias_gyro_noise_sigma);

  params.is_noisy["gps"]= params.is_noisy_gps;
  params.is_noisy["lidar"]= params.is_noisy_lidar;
  params.is_noisy["imu"]= params.is_noisy_imu;
  params.is_noisy["prior"]= params.is_noisy_prior;
  

  // deprecated
  params.isam_params.evaluateNonlinearError= params.evaluate_nonlinear_error; // for the residuals
}



// --------------- for DEBUG purposes ---------------
// int main(int argc, char **argv){
//   Params params;
//   optionsParser(argc, argv, params);
//   std::cout<< params.range_noise_sigma<< endl;
//   return 0;
// }
