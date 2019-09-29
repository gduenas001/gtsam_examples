
#include "optionsParser.h"

using namespace std;

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


       // default error
       default:
       fprintf(stderr, "%s: invalid option -- %c\n", argv[0], opt);
       return (-2);
    };
  };

  return (0);
}


// int main(int argc, char **argv){
//   Params params;
//   optionsParser(argc, argv, params);
//   std::cout<< params.range_noise_sigma<< endl;
//   return 0;
// }
