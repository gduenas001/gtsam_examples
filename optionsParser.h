
#include <iostream>
#include <getopt.h>
#include <cmath>

struct Params{
  double accel_noise_sigma = 0.1; // what units?? 0003924
  double gyro_noise_sigma = 0.1; //000205689024915
  double accel_bias_rw_sigma = 0.005; //1e-20; //004905
  double gyro_bias_rw_sigma = 0.000001454441043; //1e-20; //000001454441043
  double gps_noise_sigma = 1.0; // meters
  double dt_imu = 1.0 / 200; // default 125HZ -- makes for 10 degrees per step (1.0 / 18)
  double dt_gps = 1.0; // seconds
  double scenario_radius = 30; // meters
  double scenario_linear_vel = 50 / 3.6; // m/s
  double range_noise_sigma = 0.50; // range standard deviation
  double bearing_noise_sigma = 5 * M_PI / 180; // bearing standard dev
  double sim_time= 2; // seconds -- time to run the simulation
};

int optionsParser (int argc, char **argv, Params &params);
