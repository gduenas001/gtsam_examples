
#pragma once

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include <cmath>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <random>
#include <gtsam/nonlinear/ISAM2.h>
#include <algorithm>

#include "Params.h"

using namespace gtsam;
using namespace std;


/*
loads the params from "params.txt" into a Params struct
TODO: remove default values from the params initilization
*/
Params load_params_from_file();


/*
adds the parameter read from the text file to the Params
struct
*/
bool add_parameter(std::string name, 
				   std::string value, 
				   Params &params);


/*
This function builds the parameters from the inputs given, 
e.g. from the inputs "AL_x", "AL_y", and "AL_z" it builds a 
map to access them as "params.AL["x"]", "params.AL["y"]", and "params.AL["z"]".
*/
void build_variables(Params &params);


/*
Function to parse the options at runtime
*/
int params_parser(int argc, char **argv, Params &params);


/*
Prints the params struct
NOTE: if you add a new value, you have to add it to
this function too b/c it doesn't automatically 
iterate through the parameters
*/
void print_params(const Params &params);

