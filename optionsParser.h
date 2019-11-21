
#pragma once


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
Function to parse the options at runtime
*/
int optionsParser (int argc, char **argv, Params &params);

/*
This function builds the parameters from the inputs given, 
e.g. from the inputs "AL_x", "AL_y", and "AL_z" it builds a 
map to access them as "params.AL["x"]", "params.AL["y"]", and "params.AL["z"]".
*/
void build_variables(Params &params);


/*
Load parameters from text file into params struct
*/
void load_params(Params &params);
