
#include "helpers.h"

using namespace std;
using namespace gtsam;

// generate a random 3D point
Point3 generate_random_point(std::default_random_engine &generator, std::normal_distribution<double> &distribution) {
  return Point3(distribution(generator),distribution(generator),distribution(generator));
}
