
#pragma once

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


using namespace gtsam;
using namespace std;

/*
 Factor for range-bearing measurements to a landmark
 in the prior map.
 It's different from the ones already implemented in 
 that there is no key for the landmarks here b/c they 
 are in the map and thus not estimated
*/
class RangeBearingFactorMap: public NoiseModelFactor1<Pose3> {
  public:
    double range_msmt;
    Unit3 bearing_msmt;
    Point3 landmark_;

  public:
    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    RangeBearingFactorMap(Key j, 
                          double range,
                          Unit3 bearing,
                          Point3 landmark, 
                          const SharedNoiseModel& noise_model):
    range_msmt(range), 
    bearing_msmt(bearing),
    landmark_(landmark),
    NoiseModelFactor1<Pose3>(noise_model, j) {}

    virtual ~RangeBearingFactorMap() {}

    // Vector evaluateError(const Pose3& q, boost::optional<Matrix&> H = boost::none) const;
    Vector evaluateError(const Pose3& q, boost::optional<Matrix&> H) const;
};