
#include "RangeBearingFactorMap.h"

Vector RangeBearingFactorMap::evaluateError(const Pose3& q, boost::optional<Matrix&> H) const {

  Eigen::MatrixXd range_jacobian;
  Eigen::MatrixXd bearing_jacobian;
  
  double expected_range = q.range(landmark_, range_jacobian);
  // double expected_range = sqrt(pow( q.x() - landmark_.x(), 2 ) + 
  //                              pow( q.y() - landmark_.y(), 2 ) + 
  //                              pow( q.z() - landmark_.z(), 2 ) );
  double range_error = expected_range - range_msmt;

  Unit3 expected_bearing = q.bearing(landmark_, bearing_jacobian);  
  // Unit3 expected_bearing(landmark_.x() - q.x(), 
  //                        landmark_.y() - q.y(), 
  //                        landmark_.z() - q.z());
  Vector2 bearing_error = bearing_msmt.errorVector(expected_bearing);

  if (H) {
    Eigen::MatrixXd jacobian( 3, 6);
    jacobian << bearing_jacobian, range_jacobian;
    // (*H) = (Eigen::MatrixXd <<  bearing_jacobian, range_jacobian).finished();
    (*H) = jacobian;

  }
  return (Vector(3) << bearing_error, range_error).finished();
}
