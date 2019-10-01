
#include <gtsam/geometry/SimpleCamera.h>


class RangeBearingMeasurement{
  public:

    RangeBearingMeasurement(double range, Unit3 bearing){
      this->range= range;
      this->bearing= bearing;
    }
    double range;
    Unit3 bearing;
};