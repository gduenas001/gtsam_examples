
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <typeinfo>

#include <vector>

using namespace std;
using namespace gtsam;

// Shorthand for velocity and pose variables
using symbol_shorthand::V;
using symbol_shorthand::X;
using symbol_shorthand::B;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  boost::shared_ptr<PreintegrationParams> imu_params = PreintegrationParams::MakeSharedU(kGravity);
  imu_params->accelerometerCovariance = (I_3x3 * 0.1);
  imu_params->setGyroscopeCovariance(I_3x3 * 0.1);
  imu_params->setIntegrationCovariance(I_3x3 * 0.1);
  imu_params->setUse2ndOrderCoriolis(false);
  imu_params->setOmegaCoriolis(Vector3(0, 0, 0));

  // IMU preintegrator
  PreintegratedImuMeasurements accum(imu_params);

  // I think this is the error that we introduce artificially in the starting points
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));

  // Start with a camera on x-axis looking at origin (only use pose_0 from here to generate the scenario)
  double radius = 30;
  const Point3 up(0, 0, 1), target(0, 0, 0);
  const Point3 position(radius, 0, 0);
  const SimpleCamera camera = SimpleCamera::Lookat(position, target, up);
  const Pose3 pose_0 = camera.pose();

  // Now, create a constant-twist scenario that makes the camera orbit the origin
  double angular_velocity = M_PI,  // rad/sec
      delta_t = 1.0 / 18;          // makes for 10 degrees per step
  Vector3 angular_velocity_vector(0, -angular_velocity, 0);
  Vector3 linear_velocity_vector(radius * angular_velocity, 0, 0);
  ConstantTwistScenario scenario(angular_velocity_vector, linear_velocity_vector, pose_0);

  // Create a factor graph
  NonlinearFactorGraph newgraph;

  // Create (incremental) ISAM2 solver
  ISAM2 isam;

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 0.1 rad std on roll, pitch, yaw, 30cm std on x,y,z.
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  PriorFactor<Pose3> pose_prior(X(0), pose_0, pose_noise);
  newgraph.add(PriorFactor<Pose3>(X(0), pose_0, pose_noise));
  initialEstimate.insert(X(0), pose_0);

  // add velocity prior to graph and init values
  Vector velocity_prior(3); // needs to be a dynamically allocated vector (I don't know why)
  velocity_prior << 0, angular_velocity * radius, 0;
  noiseModel::Diagonal::shared_ptr velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  PriorFactor<Vector> vel_prior(V(0), velocity_prior, velnoise);
  newgraph.add(vel_prior);
  initialEstimate.insert(V(0), velocity_prior);


  // Add bias priors to graph and init values
  Key biasKey(B(0));
  noiseModel::Diagonal::shared_ptr biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
  PriorFactor<imuBias::ConstantBias> biasprior(biasKey, imuBias::ConstantBias(), biasnoise);
  newgraph.add(biasprior); // add or push_back seems to be the same
  initialEstimate.insert(biasKey, imuBias::ConstantBias());

  // IMU covariance always the same
  noiseModel::Diagonal::shared_ptr imu_cov = 
          noiseModel::Diagonal::Variances( (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished() );

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 0; i < 36; ++i) {
    double t = i * delta_t;
    if (i == 0) {  // First time add two poses
      Pose3 pose_1 = scenario.pose(delta_t);
      // initialEstimate.insert(X(0), pose_0.compose(delta)); // I think it's adding delta to make the initial estimate incorrect
      initialEstimate.insert(X(1), pose_1.compose(delta));
    } else if (i >= 2) {  // Add more poses as necessary
      Pose3 pose_i = scenario.pose(t);
      initialEstimate.insert(X(i), pose_i.compose(delta));
    }

    if (i > 0) {
      // Add Bias variables periodically
      if (i % 5 == 0) {
        biasKey++;
        Symbol b1 = biasKey - 1;
        Symbol b2 = biasKey;

        newgraph.emplace_shared<BetweenFactor<imuBias::ConstantBias> >(b1, b2, imuBias::ConstantBias(), imu_cov);
        initialEstimate.insert(biasKey, imuBias::ConstantBias());
      }
      // Predict acceleration and gyro measurements in (actual) body frame
      Vector3 measuredAcc = scenario.acceleration_b(t) -
                            scenario.rotation(t).transpose() * imu_params->n_gravity;
      Vector3 measuredOmega = scenario.omega_b(t);
      accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t);

      // Add Imu Factor
      ImuFactor imufac(X(i - 1), V(i - 1), X(i), V(i), biasKey, accum);
      newgraph.add(imufac);

      // insert new velocity, which is wrong
      initialEstimate.insert(V(i), velocity_prior);
      accum.resetIntegration();
    }

    // Incremental solution
    isam.update(newgraph, initialEstimate);
    result = isam.calculateEstimate();
    newgraph = NonlinearFactorGraph();
    initialEstimate.clear();
  } // end for loop
  
  GTSAM_PRINT(result);
  return 0;
}
/* ************************************************************************* */
