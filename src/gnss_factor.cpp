#include "gnss_factor.h"

using namespace gtsam;

namespace fast_livo
{
GnssFactor::GnssFactor(Key poseKey,
                       const Point3 &meas,
                       const SharedNoiseModel &model)
    : NoiseModelFactor1<Pose3>(model, poseKey), meas_(meas) {}

Vector GnssFactor::evaluateError(const Pose3 &pose,
                                 boost::optional<Matrix &> H) const
{
  if (H)
  {
    Matrix H_tmp = Matrix::Zero(3, 6);
    H_tmp.block<3, 3>(0, 0) = Matrix3::Identity();
    *H = H_tmp;
  }
  return pose.translation() - meas_;
}
} // namespace fast_livo
