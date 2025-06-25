#ifndef FAST_LIVO_GNSS_FACTOR_H
#define FAST_LIVO_GNSS_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace fast_livo
{
class GnssFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Point3 meas_;

public:
  GnssFactor(gtsam::Key poseKey,
             const gtsam::Point3 &meas,
             const gtsam::SharedNoiseModel &model);

  gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                              boost::optional<gtsam::Matrix &> H) const override;
};
} // namespace fast_livo

#endif // FAST_LIVO_GNSS_FACTOR_H
