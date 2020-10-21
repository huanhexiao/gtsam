/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactorLA.cpp
 *  @author Jin
 *  @brief  Header file for GPS factor with Lever Arm(LA)
 *  @date   September 28, 2020
 **/

#include "GPSFactorLA.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void GPSFactorLA::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactorLA on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_ << "\n";
  noiseModel_->print("  noise model: ");
  if (this->body_t_) this->body_t_->print(" lever arm: ");
}

//***************************************************************************
bool GPSFactorLA::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol) &&
         ((!body_t_ && !e->body_t_) ||
          (body_t_ && e->body_t_ && body_t_->equals(*e->body_t_)));
}

//***************************************************************************
Vector GPSFactorLA::evaluateError(const Pose3& p,
    boost::optional<Matrix&> H) const {
  if (body_t_) {
    if (H) {
      H->resize(3, 6);
      *H << -p.rotation().matrix() * skewSymmetric(*body_t_),
          p.rotation().matrix();
    }
    return p.transformFrom(*body_t_) - nT_;
  } else {
    return p.translation(H) - nT_;
  }
}

//***************************************************************************
pair<Pose3, Vector3> GPSFactorLA::EstimateState(double t1, const Point3& NED1,
    double t2, const Point3& NED2, double timestamp) {
  // Estimate initial velocity as difference in NED frame
  double dt = t2 - t1;
  Point3 nV = (NED2 - NED1) / dt;

  // Estimate initial position as linear interpolation
  Point3 nT = NED1 + nV * (timestamp - t1);

  // Estimate Rotation
  double yaw = atan2(nV.y(), nV.x());
  Rot3 nRy = Rot3::Yaw(yaw); // yaw frame
  Point3 yV = nRy.inverse() * nV; // velocity in yaw frame
  double pitch = -atan2(yV.z(), yV.x()), roll = 0;
  Rot3 nRb = Rot3::Ypr(yaw, pitch, roll);

  // Construct initial pose
  Pose3 nTb(nRb, nT); // nTb

  return make_pair(nTb, nV);
}

//***************************************************************************

}/// namespace gtsam
