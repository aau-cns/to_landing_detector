/// Copyright (C) 2022 Christian Brommer and Alessandro Fornasier,
/// Control of Networked Systems, University of Klagenfurt, Austria
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the authors at <christian.brommer@ieee.org>
/// and <alessandro.fornasier@ieee.org>

#ifndef MATHEMATICS_H
#define MATHEMATICS_H

#include <math.h>
#include <stdlib.h>

#include <Eigen/Eigen>
#include <vector>

namespace utils
{
namespace math
{
/**
 * @brief Skew-symmetric matrix from a given 3x1 vector
 * @param 3x1 vector
 * @return 3x3 skew-symmetric matrix
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d skew;

  skew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  return skew;
}

/**
 * @brief Rotation matrix from a given 9x1 vector in row-major order
 * @param 9x1 vector in row-major order
 * @return 3x3 Rotation matrix
 * @bug technically this is not a good definition (not implicit); does not adhere to coding style.
 */
inline Eigen::Matrix3d Rot(const std::vector<double>& x)
{
  Eigen::Matrix3d R;

  R << x.at(0), x.at(1), x.at(2), x.at(3), x.at(4), x.at(5), x.at(6), x.at(7), x.at(8);

  return R;
}

///
/// \brief Vec creates 3x1 vector from given a std::vector
/// \param x vector as std::vector
/// \return 3x1 Eigen vector
///
inline Eigen::Vector3d Vec(const std::vector<double>& x)
{
  return Eigen::Vector3d(x.at(0), x.at(1), x.at(2));
}

/**
 * @brief Conversion from radians to degrees.
 */
inline double rad2deg(const double rad)
{
  return rad * 180 / M_PI;
}

/**
 * @brief Rotation matrix to euler angles conversion.
 *
 * roll = rotation about x
 * pitch = rotation about y
 * yaw = rotation abot z
 *
 * @param 3x3 Rotation matrix
 * @return 3x1 vector of angles in degree
 */
inline Eigen::Vector3d eul(const Eigen::Matrix3d& R)
{
  double eps = 1.0e-3;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  Eigen::Vector3d eul;

  if ((abs(R(2, 0)) - 1) < eps)
  {
    pitch = -asin(R(2, 0));
    roll = atan2(R(2, 1) / cos(roll), R(2, 2) / cos(roll));
    yaw = atan2(R(1, 0) / cos(roll), R(0, 0) / cos(roll));
  }
  else
  {
    yaw = 0.0;
    if (abs(R(2, 0) + 1) < eps)
    {
      pitch = M_PI / 2;
      roll = atan2(R(0, 1), R(0, 2));
    }
    else
    {
      pitch = -M_PI / 2;
      roll = atan2(-R(0, 1), -R(0, 2));
    }
  }
  eul << rad2deg(roll), rad2deg(pitch), rad2deg(yaw);

  return eul;
}

}  // namespace math
}  // namespace utils

#endif  // MATHEMATICS_H
