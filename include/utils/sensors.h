/// Copyright (C) 2022 Martin Scheiber, Alessandro Fornasier, and others,
/// Control of Networked Systems, University of Klagenfurt, Austria
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the authors at <martin.scheiber@ieee.org>
/// and <alessandro.fornasier@ieee.org>

#ifndef SENSORS_H
#define SENSORS_H

#include <Eigen/Eigen>
#include <vector>

namespace utils
{
namespace sensors
{
///
/// \brief The sensorData struct is a interface for sensor data structures.
/// \note This structure only implements operators for comparision reasons
///
struct sensorData
{
  double timestamp;

  /// Sort function to allow for using of STL containers
  bool operator<(const sensorData& other) const
  {
    return timestamp < other.timestamp;
  }
};

///
/// \brief The imuData struct
///
struct imuData : sensorData
{
  Eigen::Matrix<double, 3, 1> wm;  //!<  Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> am;  //!< Accelerometer reading, linear acceleration (m/s^2)
};

///
/// \brief The lrfData struct
///
struct lrfData : sensorData
{
  double range;  //!< Range reading, distance (m)
};

///
/// \brief The baroData struct
///
struct baroData : sensorData
{
  double p;  //!< barometric pressure reading (Pa)
  double h;  //!< barometric height (m)
};
}  // namespace sensors
}  // namespace utils

#endif  // SENSORS_H
