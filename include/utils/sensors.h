/// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
/// Control of Networked Systems, Universitaet Klagenfurt, Austria
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the authors at <christian.brommer@ieee.org>
/// and <alessandro.fornasier@ieee.org>


#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <Eigen/Eigen>

namespace utils
{
  namespace sensors
  {
    /**
     * @brief Struct for imu data
     */
    struct imuData
    {

        /// Timestamp of the reading (s)
        double timestamp;

        /// Gyroscope reading, angular velocity (rad/s)
        Eigen::Matrix<double, 3, 1> wm;

        /// Accelerometer reading, linear acceleration (m/s^2)
        Eigen::Matrix<double, 3, 1> am;

        /// Sort function to allow for using of STL containers
        bool operator<(const imuData& other) const {
            return timestamp < other.timestamp;
        }

    };
  }
}


#endif  // SENSORS_H
