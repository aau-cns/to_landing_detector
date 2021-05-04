/// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the author at <martin.scheiber@ieee.org>

#ifndef FLIGHTDETECTOR_HPP
#define FLIGHTDETECTOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>

#include "utils/sensors.h"
#include "utils/mathematics.h"

namespace toland
{
  /**
   * @brief The FlightDetector class
   */
  class FlightDetector
  {
  private:
    typedef utils::sensors::imuData ImuData_t ;

  private:
    /// ROS NODE HANDLES
    ros::NodeHandle nh_;                                          //!< ROS Nodehandle

    /// ROS Subscribers
    ros::Subscriber sub_imu_;                                     //!< IMU topic subscriber
    ros::Subscriber sub_lrf_;                                     //!< LRF topic subscriber

    ros::ServiceServer srv_to_;                                   //!< Takeoff service

    /// Measurement buffers
    std::vector<ImuData_t> imu_data_buffer_;                       //!< buffer for IMU measurements

    /// ROS Static Parameters
    double k_sensor_readings_window_s_ {1.0};
    double k_angle_threshold_deg_ {10.0};
    std::vector<double> k_R_IP = {1,0,0,0,1,0,0,0,1};

    /// ROS Callback Functions

    ///
    /// \brief imuCallback
    /// \param msg
    /// \author Alessandro Fornasier
    ///
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ///
    /// \brief lrfCallback
    /// \param msg
    ///
    void lrfCallback(const sensor_msgs::Range::ConstPtr& msg);

    bool takeoffHandler(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res);

    ///
    /// \brief checkFlatness
    /// \return
    /// \author Alessandro Fornasier
    ///
    bool checkFlatness();

public:
    FlightDetector();

  }; // class FlightDetector
} // namespace toland

#endif // define FLIGHTDETECTOR_HPP
