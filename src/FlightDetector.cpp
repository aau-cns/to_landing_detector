/// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the author at <martin.scheiber@ieee.org>

#include "toland_flight/FlightDetector.hpp"

namespace toland
{

  FlightDetector::FlightDetector()
    : nh_("toland")
  {
    // topic strings
    std::string imu_topic, lrf_topic;

    // read topic parameters
    if(!nh_.getParam("imu_topic", imu_topic))
    {
      imu_topic = "/imu";
      ROS_WARN_STREAM(
            "No IMU topic defined, using " << imu_topic << std::endl);
    }
    if(!nh_.getParam("lrf_topic", lrf_topic))
    {
      lrf_topic = "/lrf";
      ROS_WARN_STREAM(
            "No LRF topic defined, using " << lrf_topic << std::endl);
    }

    // setup subsribers
    sub_imu_ = nh_.subscribe(imu_topic, 999, &FlightDetector::imuCallback, this);
    sub_imu_ = nh_.subscribe(lrf_topic, 999, &FlightDetector::lrfCallback, this);
  } // FlightDetector()

  void FlightDetector::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // Parse incoming message and fill out specified data structure
    imuData meas;
    meas.timestamp = msg->header.stamp.toSec();
    meas.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    meas.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // Push measurement into buffr
    imu_data_buffer_.emplace_back(meas);

    // Remove oldest if sensor reading window width is reached
    if ((meas.timestamp - imu_data_buffer_.begin()->timestamp) > k_sensor_readings_window_s_)
    {
      // Since we check everytime a new measurement is added to the buffer it would
      // be sufficient to simply remove the first element, however it is more robust
      // to check everytime how many elements we should remove.

      double Dt = meas.timestamp - k_sensor_readings_window_s_;

      // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
      auto it = std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(), [&Dt](imuData meas){return meas.timestamp >= Dt;});

      // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
      imu_data_buffer_.erase(imu_data_buffer_.begin(), it-1);
    }
  } // void imuCallback(...)

  void FlightDetector::lrfCallback(const sensor_msgs::Range::ConstPtr& msg)
  {

  } // void lrfCallback(...)

} // namespace toland
