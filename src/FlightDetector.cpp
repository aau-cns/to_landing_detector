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
    // read general parameters
    // Get sensor reading window
    nh_.param<double>("sensor_readings_window", k_sensor_readings_window_s_, k_sensor_readings_window_s_);
    // Get roll and pitch angle threshold for platform flatness check
    nh_.param<double>("angle_threshold", k_angle_threshold_deg_, k_angle_threshold_deg_);
    // Get IMU-Platform rotation
    nh_.param<std::vector<double>>("R_IP", k_R_IP, k_R_IP);
    // Get LRF-Platform rotation
    nh_.param<std::vector<double>>("R_LP", k_R_PL, k_R_PL);
    // Get LRF-Platform translation
    nh_.param<std::vector<double>>("t_LP", k_t_PL, k_t_PL);

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

    // setup subscribers
    sub_imu_ = nh_.subscribe(imu_topic, 999, &FlightDetector::imuCallback, this);
    sub_imu_ = nh_.subscribe(lrf_topic, 999, &FlightDetector::lrfCallback, this);

    // setup services
    srv_to_ = nh_.advertiseService("service/takeoff", &FlightDetector::takeoffHandler, this);
  } // FlightDetector()

  void FlightDetector::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // Parse incoming message and fill out specified data structure
    ImuData_t meas;
    meas.timestamp = msg->header.stamp.toSec();
    meas.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    meas.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // Push measurement into buffer
    imu_data_buffer_.emplace_back(meas);

    // Remove oldest if sensor reading window width is reached
    if ((meas.timestamp - imu_data_buffer_.begin()->timestamp) > k_sensor_readings_window_s_)
    {
      // Since we check everytime a new measurement is added to the buffer it would
      // be sufficient to simply remove the first element, however it is more robust
      // to check everytime how many elements we should remove.

      double Dt = meas.timestamp - k_sensor_readings_window_s_;

      // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
      auto it = std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(), [&Dt](ImuData_t meas){return meas.timestamp >= Dt;});

      // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
      imu_data_buffer_.erase(imu_data_buffer_.begin(), it-1);
    }
  } // void imuCallback(...)

  /// \todo merge part of this function with FlightDetector::imuCallback
  void FlightDetector::lrfCallback(const sensor_msgs::Range::ConstPtr& msg)
  {
    // Parse incoming message and fill out specified data structure
    LrfData_t meas;
    meas.timestamp = msg->header.stamp.toSec();
    meas.range = msg->range;

    // Push measurement into buffer
    lrf_data_buffer_.emplace_back(meas);

    // TODO(scm): merge this with imu callback into general function
    // Remove oldest if sensor reading window width is reached
    if ((meas.timestamp - lrf_data_buffer_.begin()->timestamp) > k_sensor_readings_window_s_)
    {
      // Since we check everytime a new measurement is added to the buffer it would
      // be sufficient to simply remove the first element, however it is more robust
      // to check everytime how many elements we should remove.

      double Dt = meas.timestamp - k_sensor_readings_window_s_;

      // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
      auto it = std::find_if(lrf_data_buffer_.begin(), lrf_data_buffer_.end(), [&Dt](LrfData_t meas){return meas.timestamp >= Dt;});

      // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
      lrf_data_buffer_.erase(lrf_data_buffer_.begin(), it-1);
    }
  } // void lrfCallback(...)

  bool FlightDetector::takeoffHandler(
      std_srvs::Trigger::Request& req,
      std_srvs::Trigger::Response& res)
  {
    // create response message
    res.success = checkFlatness();
    res.message = std::string("number of measurements: %d", imu_data_buffer_.size());

    // return sucessfull execution
    return true;
  } // bool takeoffHandler(...)

  bool FlightDetector::checkFlatness()
  {
    // Return if buffer is empty
      if (imu_data_buffer_.empty())
          return false;

      // Return if minimum window is not reached
      if ((imu_data_buffer_.end()->timestamp - imu_data_buffer_.begin()->timestamp) < k_sensor_readings_window_s_)
        return false;

      // Define mean acceleration and mean angular velocity
      Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
      Eigen::Vector3d ang_mean = Eigen::Vector3d::Zero();

      // Calculate the mean acceleration and the mean angular velocity
      for (auto &it : imu_data_buffer_)
      {
        acc_mean += it.am;
        ang_mean += it.wm;
      }
      acc_mean = acc_mean/imu_data_buffer_.size();
      ang_mean = ang_mean/imu_data_buffer_.size();

      // As further check we could eventually compute
      // the sample variance to check if there have
      // been to much excitation and return false

      // Get z axis aligned with gravity direction
      Eigen::Vector3d z_axis = acc_mean/acc_mean.norm();

      // Make x axis perpendicular to z axis
      Eigen::Vector3d e_1(1,0,0);
      Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
      x_axis= x_axis/x_axis.norm();

      // Get y axis from the cross product of these two
      Eigen::Vector3d y_axis = utils::math::skew(z_axis)*x_axis;

      // Get rotation of the imu using axes as columns
      // R_GI defines the rotation matrix that rotates
      // vector in IMU frame (I_x) to vector in global
      // inertial gravity-aligned frame (G_x = R_GI * I_x)
      Eigen::Matrix3d R_GI;
      R_GI.block(0,0,3,1) = -x_axis;
      R_GI.block(0,1,3,1) = -y_axis;
      R_GI.block(0,2,3,1) = z_axis;

      // Apply Rotation between the imu and the platform
      Eigen::Matrix3d R_GP = R_GI*utils::math::Rot(k_R_IP);

      // Convert Rotation matrix to euler angles
      Eigen::Vector3d eul_ang = utils::math::eul(R_GP);

      // Compare roll and pitch with thresholds
      if (abs(eul_ang(0)) > k_angle_threshold_deg_ || abs(eul_ang(1)) > k_angle_threshold_deg_)
        return false;

      // test passed
      return true;
  }  // bool checkFlatness()

  bool FlightDetector::checkLanded()
  {
    // Return if buffer is empty
      if (lrf_data_buffer_.empty())
          return false;

      // Return if minimum window is not reached
      if ((lrf_data_buffer_.end()->timestamp - lrf_data_buffer_.begin()->timestamp) < k_sensor_readings_window_s_)
        return false;

      // Define mean range
      double range_mean = 0.0;

      // Calculate the mean acceleration and the mean angular velocity
      for (auto &it : lrf_data_buffer_)
      {
        range_mean += it.range;
      }
      range_mean = range_mean/lrf_data_buffer_.size();

      // create vector with range in z direction (as defined per LRF)
      Eigen::Vector3d r_L(0,0,range_mean);

      // Apply transformation between the lrf and the platform
      Eigen::Vector3d r_P = utils::math::Rot(k_R_PL)*r_L + utils::math::Vec(k_t_PL);

      // check distance to ground with threshold
      if (r_P(2) > k_distance_threshold_m_ || r_P(2) < 0)
        return false;

      // test passed
      return true;
  } // bool checkLanded()

} // namespace toland
