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
    // Get distance threshold for platform landing check
    nh_.param<double>("distance_threshold", k_distance_threshold_m_, k_distance_threshold_m_);
    // Get takeoff threshold for platform landing check
    nh_.param<double>("takeoff_theshold", k_takeoff_threshold_m_, k_takeoff_threshold_m_);
    // Get IMU-Platform rotation
    nh_.param<std::vector<double>>("R_IP", k_R_IP, k_R_IP);
    // Get LRF-Platform rotation
    nh_.param<std::vector<double>>("R_LP", k_R_PL, k_R_PL);
    // Get LRF-Platform translation
    nh_.param<std::vector<double>>("t_LP", k_t_PL, k_t_PL);
    // Get LRF distance calculation method
    nh_.param<bool>("lrf_use_median", k_lrf_use_median_, k_lrf_use_median_);

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
    sub_imu_ = nh_.subscribe(imu_topic, 1, &FlightDetector::imuCallback, this);
    sub_lrf_ = nh_.subscribe(lrf_topic, 1, &FlightDetector::lrfCallback, this);

    // setup publishers
    pub_land_ = nh_.advertise<std_msgs::Bool>("is_landed", 1);

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
      if (it != imu_data_buffer_.begin())
      {
        imu_data_buffer_.erase(imu_data_buffer_.begin(), it);
      }     
    }

    // set imu flag
    f_have_imu_ = true;
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
      if (it != lrf_data_buffer_.begin())
      {
	      lrf_data_buffer_.erase(lrf_data_buffer_.begin(), it);
      }
    }

    // set lrf flag
    f_have_lrf_ = true;

    // publish landed message if below threshold
    if (f_reqested_to)
    {
      // check if we have taken off successfully (> threshold)
      if (f_successful_to)
      {
        // setup message
        std_msgs::Bool msg;
        msg.data = checkFlatness();
        pub_land_.publish(msg);

        // unset flags
        f_successful_to = false;
        // f_requested_to = false;
      }
      else
      {
        // check wether takeoff distance is reached
        if (calculateDistance() > k_takeoff_threshold_m_)
        {
          f_successful_to = true;
          ROS_DEBUG("Successfully taken off.");
        }
      }
    }
  } // void lrfCallback(...)

  bool FlightDetector::takeoffHandler(
      std_srvs::Trigger::Request& req,
      std_srvs::Trigger::Response& res)
  {
    // check for flatness and if LRF present if we are landed
    bool is_sucess = f_have_imu_ && checkFlatness();
    if (is_sucess && f_have_lrf_)
      is_sucess = checkLanded();
    else
    {
      is_sucess = false;
    }

    // create response message
    res.success = is_sucess;
    res.message = "number of measurements: " + std::to_string(imu_data_buffer_.size());

    // setup request takeof
    f_reqested_to = is_sucess;
    takeoff_start_time = ros::Time::now().toSec();

    // return sucessfull execution
    return true;
  } // bool takeoffHandler(...)

  bool FlightDetector::checkFlatness()
  {
    ROS_DEBUG("checkFlatness called");

    // Return if buffer is empty
      if (imu_data_buffer_.empty()) 
      {
          ROS_WARN("Imu buffer emtpy.");
          return false;
      }

      // return if last measurement is older than minimum window time
      if ((ros::Time::now().toSec() - imu_data_buffer_.end()->timestamp) > k_sensor_readings_window_s_)
      {
        // also reset the buffer in this case
        imu_data_buffer_.clear();
        f_have_imu_ = false;
        ROS_WARN("IMU older than window time");
        return false;
      }

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
      {
          ROS_WARN("Attidue over threshold:\n\troll:  %f\n\tpitch: %f", abs(eul_ang(0)), abs(eul_ang(1)));
          return false;
      }

      ROS_DEBUG("we are flat:\n\troll:  %f\n\tpitch: %f", abs(eul_ang(0)), abs(eul_ang(1)));
      return true;
  }  // bool checkFlatness()

  bool FlightDetector::checkLanded()
  {
      double range = calculateDistance();

      // check if range is valid
      if (range < 0.0)
        return false;

      // check distance to ground with threshold
      if (range > k_distance_threshold_m_ || range < 0)
        return false;

      ROS_DEBUG("we are grounded:\n\tdistance: %f", range);
      return true;
  } // bool checkLanded()

  double FlightDetector::calculateDistance()
  {
    // Return if buffer is empty
    if (lrf_data_buffer_.empty())
      return -1.0;

    // Return if minimum window is not reached
    //if ((lrf_data_buffer_.end()->timestamp - lrf_data_buffer_.begin()->timestamp) < k_sensor_readings_window_s_)
    //  return false;

    // return if last measurement is older than minimum window time
    if ((ros::Time::now().toSec() - lrf_data_buffer_.end()->timestamp) > k_sensor_readings_window_s_)
    {
      // also reset the buffer in this case
      lrf_data_buffer_.clear();
      ROS_WARN("LRF older than window time");
      f_have_lrf_ = false;
      return -1.0;
    }

    // Define mean range
    double range = 0.0;
    size_t num_meas = lrf_data_buffer_.size();

    // check if using median or mean
    if (k_lrf_use_median_)
    {
      // median calculations

      // create vector with range measurements
      std::vector<double> lrf_ranges;
      for (auto &it : lrf_data_buffer_)
      {
        lrf_ranges.push_back(it.range);
      }

      // derive the median of vector using n-th element
      // see also https://www.geeksforgeeks.org/finding-median-of-unsorted-array-in-linear-time-using-c-stl/
      if (num_meas % 2 == 0)
      {
        // even vector size
        nth_element(lrf_ranges.begin(), lrf_ranges.begin() + num_meas/2, lrf_ranges.end());
        nth_element(lrf_ranges.begin(), lrf_ranges.begin() + (num_meas-1)/2, lrf_ranges.end());

        // value is mean of idexed elements (N/2) & (N-1/2)
        range = (double)(lrf_ranges[(num_meas - 1)/2] + lrf_ranges[num_meas/2])/ 2.0;
      }
      else
      {
        // odd vector size
        nth_element(lrf_ranges.begin(), lrf_ranges.begin() + num_meas/2, lrf_ranges.end());

        // value is mean of idexed elements (N/2) & (N-1/2)
        range = (double)lrf_ranges[num_meas/2];
      }

    }
    else
    {
      // mean calculations

      // calculate the mean acceleration and the mean range
      for (auto &it : lrf_data_buffer_)
      {
        range += it.range;
      }
      range = range/lrf_data_buffer_.size();
    }

    // create vector with range in z direction (as defined per LRF)
    Eigen::Vector3d r_L(0,0,range);

    // apply transformation between the lrf and the platform
    Eigen::Vector3d r_P = utils::math::Rot(k_R_PL)*r_L + utils::math::Vec(k_t_PL);

    // return distance in z direction
    return r_P(2);
  }  // double calculateDistance()

} // namespace toland
