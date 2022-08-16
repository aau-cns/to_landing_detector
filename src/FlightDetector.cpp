/// Copyright (C) 2022 Martin Scheiber, Alessandro Fornasier,
/// Control of Networked Systems, University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the author at <martin.scheiber@ieee.org>
/// <alessandro.fornasier@ieee.org>

#include "toland_flight/FlightDetector.hpp"

namespace toland
{
FlightDetector::FlightDetector() : nh_("toland")
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
  // Get distance calculation method
  nh_.param<bool>("use_median", k_use_median_, k_use_median_);

  // topic strings
  std::string imu_topic, lrf_topic, baro_topic;

  // read topic parameters
  if (!nh_.getParam("imu_topic", imu_topic))
  {
    imu_topic = "/imu";
    ROS_WARN_STREAM("No IMU topic defined, using " << imu_topic << std::endl);
  }
  if (!nh_.getParam("lrf_topic", lrf_topic))
  {
    lrf_topic = "/lrf";
    ROS_WARN_STREAM("No LRF topic defined" << std::endl);
  }
  else
  {
    if (sensor_ == Sensor::UNDEFINED)
    {
      sensor_ = Sensor::LRF;
    }
  }
  if (!nh_.getParam("baro_topic", lrf_topic))
  {
    baro_topic = "/baro";
    ROS_WARN_STREAM("No BARO topic defined" << std::endl);
  }
  else
  {
    // Keep LRF as primary sensor
    if (sensor_ == Sensor::UNDEFINED && sensor_ != Sensor::LRF)
    {
      sensor_ = Sensor::BARO;
    }
  }

  // print parameter summary
  ROS_INFO_STREAM("Parameter summary as loaded by toland"
                      << "\n\tsensor_readings_window [s]: " << k_sensor_readings_window_s_
                      << "\n\tangle_threshold [deg]:      " << k_angle_threshold_deg_
                      << "\n\tdistance_threshold [m]:     " << k_distance_threshold_m_
                      << "\n\ttakeoff_theshold [m]:       " << k_takeoff_threshold_m_
                      << "\n\tuse_median [bool]:          " << k_use_median_ << "\n\timu_topic [string]:         "
                      << imu_topic << "\n\tlrf_topic [string]:         " << lrf_topic
                      << "\n\tbaro_topic [string]:        " << baro_topic << std::endl;);
  // TODO(scm): missing debug information on vectors loaded

  // setup subscribers
  sub_imu_ = nh_.subscribe(imu_topic, 1, &FlightDetector::imuCallback, this);
  sub_lrf_ = nh_.subscribe(lrf_topic, 1, &FlightDetector::lrfCallback, this);
  sub_baro_ = nh_.subscribe(baro_topic, 1, &FlightDetector::baroCallback, this);

  // setup publishers
  pub_land_ = nh_.advertise<std_msgs::Bool>("is_landed", 1);

  // setup services
  srv_to_ = nh_.advertiseService("service/takeoff", &FlightDetector::takeoffHandler, this);
}  // FlightDetector()

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
  removeOldestWindow<ImuData_t>(meas, imu_data_buffer_);

  // set imu flag
  f_have_imu_ = true;
}  // void imuCallback(...)

void FlightDetector::lrfCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  // Parse incoming message and fill out specified data structure
  LrfData_t meas;
  meas.timestamp = msg->header.stamp.toSec();
  meas.range = msg->range;

  // Push measurement into buffer
  lrf_data_buffer_.emplace_back(meas);

  // Remove oldest if sensor reading window width is reached
  removeOldestWindow<LrfData_t>(meas, lrf_data_buffer_);

  // set lrf flag
  f_have_lrf_ = true;

  // publish landed message if below threshold
  publishLanded();

}  // void lrfCallback(...)

void FlightDetector::baroCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
  // Check if P0_ is initialized and try to initialize it eventually
  if (!initializeP0())
  {
    return;
  }

  // Parse incoming message and compute barometric height and fill out specified data structure
  BaroData_t meas;
  meas.timestamp = msg->header.stamp.toSec();
  meas.p = msg->fluid_pressure;  // Pascal
  meas.h = ((r_ * T_) / (M_ * g_)) * (log(P0_) - log(meas.p));

  // Push measurement into buffer
  baro_data_buffer_.emplace_back(meas);

  // Remove oldest if sensor reading window width is reached
  removeOldestWindow<BaroData_t>(meas, baro_data_buffer_);

  // set lrf flag
  f_have_baro_ = true;

  // publish landed message if below threshold
  publishLanded();

}  // void baroCallback(...)

bool FlightDetector::takeoffHandler(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  // check for flatness and if LRF present if we are landed
  bool is_sucess = f_have_imu_ && checkFlatness();
  if (is_sucess && (f_have_lrf_ || f_have_baro_))
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
}  // bool takeoffHandler(...)

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
  if ((ros::Time::now().toSec() - imu_data_buffer_.back().timestamp) > k_sensor_readings_window_s_)
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
  for (auto& it : imu_data_buffer_)
  {
    acc_mean += it.am;
    ang_mean += it.wm;
  }
  acc_mean = acc_mean / imu_data_buffer_.size();
  ang_mean = ang_mean / imu_data_buffer_.size();

  // As further check we could eventually compute
  // the sample variance to check if there have
  // been to much excitation and return false

  // Get z axis aligned with gravity direction
  Eigen::Vector3d z_axis = acc_mean / acc_mean.norm();

  // Make x axis perpendicular to z axis
  Eigen::Vector3d e_1(1, 0, 0);
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get y axis from the cross product of these two
  Eigen::Vector3d y_axis = utils::math::skew(z_axis) * x_axis;

  // Get rotation of the imu using axes as columns
  // R_GI defines the rotation matrix that rotates
  // vector in IMU frame (I_x) to vector in global
  // inertial gravity-aligned frame (G_x = R_GI * I_x)
  Eigen::Matrix3d R_GI;
  R_GI.block(0, 0, 3, 1) = -x_axis;
  R_GI.block(0, 1, 3, 1) = -y_axis;
  R_GI.block(0, 2, 3, 1) = z_axis;

  // Apply Rotation between the imu and the platform
  Eigen::Matrix3d R_GP = R_GI * utils::math::Rot(k_R_IP);

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
}  // bool checkLanded()

double FlightDetector::calculateDistance()
{
  // Check what sensor is used
  if (sensor_ == Sensor::LRF)
  {
    // Return if buffer is empty
    if (lrf_data_buffer_.empty())
      return -1.0;

    // Return if minimum window is not reached
    if ((lrf_data_buffer_.back().timestamp - lrf_data_buffer_.front().timestamp) < k_sensor_readings_window_s_)
      return false;

    // return if last measurement is older than minimum window time
    if ((ros::Time::now().toSec() - lrf_data_buffer_.back().timestamp) > k_sensor_readings_window_s_)
    {
      // also reset the buffer in this case
      lrf_data_buffer_.clear();
      ROS_WARN("LRF older than window time");
      f_have_lrf_ = false;
      return -1.0;
    }

    // Define range
    double range = 0.0;

    // check if using median or mean
    if (k_use_median_)
    {
      range = medianRange<LrfData_t>(lrf_data_buffer_);
    }
    else
    {
      range = meanRange<LrfData_t>(lrf_data_buffer_);
    }

    // create vector with range in z direction (as defined per LRF)
    Eigen::Vector3d r_L(0, 0, range);

    // apply transformation between the lrf and the platform
    Eigen::Vector3d r_P = utils::math::Rot(k_R_PL) * r_L + utils::math::Vec(k_t_PL);

    // return distance in z direction
    return r_P(2);
  }
  else if (sensor_ == Sensor::BARO)
  {
    // Return if buffer is empty
    if (baro_data_buffer_.empty())
      return -1.0;

    // Return if minimum window is not reached
    if ((baro_data_buffer_.back().timestamp - baro_data_buffer_.front().timestamp) < k_sensor_readings_window_s_)
      return false;

    // return if last measurement is older than minimum window time
    if ((ros::Time::now().toSec() - baro_data_buffer_.back().timestamp) > k_sensor_readings_window_s_)
    {
      // also reset the buffer in this case
      baro_data_buffer_.clear();
      ROS_WARN("BARO older than window time");
      f_have_baro_ = false;
      return -1.0;
    }

    // Define range
    double range = 0.0;

    // check if using median or mean
    if (k_use_median_)
    {
      range = medianRange<BaroData_t>(baro_data_buffer_);
    }
    else
    {
      range = meanRange<BaroData_t>(baro_data_buffer_);
    }
    // return distance in z direction
    return range;
  }
  else
  {
    return -1.0;
  }
}  // double calculateDistance()

void FlightDetector::publishLanded()
{
  // publish landed message if below threshold
  if (f_reqested_to)
  {
    // check if we have taken off successfully (> threshold)
    if (f_successful_to)
    {
      if (checkLanded())
      {
        // setup message
        std_msgs::Bool msg;
        msg.data = checkFlatness();
        pub_land_.publish(msg);

        // unset flags
        f_successful_to = false;
        // f_requested_to = false;
      }
    }
    else
    {
      // check wether takeoff distance is reached
      double dist = calculateDistance();
      //        ROS_INFO_STREAM("reached distance: " << dist << std::endl);
      if (dist > k_takeoff_threshold_m_)
      {
        f_successful_to = true;
        ROS_DEBUG("Successfully taken off.");
      }
    }
  }
}  // void publishLanded()

bool FlightDetector::initializeP0()
{
  // Very naive initialization of P0 by averaging the first 100
  // measurements before takeoff
  // TODO(anyone): make it better
  if (!(f_successful_to || f_have_P0_))
  {
    if (baro_data_buffer_.size() > 100)
    {
      double p0 = 0.0;
      for (auto& it : baro_data_buffer_)
      {
        p0 += it.p;
      }
      P0_ = p0 / baro_data_buffer_.size();
      f_have_P0_ = true;
    }
  }

  return f_have_P0_;

}  // bool initializeP0

}  // namespace toland
