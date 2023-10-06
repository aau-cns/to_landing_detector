// Copyright (C) 2022 Martin Scheiber, Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <martin.scheiber@ieee.org> and
// <alessandro.fornasier@ieee.org>.

#include "toland_flight/FlightDetector.hpp"

namespace toland
{
FlightDetector::FlightDetector() : nh_("toland_detector")
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
  nh_.param<std::vector<double>>("R_IP", k_R_IP_, k_R_IP_);
  // Get LRF-Platform rotation
  nh_.param<std::vector<double>>("R_LP", k_R_PL_, k_R_PL_);
  // Get LRF-Platform translation
  nh_.param<std::vector<double>>("t_LP", k_t_PL_, k_t_PL_);
  // Get distance calculation method
  nh_.param<bool>("use_median", k_use_median_, k_use_median_);
  // Get distance calculation method
  nh_.param<bool>("playback", k_is_playback_, k_is_playback_);
  // Get flag for requirement on service
  nh_.param<bool>("require_srv_call", k_require_srv_, k_require_srv_);

  // topic strings
  std::string imu_topic, lrf_topic, baro_topic, default_sensor;

  // read default sensor
  if (!nh_.getParam("default_sensor", default_sensor))
  {
    sensor_ = Sensor::UNDEFINED;
    ROS_WARN_STREAM("No default sensor defined, using whichever topic is given otherwise LRF.");
  }
  else
  {
    if (default_sensor == "disabled")
      sensor_ = Sensor::DISABLED;
    else if (default_sensor == "lrf")
      sensor_ = Sensor::LRF;
    else if (default_sensor == "baro")
      sensor_ = Sensor::BARO;
    else
    {
      sensor_ = Sensor::UNDEFINED;
      ROS_WARN_STREAM("Unknown default sensor '" << default_sensor
                                                 << "'. Defaulting to whichever topic is given, otherwise LRF.");
    }
  }

  // read topic parameters
  if (!nh_.getParam("imu_topic", imu_topic))
  {
    imu_topic = "imu";
    ROS_WARN_STREAM("No IMU topic defined, using " << imu_topic << std::endl);
  }
  if (!nh_.getParam("lrf_topic", lrf_topic))
  {
    lrf_topic = "lrf";
    ROS_WARN_STREAM("No LRF topic defined" << std::endl);
  }
  else
  {
    if (sensor_ == Sensor::UNDEFINED)
    {
      sensor_ = Sensor::LRF;
    }
  }
  if (!nh_.getParam("baro_topic", baro_topic))
  {
    baro_topic = "baro";
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
                      << "\n\tsensor_readings_window [s]: " << k_sensor_readings_window_s_ << "\n"
                      << "\tangle_threshold [deg]:      " << k_angle_threshold_deg_ << "\n"
                      << "\tdistance_threshold [m]:     " << k_distance_threshold_m_ << "\n"
                      << "\ttakeoff_theshold [m]:       " << k_takeoff_threshold_m_ << "\n"
                      << "\tuse_median [bool]:          " << k_use_median_ << "\n"
                      << "\trequire_srv_call [bool]:    " << k_require_srv_ << "\n"
                      << "\timu_topic [string]:         " << imu_topic << "\n"
                      << "\tlrf_topic [string]:         " << lrf_topic << "\n"
                      << "\tbaro_topic [string]:        " << baro_topic << "\n"
                      << "\tdefault sensor [string]:    " << sensor_ << std::endl;);
  // TODO(scm): missing debug information on vectors loaded

  // setup subscribers
  sub_imu_ = nh_.subscribe(imu_topic, 1, &FlightDetector::imuCallback, this);
  sub_lrf_ = nh_.subscribe(lrf_topic, 1, &FlightDetector::lrfCallback, this);
  sub_baro_ = nh_.subscribe(baro_topic, 1, &FlightDetector::baroCallback, this);

  // setup publishers
  pub_land_ = nh_.advertise<std_msgs::Bool>("is_landed", 1);
  pub_to_ = nh_.advertise<std_msgs::Bool>("is_takeoff", 1);

  // setup services
  srv_to_ = nh_.advertiseService("service/takeoff", &FlightDetector::takeoffHandler, this);

  // update 'f_requested_to' based on requirement to servicec call
  f_requested_to_ = !k_require_srv_;
  takeoff_start_time_ = ros::Time::now().toSec();
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
  removeOldestWindow<ImuData_t>(meas, &imu_data_buffer_);

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

  // set lrf flag
  f_have_lrf_ = true;
  ROS_DEBUG_STREAM("*   add lrf-height:  " << meas.range);

  // publish landed message if below threshold
  publishLanded();

  // Remove oldest if sensor reading window width is reached
  removeOldestWindow<LrfData_t>(meas, &lrf_data_buffer_);

}  // void lrfCallback(...)

void FlightDetector::baroCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
  // Parse incoming message and compute barometric height and fill out specified data structure
  BaroData_t meas;
  meas.timestamp = msg->header.stamp.toSec();
  meas.p = msg->fluid_pressure;  // Pascal

  // Check if P0_ is initialized and try to initialize it eventually
  if (!initializeP0())
  {
    // set distance invalid but add measurement to buffer anyways
    meas.h = -1;
    baro_data_buffer_.emplace_back(meas);
    return;
  }

  // calculate distance
  meas.h = utils::physics::rTOverMg_ * (std::log(ref_pressure_) - std::log(meas.p));

  // Push measurement into buffer
  baro_data_buffer_.emplace_back(meas);

  // set baro flag
  f_have_baro_ = true;
  ROS_DEBUG_STREAM("*   add baro-height: " << meas.h);

  // publish landed message if below threshold
  publishLanded();

  // Remove oldest if sensor reading window width is reached
  removeOldestWindow<BaroData_t>(meas, &baro_data_buffer_);

}  // void baroCallback(...)

bool FlightDetector::takeoffHandler(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_DEBUG_STREAM("> checking TO condition");

  // check for flatness and if LRF present if we are landed
  bool is_sucess = f_have_imu_ && checkFlatness();
  if (is_sucess && (f_have_lrf_ || f_have_baro_ || sensor_ == Sensor::DISABLED))
  {
    is_sucess = checkLanded();
  }
  else
  {
    /// \todo(scm): this is related to issue #2 (internal) and has to be discussed. In previous states, when no AGL
    /// sensor was active, the toland_flight checks only if we are flat.
    ROS_WARN_STREAM("No distance measurement received yet. Returning 'false' although flat ...");
    is_sucess = false;
  }

  // create response message
  std::string res_msg = std::string("number of measurements -- IMU: ") + std::to_string(imu_data_buffer_.size()) +
                        std::string(" -- LRF: ") + std::to_string(lrf_data_buffer_.size()) + std::string(" -- BARO: ") +
                        std::to_string(baro_data_buffer_.size());
  res.success = is_sucess;
  res.message = res_msg;

  // setup request takeof
  f_requested_to_ = !k_require_srv_ || is_sucess;
  takeoff_start_time_ = ros::Time::now().toSec();

#ifndef NDEBUG
  // only perform this in debug mode
  if (is_sucess)
    ROS_DEBUG_STREAM("> TO request SUCCESSFUL");
  else
    ROS_DEBUG_STREAM("> TO request FAILED");
#endif  // NDBEUG

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
  if (!k_is_playback_ && (ros::Time::now().toSec() - imu_data_buffer_.back().timestamp) > k_sensor_readings_window_s_)
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
  Eigen::Matrix3d R_GP = R_GI * utils::math::Rot(k_R_IP_);

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
  // return true of distance sensor is disabled
  if (sensor_ == Sensor::DISABLED)
  {
    ROS_DEBUG("distance sensor is disabled, thus we are always grounded");
    return true;
  }

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
    {
      ROS_DEBUG_STREAM("> empty buffer");
      return -1.0;
    }

    // Return if minimum window is not reached
    if ((lrf_data_buffer_.back().timestamp - lrf_data_buffer_.front().timestamp) < k_sensor_readings_window_s_)

    {
      ROS_DEBUG_STREAM("> not enough readings in buffer");
      return -1.0;
    }

    // return if last measurement is older than minimum window time
    if (!k_is_playback_ && (ros::Time::now().toSec() - lrf_data_buffer_.back().timestamp) > k_sensor_readings_window_s_)
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
    Eigen::Vector3d r_P = utils::math::Rot(k_R_PL_) * r_L + utils::math::Vec(k_t_PL_);

    // return distance in z direction
    return r_P(2);
  }
  else if (sensor_ == Sensor::BARO)
  {
    // Return if buffer is empty
    if (baro_data_buffer_.empty())
    {
      ROS_DEBUG_STREAM(">   no measurements in buffer yet.");
      return -1.0;
    }

    // Return if minimum window is not reached (use 80% of ideal window to allow success for lower rate sensors)
    if ((baro_data_buffer_.back().timestamp - baro_data_buffer_.front().timestamp) < 0.8 * k_sensor_readings_window_s_)
    {
      ROS_DEBUG_STREAM(">   not enough measurements in buffer yet.");
      return -1.0;
    }

    // return if last measurement is older than minimum window time
    if (!k_is_playback_ &&
        (ros::Time::now().toSec() - baro_data_buffer_.back().timestamp) > k_sensor_readings_window_s_)
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
  if (f_requested_to_)
  {
    ROS_DEBUG_STREAM("> checking if LANDED");

    // check if we have taken off successfully (> threshold)
    if (f_successful_to_)
    {
      if (checkLanded())
      {
        // setup message
        std_msgs::Bool msg;
        msg.data = checkFlatness();
        pub_land_.publish(msg);

        // unset flags
        f_successful_to_ = false;
        // f_requested_to = false;
      }
    }
    else
    {
      // check wether takeoff distance is reached
      double dist = calculateDistance();
      ROS_DEBUG_STREAM("> reached TO distance: " << dist);

      if (dist > k_takeoff_threshold_m_)
      {
        // setup message
        std_msgs::Bool msg;
        msg.data = true;  // INFO(scm): no real information to set here atm
        pub_to_.publish(msg);

        f_successful_to_ = true;
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
  if (!(f_have_P0_ || f_successful_to_))
  {
    if (baro_data_buffer_.size() > 100)
    {
      double p0 = 0.0;
      for (auto& it : baro_data_buffer_)
      {
        p0 += it.p;
      }
      ref_pressure_ = p0 / baro_data_buffer_.size();
      f_have_P0_ = true;

      // update buffer entry heights given p0
      for (auto& it : baro_data_buffer_)
      {
        it.h = utils::physics::rTOverMg_ * (std::log(ref_pressure_) - std::log(it.p));
      }

      // ouptput
      ROS_INFO_STREAM("set reference pressure to " << ref_pressure_ << " Pa.");
    }
  }

  return f_have_P0_;

}  // bool initializeP0()

template <typename T>
bool FlightDetector::removeOldestWindow(const T meas, std::vector<T>* const buffer)
{
  // Type cross-check
  if constexpr (!(std::is_same_v<T, ImuData_t> || std::is_same_v<T, LrfData_t> || std::is_same_v<T, BaroData_t>))
  {
    return false;
  }
  else
  {
    const double max_readings_window = 1.5 * k_sensor_readings_window_s_;

    // Remove oldest if sensor reading window width is reached
    if ((meas.timestamp - buffer->begin()->timestamp) > max_readings_window)
    {
      // Get first time to be kept (timestamp of the first element within the specified window)
      double Dt = meas.timestamp - max_readings_window;

      // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
      auto it = std::find_if(buffer->begin(), buffer->end(), [&Dt](T meas) { return meas.timestamp >= Dt; });

      // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
      if (it != buffer->begin())
      {
        buffer->erase(buffer->begin(), it);
      }
    }
    return true;
  }
}  // <T> bool removeOldestWindow(...)

template <typename T>
double FlightDetector::medianRange(const std::vector<T>& buffer)
{
  // Define meadin range
  double range = 0.0;
  size_t num_meas = buffer.size();

  // create vector with range measurements
  std::vector<double> ranges;
  for (auto& it : buffer)
  {
    // Get ranges
    if constexpr (std::is_same_v<T, LrfData_t>)
    {
      ranges.push_back(it.range);
    }
    else if constexpr (std::is_same_v<T, BaroData_t>)
    {
      ranges.push_back(it.h);
    }
    else
    {
      return -1.0;
    }
  }

  // derive the median of vector using n-th element
  // see also https://www.geeksforgeeks.org/finding-median-of-unsorted-array-in-linear-time-using-c-stl/
  if (num_meas % 2 == 0)
  {
    // even vector size
    nth_element(ranges.begin(), ranges.begin() + num_meas / 2, ranges.end());
    nth_element(ranges.begin(), ranges.begin() + (num_meas - 1) / 2, ranges.end());

    // value is mean of idexed elements (N/2) & (N-1/2)
    range = (double)(ranges[(num_meas - 1) / 2] + ranges[num_meas / 2]) / 2.0;
  }
  else
  {
    // odd vector size
    nth_element(ranges.begin(), ranges.begin() + num_meas / 2, ranges.end());

    // value is mean of idexed elements (N/2) & (N-1/2)
    range = (double)ranges[num_meas / 2];
  }

  ROS_DEBUG_STREAM("= median range: " << range);
  return range;
}

template <typename T>
double FlightDetector::meanRange(const std::vector<T>& buffer)
{
  // Define mean range
  double range = 0.0;

  // calculate the mean acceleration and the mean range
  for (auto& it : buffer)
  {
    if constexpr (std::is_same_v<T, LrfData_t>)
    {
      range += it.range;
    }
    else if constexpr (std::is_same_v<T, BaroData_t>)
    {
      range += it.h;
    }
    else
    {
      return -1.0;
    }
  }
  range = range / buffer.size();

  ROS_DEBUG_STREAM("= mean range:   " << range);
  return range;
}

}  // namespace toland
