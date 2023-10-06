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

#ifndef FLIGHTDETECTOR_HPP
#define FLIGHTDETECTOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <type_traits>

#include "utils/mathematics.h"
#include "utils/physics.h"
#include "utils/sensors.h"

namespace toland
{
///
/// @brief Sensor used as height measurement source
///
enum Sensor
{
  UNDEFINED = 0,
  DISABLED = 1,
  LRF = 2,
  BARO = 3,
};

inline std::ostream& operator<<(std::ostream& os, Sensor sensor)
{
  switch (sensor)
  {
    case Sensor::DISABLED:
      return os << "DISABLED";
    case Sensor::LRF:
      return os << "LRF";
    case Sensor::BARO:
      return os << "BARO";
    case Sensor::UNDEFINED:
    default:
      return os << "UNDEFINED";
  }

  return os;
}

///
/// \brief The FlightDetector class
///
class FlightDetector
{
private:
  typedef utils::sensors::imuData ImuData_t;
  typedef utils::sensors::lrfData LrfData_t;
  typedef utils::sensors::baroData BaroData_t;

private:
  /// ROS NODE HANDLES
  ros::NodeHandle nh_;  //!< ROS Nodehandle

  /// ROS Subscribers
  ros::Subscriber sub_imu_;   //!< IMU topic subscriber
  ros::Subscriber sub_lrf_;   //!< LRF topic subscriber
  ros::Subscriber sub_baro_;  //!< BARO topic subscriber

  ros::ServiceServer srv_to_;  //!< Takeoff service

  ros::Publisher pub_land_;  //!< publishes a message upon registering a landing
  ros::Publisher pub_to_;    //!< publishes a message upon registering a takeoff

  /// Measurement buffers
  std::vector<ImuData_t> imu_data_buffer_;    //!< buffer for IMU measurements
  std::vector<LrfData_t> lrf_data_buffer_;    //!< buffer for LRF measurements
  std::vector<BaroData_t> baro_data_buffer_;  //!< buffer for BARO measurements
  double takeoff_start_time_{ 0.0 };

  /// ROS Static Parameters
  double k_sensor_readings_window_s_{ 1.0 };  //!< buffer time
  double k_angle_threshold_deg_{ 10.0 };      //!< threshold for gravity direction in [deg]
  double k_distance_threshold_m_{ 0.1 };      //!< threshold for distance in [m]
  double k_takeoff_threshold_m_{ 0.5 };       //!< threshold for takeoff in [m]
  std::vector<double> k_R_IP_ = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  std::vector<double> k_R_PL_ = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  std::vector<double> k_t_PL_ = { 0, 0, 0 };
  double k_landed_wait_time_ = { 30.0 };
  bool k_use_median_ = { false };  //!< determines if the SENSOR distance calculations use median or mean
  bool k_is_playback_{ false };    //!< determines if a playback is active (used for debugging)
  bool k_require_srv_{ true };     //!< determines if a service call has to be performed befor any topic publication

  /// Internal Flags
  std::atomic<bool> f_have_imu_{ false };       //!< flag to determine if IMU measurement was received
  std::atomic<bool> f_have_lrf_{ false };       //!< flag to determine if LRF measurement was received
  std::atomic<bool> f_have_baro_{ false };      //!< flag to determine if baro measurement was received
  std::atomic<bool> f_have_P0_{ false };        //!< flag to determine if baro was initialized
  std::atomic<bool> f_requested_to_{ false };   //!< flag to determine if takeoff has been requested (once)
  std::atomic<bool> f_successful_to_{ false };  //!< flag to determine if takeoff has succeeded
  Sensor sensor_{ Sensor::UNDEFINED };

  /// Barometer Parameters
  double ref_pressure_{ 0.0 };  //!< barometric reference pressure at ground level (default 0 Pa)

  /// ROS Callback Functions

  ///
  /// \brief imuCallback saves the IMU measurement in a buffer
  /// \param msg IMU measurement
  /// \author Alessandro Fornasier
  ///
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  ///
  /// \brief lrfCallback saves the LRF measurement in a buffer and triggers a checkLanded() calculation
  /// \param msg LRF measurement
  ///
  void lrfCallback(const sensor_msgs::Range::ConstPtr& msg);

  ///
  /// \brief baroCallback
  /// \param msg
  ///
  void baroCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);

  ///
  /// \brief takeoffHandler ROS service callback for requesting a takeoff check
  /// \param req trigger (empty)
  /// \param res response (boolean, true if flat and close to ground)
  /// \return true if successful
  ///
  bool takeoffHandler(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  ///
  /// \brief checkFlatness checks if the UAV is flat on the ground (gravity aligned)
  /// \return true if gravity aligned within threshold k_angle_threshold_deg_
  /// \author Alessandro Fornasier
  ///
  bool checkFlatness();

  ///
  /// \brief checkLanded checks if the UAV has landed using the LRF for distance to ground measurement
  /// \return true if distance is below threshold k_distance_threshold_m_
  /// \author Martin Scheiber
  ///
  bool checkLanded();

  ///
  /// \brief calculateDistance calculates the median distance given by the LRF measurements
  /// \return distance
  /// \author Martin Scheiber
  ///
  double calculateDistance();

  ///
  /// \brief Publish landed information
  /// \return
  /// \author Martin Scheiber, Alessandro Fornasier
  ///
  void publishLanded();

  ///
  /// \brief  Initialize P0 at takeoff location
  /// \return bool
  /// \author Alessandro Fornasier
  ///
  bool initializeP0();

  ///
  /// \brief  Remove entries older than (newest - specified window)
  /// \param  meas
  /// \param  buffer
  /// \return bool if succesfull
  /// \author Alessandro Fornasier
  ///
  template <typename T>
  bool removeOldestWindow(const T meas, std::vector<T>* const buffer);

  ///
  /// \brief  Get the median range from buffer
  /// \param  buffer buffer to get the median range from
  /// \return double median range
  /// \author Alessandro Fornasier, Martin Scheiber
  ///
  template <typename T>
  double medianRange(const std::vector<T>& buffer);

  ///
  /// \brief  Get the mean range from buffer
  /// \param  buffer vector to get the mean range from
  /// \return double mean range
  /// \author Alessandro Fornasier, Martin Scheiber
  ///
  template <typename T>
  double meanRange(const std::vector<T>& buffer);

public:
  FlightDetector();

};  // class FlightDetector
}  // namespace toland

#endif  // define FLIGHTDETECTOR_HPP
