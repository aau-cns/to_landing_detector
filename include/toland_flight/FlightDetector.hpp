/// Copyright (C) 2021-2022 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
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
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "utils/mathematics.h"
#include "utils/sensors.h"

namespace toland
{
///
/// \brief The FlightDetector class
///
class FlightDetector
{
private:
  typedef utils::sensors::imuData ImuData_t;
  typedef utils::sensors::lrfData LrfData_t;

private:
  /// ROS NODE HANDLES
  ros::NodeHandle nh_;  //!< ROS Nodehandle

  /// ROS Subscribers
  ros::Subscriber sub_imu_;  //!< IMU topic subscriber
  ros::Subscriber sub_lrf_;  //!< LRF topic subscriber

  ros::ServiceServer srv_to_;  //!< Takeoff service

  ros::Publisher pub_land_;  //!< publishes

  /// Measurement buffers
  std::vector<ImuData_t> imu_data_buffer_;  //!< buffer for IMU measurements
  std::vector<LrfData_t> lrf_data_buffer_;  //!< buffer for LRF measurements
  double takeoff_start_time{ 0.0 };

  /// ROS Static Parameters
  double k_sensor_readings_window_s_{ 1.0 };  //!< buffer time
  double k_angle_threshold_deg_{ 10.0 };      //!< threshold for gravity direction in [deg]
  double k_distance_threshold_m_{ 0.1 };      //!< threshold for distance in [m]
  double k_takeoff_threshold_m_{ 0.5 };       //!< threshold for takeoff in [m]
  std::vector<double> k_R_IP = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  std::vector<double> k_R_PL = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  std::vector<double> k_t_PL = { 0, 0, 0 };
  double k_landed_wait_time = { 30.0 };
  bool k_lrf_use_median_ = { false };  //!< determines if the LRF distance calculations use median or mean

  /// Internal Flags
  std::atomic<bool> f_have_imu_{ false };      //!< flag to determine if IMU measurement was received
  std::atomic<bool> f_have_lrf_{ false };      //!< flag to determine if LRF measurement was received
  std::atomic<bool> f_reqested_to{ false };    //!< flag to determine if takeoff has been requested (once)
  std::atomic<bool> f_successful_to{ false };  //!< flag to determine if takeoff has succeeded

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
  /// \brief calculateDistance calculates the mean/median distance given by the LRF measurements
  /// \return the mean/median distance from the buffer
  /// \author Martin Scheiber
  ///
  double calculateDistance();

public:
  FlightDetector();

};  // class FlightDetector
}  // namespace toland

#endif  // define FLIGHTDETECTOR_HPP
