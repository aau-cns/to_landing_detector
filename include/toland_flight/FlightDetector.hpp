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
#include <sensor_msgs/FluidPressure.h>
#include <std_srvs/Trigger.h>
#include <type_traits>

#include "utils/mathematics.h"
#include "utils/sensors.h"

namespace toland
{

///
/// @brief Sensor used as height measurement source
///
enum Sensor {UNDEFINED = 0, LRF = 1, BARO = 2};


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
  ros::NodeHandle nh_;                                          //!< ROS Nodehandle

  /// ROS Subscribers
  ros::Subscriber sub_imu_;                                     //!< IMU topic subscriber
  ros::Subscriber sub_lrf_;                                     //!< LRF topic subscriber
  ros::Subscriber sub_baro_;                                     //!< BARO topic subscriber

  ros::ServiceServer srv_to_;                                   //!< Takeoff service

  ros::Publisher pub_land_;                                     //!< publishes

  /// Measurement buffers
  std::vector<ImuData_t> imu_data_buffer_;                      //!< buffer for IMU measurements
  std::vector<LrfData_t> lrf_data_buffer_;                      //!< buffer for LRF measurements
  std::vector<BaroData_t> baro_data_buffer_;                    //!< buffer for BARO measurements
  double takeoff_start_time {0.0};

  /// ROS Static Parameters
  double k_sensor_readings_window_s_ {1.0};                     //!< buffer time
  double k_angle_threshold_deg_ {10.0};                         //!< threshold for gravity direction in [deg]
  double k_distance_threshold_m_ {0.1};                         //!< threshold for distance in [m]
  double k_takeoff_threshold_m_ {0.5};                          //!< threshold for takeoff in [m]
  std::vector<double> k_R_IP = {1,0,0,0,1,0,0,0,1};
  std::vector<double> k_R_PL = {1,0,0,0,1,0,0,0,1};
  std::vector<double> k_t_PL = {0,0,0};
  double k_landed_wait_time = {30.0};
  bool k_use_median_ = {false};                             //!< determines if the SENSOR distance calculations use median or mean

  /// Internal Flags
  std::atomic<bool> f_have_imu_ {false};
  std::atomic<bool> f_have_lrf_ {false};
  std::atomic<bool> f_have_baro_ {false};
  std::atomic<bool> f_have_P0_ {false};
  std::atomic<bool> f_reqested_to {false};
  std::atomic<bool> f_successful_to {false};
  Sensor sensor_ {Sensor::UNDEFINED};

  /// Barometric constants
  double M_ = 0.0289644;            //Kg*mol
  double r_ = 8.31432;              //Nm/mol*K
  double g_ = 9.80665;              //m/s^2
  double T_ = 298.15;               //K
  double P0_ = 0.0;                 //Pascal


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
  /// \brief  remove entries oldest than newest timestmap - specified window
  /// \param  measurement
  /// \param  buffer
  /// \return bool if succesfull
  /// \author Alessandro Fornasier
  ///
  template<typename T> inline bool removeOldestWindow(const T meas, std::vector<T> buffer)
  {
    // Type cross-check
    if constexpr (!(std::is_same_v<T, ImuData_t> || std::is_same_v<T, LrfData_t> || std::is_same_v<T, BaroData_t>))
    {
      return false;
    }
    else
    {
      // Remove oldest if sensor reading window width is reached
      if ((meas.timestamp - buffer.begin()->timestamp) > k_sensor_readings_window_s_)
      {
        // Get first time to be kept (timestamp of the first element within the specified window)
        double Dt = meas.timestamp - k_sensor_readings_window_s_;

        // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
        auto it = std::find_if(buffer.begin(), buffer.end(), [&Dt](T meas){return meas.timestamp >= Dt;});

        // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
        if (it != buffer.begin())
        {
          buffer.erase(buffer.begin(), it);
        }
      }
      return true;
    }
  }

  ///
  /// \brief  Get the median range from sensor reading
  /// \param  measurement
  /// \return double range
  /// \author Alessandro Fornasier
  ///
  template<typename T> inline double medianRange(const std::vector<T> buffer)
  {
    // Define meadin range
    double range = 0.0;
    size_t num_meas = buffer.size();

    // create vector with range measurements
    std::vector<double> ranges;
    for (auto &it : buffer)
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
      nth_element(ranges.begin(), ranges.begin() + num_meas/2, ranges.end());
      nth_element(ranges.begin(), ranges.begin() + (num_meas-1)/2, ranges.end());

      // value is mean of idexed elements (N/2) & (N-1/2)
      range = (double)(ranges[(num_meas - 1)/2] + ranges[num_meas/2])/ 2.0;
    }
    else
    {
      // odd vector size
      nth_element(ranges.begin(), ranges.begin() + num_meas/2, ranges.end());

      // value is mean of idexed elements (N/2) & (N-1/2)
      range = (double)ranges[num_meas/2];
    }

    return range;
  }

  ///
  /// \brief  Get the mean range from sensor reading
  /// \param  measurement
  /// \return double range
  /// \author Alessandro Fornasier
  ///
  template<typename T> inline double meanRange(const std::vector<T> buffer)
  {

    // Define mean range
    double range = 0.0;

    // calculate the mean acceleration and the mean range
    for (auto &it : buffer)
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
    range = range/buffer.size();

    return range;

  }

public:
  FlightDetector();

};  // class FlightDetector
}  // namespace toland

#endif  // define FLIGHTDETECTOR_HPP
