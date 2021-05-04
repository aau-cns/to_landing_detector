#ifndef FLIGHTDETECTOR_HPP
#define FLIGHTDETECTOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include "flatness/sensors.h"
#include "flatness/mathematics.h"

namespace toland
{
  /**
   * @brief The FlightDetector class
   */
  class FlightDetector
  {

  private:
    /// ROS NODE HANDLES
    ros::NodeHandle nh_;                                          //!< ROS Nodehandle

    /// ROS Subscribers
    ros::Subscriber sub_imu_;                                     //!< IMU topic subscriber
    ros::Subscriber sub_lrf_;                                     //!< LRF topic subscriber

    /// Measurement buffers
    std::vector<imuData> imu_data_buffer_;                        //!< buffer for IMU measurements

    /// ROS Static Parameters
    double k_sensor_readings_window_s_ {1.0};
    double k_angle_threshold_deg_ {10.0};


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
    void lrfCallback(const sensor_msgs::Range::ConstPtr& msg);    //!<

public:
    FlightDetector();

  }; // class FlightDetector
} // namespace toland

#endif // define FLIGHTDETECTOR_HPP
