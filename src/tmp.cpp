/* HEADER FILE DECLARATIONS - only the part regarding the flatness check */

#include "sensors.h"
#include "mathematics.h"

private:

/**
 * @brief Function that will use the buffered IMU data to check the "flatness" of the platform.
 * @return Return true if the platform is "flat" and if there are no errors otherwise return false
 */
[[nodiscard]] bool checkFlatness();

/**
 * @brief IMU callback
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

/// Subscribers
ros::Subscriber sub_imu_;

/// IMU measurement buffer
std::vector<imuData> imu_data_buffer_;

/*************************************************************************/

/* CPP FILE IMPLEMENTATION - only the part regarding the flatness check */

// IMU topic
std::string imu_topic;

// How long collect measurements to check flatness
double sensor_readings_window_s = 1.0;

// Pitch and roll threshold
double angle_threshold_deg = 10.0;

// IMU to Platform transformation (to include the cases where the imu is not aligned with the platform)
// Rotation matrix that rotates vector in the platform frame (P_x) to vector in the IMU frame (I_x = R_IP * P_x)
std::vector<double> R_IP = {1,0,0,0,1,0,0,0,1};

// Get sensor reading window
nh_.param<double>("sensor_readings_window", sensor_readings_window_s, sensor_readings_window_s);

// Get roll and pitch angle threshold for platform flatness check
nh_.param<double>("angle_threshold", angle_threshold_deg, angle_threshold_deg);

// Get IMU-Platform rotation
nh_.param<std::vector<double>>("R_IP", R_IP, R_IP);

// Get imu topic
if(!nh_.getParam("imu_topic", imu_topic)) {
  std::cout << std::endl << BOLD(RED("No IMU topic defined")) << std::endl;
  return false;
}





// Subscribe to IMU
sub_imu_ = nh_.subscribe(opts_->imu_topic, 999, &AmazeAutonomy::imuCallback, this);

// Sleep (at least twice sensor reading window) to get imu measurements
usleep(static_cast<__useconds_t>(sensor_readings_window_s*2e6));

// Check flatness
if(!checkFlatness()) {
  return false;
}







void AmazeAutonomy::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

  // Parse incoming message and fill out specified data structure
  imuData meas;
  meas.timestamp = msg->header.stamp.toSec();
  meas.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  meas.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // Push measurement into buffer
  imu_data_buffer_.emplace_back(meas);

  // Remove oldest if sensor reading window width is reached
  if ((meas.timestamp - imu_data_buffer_.begin()->timestamp) > opts_->sensor_readings_window) {

    // Since we check everytime a new measurement is added to the buffer it would
    // be sufficient to simply remove the first element, however it is more robust
    // to check everytime how many elements we should remove.

    double Dt = meas.timestamp - opts_->sensor_readings_window;

    // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
    auto it = std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(), [&Dt](imuData meas){return meas.timestamp >= Dt;});

    // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
    imu_data_buffer_.erase(imu_data_buffer_.begin(), it-1);
  }
}








bool AmazeAutonomy::checkFlatness() {

  // Return if buffer is empty
  if (imu_data_buffer_.empty()) {
      return false;
  }

  // Return if minimum window is not reached
  if ((imu_data_buffer_.end()->timestamp - imu_data_buffer_.begin()->timestamp) < opts_->sensor_readings_window) {
    return false;
  }

  // Define mean acceleration and mean angular velocity
  Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_mean = Eigen::Vector3d::Zero();

  // Calculate the mean acceleration and the mean angular velocity
  for (auto &it : imu_data_buffer_) {
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
  Eigen::Vector3d y_axis = skew(z_axis)*x_axis;

  // Get rotation of the imu using axes as columns
  // R_GI defines the rotation matrix that rotates
  // vector in IMU frame (I_x) to vector in global
  // inertial gravity-aligned frame (G_x = R_GI * I_x)
  Eigen::Matrix3d R_GI;
  R_GI.block(0,0,3,1) = -x_axis;
  R_GI.block(0,1,3,1) = -y_axis;
  R_GI.block(0,2,3,1) = z_axis;

  // Apply Rotation between the imu and the platform
  Eigen::Matrix3d R_GP = R_GI*Rot(opts_->R_IP);

  // Convert Rotation matrix to euler angles
  Eigen::Vector3d eul_ang = eul(R_GP);

  // Compare roll and pitch with thresholds
  if (abs(eul_ang(0)) > opts_->angle_threshold || abs(eul_ang(1)) > opts_->angle_threshold) {
    return false;
  }

  // test passed
  return true;
}

/************************************************************************/
