/* Author: Fayyadh - Modified to include Kalman Filter and 9-axis support */

#ifndef OP3_OPEN_CR_MODULE_H_
#define OP3_OPEN_CR_MODULE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"

#include <Eigen/Dense>

namespace robotis_op
{

// Kalman Filter state definition
struct KalmanState {
    Eigen::Vector3d gyro_bias;  // Gyroscope bias estimation
    Eigen::Quaterniond orientation;  // Quaternion orientation
};

class OpenCRModule : public robotis_framework::SensorModule, public robotis_framework::Singleton<OpenCRModule>, public rclcpp::Node
{
public:
  OpenCRModule();
  virtual ~OpenCRModule();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  const double G_ACC = 9.80665;
  const double GYRO_FACTOR = 2000.0 / 32800.0;
  const double ACCEL_FACTOR = 2.0 / 32768.0;
  const double MAG_FACTOR = 4912.0 / 32768.0; // Magnetometer sensitivity factor (adjust as needed for MPU9250)
  const bool DEBUG_PRINT;

  void queueThread();

  // Sensor conversion methods
  double getGyroValue(int raw_value);
  double getAccValue(int raw_value);
  double getMagValue(int raw_value);
  
  // Calibration and preprocessing methods
  void calibrateMagnetometer();
  void applyHardIronCalibration(double &mag_x, double &mag_y, double &mag_z);
  
  // Publishing methods
  void publishIMU();
  void publishMagneticField();

  // Kalman Filter implementation
  void initKalmanFilter();
  void updateKalmanFilter(double dt);
  
  // Button and voltage handling
  void handleButton(const std::string &button_name);
  void publishButtonMsg(const std::string &button_name);
  void handleVoltage(double present_volt);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDXLPowerMsg(unsigned int value);

  // Basic module variables
  int control_cycle_msec_;
  std::thread queue_thread_;
  std::map<std::string, bool> buttons_;
  std::map<std::string, rclcpp::Time> buttons_press_time_;
  rclcpp::Time button_press_time_;
  rclcpp::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  double previous_volt_;
  double present_volt_;

  // IMU Message variables
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;

  // Kalman Filter variables
  KalmanState kalman_state_;
  Eigen::Matrix<double, 6, 6> process_noise_; // Process noise covariance matrix
  Eigen::Matrix<double, 6, 6> error_cov_;     // Error covariance matrix
  
  // Magnetometer calibration variables
  Eigen::Vector3d mag_bias_;    // Hard iron calibration vector
  Eigen::Matrix3d mag_scale_;   // Soft iron calibration matrix
  bool mag_calibrated_;
  
  // Sensor fusion parameters
  double accel_weight_;  // Weight for accelerometer in orientation estimation
  double mag_weight_;    // Weight for magnetometer in orientation estimation

  /* ROS subscriber & publisher */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr dxl_power_msg_pub_;
};

} // namespace robotis_op

#endif /* OP3_OPEN_CR_MODULE_H_ */