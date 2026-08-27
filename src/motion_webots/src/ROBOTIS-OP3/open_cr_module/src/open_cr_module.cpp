/* Modified by Fayyadh for Kalman Filter and 9-axis support */

#include "open_cr_module/open_cr_module.h"

using namespace robotis_op;

OpenCRModule::OpenCRModule()
    : Node("open_cr_module"),
      control_cycle_msec_(8),
      DEBUG_PRINT(false),
      present_volt_(0.0),
      mag_calibrated_(false),
      accel_weight_(0.02),
      mag_weight_(0.01)
{
  module_name_ = "open_cr_module";  // set unique module name

  // Initialize sensor value storage
  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;
  
  // Initialize magnetometer values
  result_["mag_x"] = 0.0;
  result_["mag_y"] = 0.0;
  result_["mag_z"] = 0.0;

  result_["button_mode"] = 0;
  result_["button_start"] = 0;
  result_["button_user"] = 0;

  result_["present_voltage"] = 0.0;

  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["button_user"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;
  buttons_["published_user"] = false;

  // Initialize previous sensor values
  previous_result_["gyro_x"] = 0.0;
  previous_result_["gyro_y"] = 0.0;
  previous_result_["gyro_z"] = 0.0;

  previous_result_["acc_x"] = 0.0;
  previous_result_["acc_y"] = 0.0;
  previous_result_["acc_z"] = 0.0;
  
  previous_result_["mag_x"] = 0.0;
  previous_result_["mag_y"] = 0.0;
  previous_result_["mag_z"] = 0.0;

  last_msg_time_ = rclcpp::Clock().now();
  
  // Initialize Kalman Filter
  initKalmanFilter();
  
  // Initialize magnetometer calibration parameters
  mag_bias_ = Eigen::Vector3d::Zero();
  mag_scale_ = Eigen::Matrix3d::Identity();
}

OpenCRModule::~OpenCRModule()
{
  queue_thread_.join();
}

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = std::thread(&OpenCRModule::queueThread, this);
}

void OpenCRModule::initKalmanFilter()
{
  // Initialize Kalman filter state
  kalman_state_.gyro_bias = Eigen::Vector3d::Zero();
  kalman_state_.orientation = Eigen::Quaterniond::Identity();
  
  // Initialize process noise covariance (tuning parameter)
  process_noise_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
  
  // Initialize error covariance matrix (uncertainty)
  error_cov_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
}

void OpenCRModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* publisher */
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 1);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/robotis/open_cr/magnetic_field", 1);
  button_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/open_cr/button", 1);
  dxl_power_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 1);

  rclcpp::WallRate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void OpenCRModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, robotis_framework::Sensor *> sensors)
{
  // RCLCPP_INFO(this->get_logger(), "Process called, sensor available: %s", 
  //             (sensors["open-cr"] != NULL ? "yes" : "no"));
  
  if (sensors["open-cr"] == NULL)
    return;

  // Get raw sensor values
  int16_t gyro_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_x"];
  int16_t gyro_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_y"];
  int16_t gyro_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_z"];

  int16_t acc_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_x"];
  int16_t acc_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_y"];
  int16_t acc_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_z"];
  
  // Get magnetometer raw values (new for MPU9250)
  // int16_t mag_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_x"];
  // int16_t mag_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_y"];
  // int16_t mag_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_z"];
  int16_t mag_x = 0, mag_y = 0, mag_z = 0;
  
  // Try to get magnetometer values or provide debug info if missing
  try {
    mag_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_x"];
    mag_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_y"];
    mag_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_z"];
  } catch (const std::exception& e) {
    // If keys don't exist, provide dummy values
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                         "Magnetometer data not available, using dummy values");
    mag_x = 100; mag_y = 100; mag_z = 100; // Dummy values for testing
  }

  uint16_t present_volt = sensors["open-cr"]->sensor_state_->bulk_read_table_["present_voltage"];

  // Convert sensor values to physical units (rad/s, g, etc.)
  result_["gyro_x"] = -getGyroValue(gyro_x);
  result_["gyro_y"] = -getGyroValue(gyro_y);
  result_["gyro_z"] = getGyroValue(gyro_z);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Gyro ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT,"Raw : %d, %d, %d", gyro_x, gyro_y, gyro_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT,"Converted : %f, %f, %f", result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  // Align axis of Accelerometer to robot
  result_["acc_x"] = -getAccValue(acc_x);
  result_["acc_y"] = -getAccValue(acc_y);
  result_["acc_z"] = getAccValue(acc_z);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Acc ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", acc_x, acc_y, acc_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Converted : %f, %f, %f", result_["acc_x"], result_["acc_y"], result_["acc_z"]);
  
  // Process magnetometer data
  result_["mag_x"] = getMagValue(mag_x);
  result_["mag_y"] = getMagValue(mag_y);
  result_["mag_z"] = getMagValue(mag_z);
  
  // Apply magnetometer calibration
  if (mag_calibrated_) {
    applyHardIronCalibration(result_["mag_x"], result_["mag_y"], result_["mag_z"]);
  }
  
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Mag ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", mag_x, mag_y, mag_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Converted : %f, %f, %f", result_["mag_x"], result_["mag_y"], result_["mag_z"]);

  // Check sensor update time to ensure connection
  rclcpp::Time update_time(sensors["open-cr"]->sensor_state_->update_time_stamp_.sec_, sensors["open-cr"]->sensor_state_->update_time_stamp_.nsec_);
  rclcpp::Duration update_duration = rclcpp::Clock().now() - update_time;
  if ((update_duration.seconds() * 1000000000 + update_duration.nanoseconds()) > 100000000)
    publishDXLPowerMsg(1);

  // Calculate sensor update time for Kalman filter
  static rclcpp::Time last_update_time = update_time;
  double dt = (update_time - last_update_time).seconds();
  if (dt > 0.0001 && dt < 0.5) {  // Sanity check on dt
    updateKalmanFilter(dt);
  }
  last_update_time = update_time;
  
  // Process button states
  uint8_t button_flag = sensors["open-cr"]->sensor_state_->bulk_read_table_["button"];
  result_["button_mode"] = button_flag & 0x01;
  result_["button_start"] = (button_flag & 0x02) >> 1;
  result_["button_user"] = (button_flag & 0x04) >> 2;

  handleButton("mode");
  handleButton("start");
  handleButton("user");

  // Process voltage
  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  // Publish sensor data
  publishIMU();
  publishMagneticField();
}

// -2000 ~ 2000dps(-32800 ~ 32800), scale factor : 16.4, dps -> rps
double OpenCRModule::getGyroValue(int raw_value)
{
  return (double) raw_value * GYRO_FACTOR * DEGREE2RADIAN;
}

// -2.0 ~ 2.0g(-32768 ~ 32768), 1g = 9.8 m/s^2
double OpenCRModule::getAccValue(int raw_value)
{
  return (double) raw_value * ACCEL_FACTOR;
}

// Convert raw magnetometer value to physical units (microTesla)
double OpenCRModule::getMagValue(int raw_value)
{
  return (double) raw_value * MAG_FACTOR;
}

void OpenCRModule::updateKalmanFilter(double dt)
{
  if (dt <= 0)
    return;
    
  // Current measurements
  Eigen::Vector3d gyro(result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);
  Eigen::Vector3d accel(result_["acc_x"], result_["acc_y"], result_["acc_z"]);
  Eigen::Vector3d mag(result_["mag_x"], result_["mag_y"], result_["mag_z"]);
  
  // Step 1: Update state prediction based on gyroscope readings
  Eigen::Vector3d gyro_unbiased = gyro - kalman_state_.gyro_bias;
  
  // Convert gyroscope readings to quaternion derivative
  Eigen::Quaterniond omega_quat;
  omega_quat.w() = 0;
  omega_quat.x() = gyro_unbiased.x();
  omega_quat.y() = gyro_unbiased.y();
  omega_quat.z() = gyro_unbiased.z();
  
  // Integrate quaternion (first-order approximation)
  // Eigen::Quaterniond quat_derivative = kalman_state_.orientation * omega_quat * 0.5;
  // Eigen::Quaterniond quat_derivative = 0.5 * (kalman_state_.orientation * omega_quat);
  Eigen::Quaterniond quat_derivative = kalman_state_.orientation * omega_quat;
  quat_derivative.coeffs() *= 0.5;
  kalman_state_.orientation.w() += quat_derivative.w() * dt;
  kalman_state_.orientation.x() += quat_derivative.x() * dt;
  kalman_state_.orientation.y() += quat_derivative.y() * dt;
  kalman_state_.orientation.z() += quat_derivative.z() * dt;
  kalman_state_.orientation.normalize();
  
  // Step 2: Update error covariance matrix P (prediction step)
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity(); // System matrix for linearized system
  error_cov_ = A * error_cov_ * A.transpose() + process_noise_;
  
  // Step 3: Calculate expected acceleration and magnetometer readings based on orientation
  Eigen::Vector3d expected_accel = kalman_state_.orientation.inverse() * Eigen::Vector3d(0, 0, 1);
  
  // Reference magnetic field direction (varies by location - this is approximation for Northern Hemisphere)
  Eigen::Vector3d mag_reference(0.0, 1.0, 0.0); // Magnetic north is approximately aligned with Y-axis
  Eigen::Vector3d expected_mag = kalman_state_.orientation.inverse() * mag_reference;
  
  // Step 4: Calculate measurement error
  Eigen::Vector3d accel_norm = accel.normalized();
  Eigen::Vector3d accel_error = accel_norm.cross(expected_accel);
  
  // Normalize magnetometer reading for direction comparison
  Eigen::Vector3d mag_norm = mag.normalized();
  Eigen::Vector3d mag_error = mag_norm.cross(expected_mag);
  
  // Full measurement error vector
  Eigen::Matrix<double, 6, 1> measurement_error;
  measurement_error.block<3, 1>(0, 0) = accel_error * accel_weight_;
  measurement_error.block<3, 1>(3, 0) = mag_error * mag_weight_;
  
  // Step 5: Calculate Kalman gain
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity(); // Measurement matrix
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 0.1; // Measurement noise (tunable)
  
  Eigen::Matrix<double, 6, 6> S = H * error_cov_ * H.transpose() + R;
  Eigen::Matrix<double, 6, 6> K = error_cov_ * H.transpose() * S.inverse();
  
  // Step 6: Update state based on measurements
  Eigen::Matrix<double, 6, 1> state_correction = K * measurement_error;
  
  // Apply corrections to gyro bias
  kalman_state_.gyro_bias += state_correction.block<3, 1>(0, 0);
  
  // Apply orientation correction (convert to quaternion)
  Eigen::Vector3d orientation_correction = state_correction.block<3, 1>(3, 0);
  if (orientation_correction.norm() > 1e-10) {
    // Convert the orientation correction to a quaternion
    double angle = orientation_correction.norm();
    Eigen::Vector3d axis = orientation_correction.normalized();
    
    Eigen::Quaterniond q_correction(Eigen::AngleAxisd(angle, axis));
    kalman_state_.orientation = kalman_state_.orientation * q_correction;
    kalman_state_.orientation.normalize();
  }
  
  // Step 7: Update error covariance matrix (correction step)
  Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
  error_cov_ = (I - K * H) * error_cov_;
}

void OpenCRModule::publishIMU()
{
  // Get current time
  imu_msg_.header.stamp = this->get_clock()->now();
  imu_msg_.header.frame_id = "body_link";
  
  // Set angular velocity (gyro with bias correction)
  Eigen::Vector3d unbiased_gyro(
    result_["gyro_x"] - kalman_state_.gyro_bias.x(),
    result_["gyro_y"] - kalman_state_.gyro_bias.y(),
    result_["gyro_z"] - kalman_state_.gyro_bias.z()
  );
  
  imu_msg_.angular_velocity.x = unbiased_gyro.x();
  imu_msg_.angular_velocity.y = unbiased_gyro.y();
  imu_msg_.angular_velocity.z = unbiased_gyro.z();

  // Set linear acceleration (in m/s^2)
  imu_msg_.linear_acceleration.x = result_["acc_x"] * G_ACC;
  imu_msg_.linear_acceleration.y = result_["acc_y"] * G_ACC;
  imu_msg_.linear_acceleration.z = result_["acc_z"] * G_ACC;

  // Set orientation from Kalman filter
  imu_msg_.orientation.w = kalman_state_.orientation.w();
  imu_msg_.orientation.x = kalman_state_.orientation.x();
  imu_msg_.orientation.y = kalman_state_.orientation.y();
  imu_msg_.orientation.z = kalman_state_.orientation.z();
  
  // Set covariance (from error_cov_ matrix)
  // For simplicity, we'll just set some reasonable values
  for (int i = 0; i < 9; i++) {
    imu_msg_.orientation_covariance[i] = 0.001;
    imu_msg_.angular_velocity_covariance[i] = 0.002;
    imu_msg_.linear_acceleration_covariance[i] = 0.005;
  }

  imu_pub_->publish(imu_msg_);
}

void OpenCRModule::publishMagneticField()
{
  mag_msg_.header.stamp = this->get_clock()->now();
  mag_msg_.header.frame_id = "body_link";
  
  // Convert to Tesla (from microTesla) -> still using microTesla
  mag_msg_.magnetic_field.x = result_["mag_x"]; // hapus * 1e-6
  mag_msg_.magnetic_field.y = result_["mag_y"]; // hapus * 1e-6
  mag_msg_.magnetic_field.z = result_["mag_z"]; // hapus * 1e-6
  
  // Set covariance (reasonable defaults)
  for (int i = 0; i < 9; i++) {
    mag_msg_.magnetic_field_covariance[i] = 0.01;
  }
  
  mag_pub_->publish(mag_msg_);
}

void OpenCRModule::calibrateMagnetometer()
{
  // Simple magnetometer calibration routine
  // This would typically require collecting measurements in different orientations
  // For simplicity, we'll just use default values
  RCLCPP_INFO(this->get_logger(), "Starting magnetometer calibration...");
  
  // Hard iron calibration values (should be determined experimentally)
  mag_bias_ << 0.0, 0.0, 0.0;
  
  // Soft iron calibration (identity matrix as default)
  mag_scale_ = Eigen::Matrix3d::Identity();
  
  mag_calibrated_ = true;
  RCLCPP_INFO(this->get_logger(), "Magnetometer calibration completed.");
}

void OpenCRModule::applyHardIronCalibration(double &mag_x, double &mag_y, double &mag_z)
{
  // Apply hard iron correction
  Eigen::Vector3d mag_corrected;
  mag_corrected << mag_x, mag_y, mag_z;
  
  // Remove bias (hard iron distortion)
  mag_corrected = mag_corrected - mag_bias_;
  
  // Apply scaling (soft iron distortion)
  mag_corrected = mag_scale_ * mag_corrected;
  
  // Update values
  mag_x = mag_corrected.x();
  mag_y = mag_corrected.y();
  mag_z = mag_corrected.z();
}

void OpenCRModule::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;
  std::string button_published = "published_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  // same state
  if (buttons_[button_key] == pushed)
  {
    if (pushed == true && buttons_[button_published] == false)
    {
      // check long press
      rclcpp::Duration button_duration = rclcpp::Clock().now() - buttons_press_time_[button_name];
      if (button_duration.seconds() > 2.0)
      {
        publishButtonMsg(button_name + "_long");
        buttons_[button_published] = true;
      }
    }
  }
  else    // state is changed
  {
    buttons_[button_key] = pushed;

    if (pushed == true)
    {
      buttons_press_time_[button_name] = rclcpp::Clock().now();
      buttons_[button_published] = false;
    }
    else
    {
      rclcpp::Duration button_duration = rclcpp::Clock().now() - buttons_press_time_[button_name];

      if (button_duration.seconds() < 2.0)     // short press
        publishButtonMsg(button_name);
      else
        // long press
        ;
    }
  }
}

void OpenCRModule::publishButtonMsg(const std::string &button_name)
{
  std_msgs::msg::String button_msg;
  button_msg.data = button_name;

  button_pub_->publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void OpenCRModule::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ =
      (previous_volt_ != 0) ? previous_volt_ * (1 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (fabs(present_volt_ - previous_volt_) >= 0.1)
  {
    // check last published time
    rclcpp::Time now = rclcpp::Clock().now();
    rclcpp::Duration dur = now - last_msg_time_;
    if (dur.seconds() < 1)
      return;

    last_msg_time_ = now;

    present_volt_ = previous_volt_;
    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg(
        (present_volt_ < 11 ? 
            robotis_controller_msgs::msg::StatusMsg::STATUS_WARN : robotis_controller_msgs::msg::StatusMsg::STATUS_INFO),
        log_stream.str());
    RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
  }
}

void OpenCRModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::msg::StatusMsg status_msg;
  status_msg.header.stamp = this->get_clock()->now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

void OpenCRModule::publishDXLPowerMsg(unsigned int value)
{
  robotis_controller_msgs::msg::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "dynamixel_power";
  sync_write_msg.joint_name.push_back("open-cr");
  sync_write_msg.value.push_back(value);

  dxl_power_msg_pub_->publish(sync_write_msg);
}