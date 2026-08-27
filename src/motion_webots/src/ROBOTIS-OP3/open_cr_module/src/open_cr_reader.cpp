#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/string.hpp>

class OpenCRReader : public rclcpp::Node
{
public:
  OpenCRReader() : Node("open_cr_reader")
  {
    // Subscribe ke topic IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/robotis/open_cr/imu", 10, 
      std::bind(&OpenCRReader::imu_callback, this, std::placeholders::_1));
      
    // Subscribe ke topic magnetic field
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/robotis/open_cr/magnetic_field", 10, 
      std::bind(&OpenCRReader::mag_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "OpenCR Reader node started - Raw Data Mode");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Tampilkan data IMU sebagai angka saja menggunakan logger
    RCLCPP_INFO(this->get_logger(), 
      "ACC: %.3f %.3f %.3f | GYRO: %.3f %.3f %.3f",
      msg->linear_acceleration.x, 
      msg->linear_acceleration.y, 
      msg->linear_acceleration.z,
      msg->angular_velocity.x, 
      msg->angular_velocity.y, 
      msg->angular_velocity.z);
  }
  
  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    // Tampilkan data magnetometer sebagai angka saja menggunakan logger
    RCLCPP_INFO(this->get_logger(),
      "MAG: %.3f %.3f %.3f",
      msg->magnetic_field.x, 
      msg->magnetic_field.y, 
      msg->magnetic_field.z);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCRReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}