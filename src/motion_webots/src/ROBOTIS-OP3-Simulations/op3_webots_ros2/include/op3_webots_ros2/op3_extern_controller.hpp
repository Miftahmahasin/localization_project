#ifndef OP3_WEBOTS_ROS2_OP3_EXTERN_CONTROLLER_HPP
#define OP3_WEBOTS_ROS2_OP3_EXTERN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <webots/Supervisor.hpp>
#include <yaml-cpp/yaml.h>

#define N_MOTORS (20)

namespace robotis_op
{

class OP3ExternController : public webots::Supervisor, public rclcpp::Node
{
public:
  OP3ExternController();
  virtual ~OP3ExternController();

  void initialize(std::string gain_file_path);
  void process();
  void stepWebots();

  void setDesiredJointAngles();
  void getPresentJointAngles();
  void getPresentJointTorques();

  void getCurrentRobotCOM();
  void getIMUOutput();

  void publishPresentJointStates();
  void publishIMUOutput();
  void publishCOMData();
  void publishCameraData();

  void publishGroundTruth();

  void posCommandCallback(
    const std_msgs::msg::Float64::SharedPtr msg,
    int joint_idx
  );

  // Supervisor teleport: dataset generation places the robot at many poses.
  // The callback (ROS thread) only stores the target; applyPendingTeleport()
  // runs in the main Webots thread (Webots node/field writes are not thread-safe).
  void teleportCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  void applyPendingTeleport();
  // live-tunable downward spawn offset (meters); avoids restarting Webots
  void spawnZOffsetCallback(const std_msgs::msg::Float64::SharedPtr msg);
  // sim-to-real render domain randomization (lighting/ambient/background).
  // Callback (ROS thread) stores values; applyPendingLighting() runs main-thread.
  void lightingCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void applyPendingLighting();

  void queueThread();
  bool parsePIDGainYAML(std::string gain_file_path);

  int time_step_ms_;
  double time_step_sec_;
  double current_time_sec_;

  double desired_joint_angle_rad_[N_MOTORS];
  double current_joint_angle_rad_[N_MOTORS];
  double current_joint_torque_Nm_[N_MOTORS];
  bool desired_joint_angle_rcv_flag_[N_MOTORS];

  sensor_msgs::msg::JointState joint_state_msg_;

  double current_com_m_[3];
  double previous_com_m_[3];
  double current_com_vel_mps_[3];
  geometry_msgs::msg::Vector3 com_m_;

  sensor_msgs::msg::Imu imu_data_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  sensor_msgs::msg::Image image_data_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr com_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr present_joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr set_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_spawn_z_sub_;
  std::mutex teleport_mutex_;
  bool teleport_pending_ = false;
  double teleport_x_ = 0.0;
  double teleport_y_ = 0.0;
  double teleport_theta_ = 0.0;
  double spawn_z_offset_ = 0.0;    // guarded by teleport_mutex_; lowers spawn z
  double stand_z_ref_ = 0.0;       // main thread only: captured standing height
  bool stand_z_captured_ = false;  // main thread only

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lighting_sub_;
  std::mutex lighting_mutex_;
  bool lighting_pending_ = false;
  std::vector<double> lighting_vals_;   // [amb, int, cr,cg,cb, dx,dy,dz, sr,sg,sb]

  webots::Camera* camera_;
  webots::LED* head_led_;
  webots::LED* body_led_;
  webots::Speaker* speaker_;
  webots::Keyboard* key_board_;

  webots::Gyro* gyro_;
  webots::Accelerometer* acc_;
  webots::InertialUnit* iu_;

  webots::Motor* motors_[N_MOTORS];
  webots::PositionSensor* encoders_[N_MOTORS];

  std::thread queue_thread_;
};

}  

#endif