#ifndef F1TENTH_HARDWARE_INTERFACE_HPP_
#define F1TENTH_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float64.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace f1tenth_hardware
{
class F1TenthHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(F1TenthHardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  double wheel_base_;
  double wheel_radius_;
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double steering_angle_to_servo_gain_;
  double steering_angle_to_servo_offset_;
  
  // Hardware states and commands
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  
  // ROS2 node for communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_position_pub_;
  
  // Subscribers  
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

  // State variables
  double current_speed_;
  double current_steering_angle_;
  double position_x_;
  double position_y_;
  double orientation_z_;
  double linear_velocity_;
  double angular_velocity_;
  
  // Callbacks
  void vesc_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void servo_callback(const std_msgs::msg::Float64::SharedPtr msg);
  // Utility functions
  double speed_to_erpm(double speed);
  double erpm_to_speed(double erpm);
  double steering_angle_to_servo(double angle);
  double servo_to_steering_angle(double servo);
};

}  // namespace f1tenth_hardware

#endif  // F1TENTH_HARDWARE_INTERFACE_HPP_