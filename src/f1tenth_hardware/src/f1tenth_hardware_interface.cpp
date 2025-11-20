#include "f1tenth_hardware/f1tenth_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f1tenth_hardware
{
hardware_interface::CallbackReturn F1TenthHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize parameters with defaults if not provided
  try {
    wheel_base_ = std::stod(info_.hardware_parameters.at("wheel_base"));
  } catch (const std::exception& e) {
    wheel_base_ = 0.33;  // Default F1TENTH wheelbase
  }
  
  try {
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  } catch (const std::exception& e) {
    wheel_radius_ = 0.0508;  // Default F1TENTH wheel radius
  }
  
  try {
    speed_to_erpm_gain_ = std::stod(info_.hardware_parameters.at("speed_to_erpm_gain"));
  } catch (const std::exception& e) {
    speed_to_erpm_gain_ = 4614.0;  // Default gain
  }
  
  try {
    speed_to_erpm_offset_ = std::stod(info_.hardware_parameters.at("speed_to_erpm_offset"));
  } catch (const std::exception& e) {
    speed_to_erpm_offset_ = 0.0;  // Default offset
  }
  
  try {
    steering_angle_to_servo_gain_ = std::stod(info_.hardware_parameters.at("steering_angle_to_servo_gain"));
  } catch (const std::exception& e) {
    steering_angle_to_servo_gain_ = -1.2135;  // Default gain
  }
  
  try {
    steering_angle_to_servo_offset_ = std::stod(info_.hardware_parameters.at("steering_angle_to_servo_offset"));
  } catch (const std::exception& e) {
    steering_angle_to_servo_offset_ = 0.5304;  // Default offset
  }

  // Initialize ONLY state vectors (no commands needed)
  hw_states_.resize(info_.joints.size() * 2, 0.0);

  // Initialize state variables
  current_speed_ = 0.0;
  current_steering_angle_ = 0.0;
  position_x_ = 0.0;
  position_y_ = 0.0;
  orientation_z_ = 0.0;
  linear_velocity_ = 0.0;
  angular_velocity_ = 0.0;

  // Initialize joint positions to zero
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    hw_states_[i * 2] = 0.0;     // position
    hw_states_[i * 2 + 1] = 0.0; // velocity
  }

  RCLCPP_INFO(
    rclcpp::get_logger("F1TenthHardwareInterface"),
    "Successfully initialized F1TENTH hardware interface (STATE ONLY) with %zu joints", info_.joints.size());

  // Log joint names for debugging
  for (const auto& joint : info_.joints) {
    RCLCPP_INFO(rclcpp::get_logger("F1TenthHardwareInterface"), "Joint: %s", joint.name.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn F1TenthHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create node for ROS2 communication
  node_ = rclcpp::Node::make_shared("f1tenth_hardware_node");
  
  // Create executor and start in separate thread
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // REMOVED: No publishers needed since we're not sending commands
  // motor_speed_pub_ = ...
  // servo_position_pub_ = ...

  // Initialize subscribers for state feedback
  vesc_state_sub_ = node_->create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core", 10,
    std::bind(&F1TenthHardwareInterface::vesc_state_callback, this, std::placeholders::_1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&F1TenthHardwareInterface::odom_callback, this, std::placeholders::_1));

  // Subscribe to servo position commands for real-time steering feedback
  servo_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "commands/servo/position", 10,
    std::bind(&F1TenthHardwareInterface::servo_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    rclcpp::get_logger("F1TenthHardwareInterface"),
    "Successfully configured F1TENTH hardware interface (STATE ONLY)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> F1TenthHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export ONLY joint state interfaces (position and velocity)
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i * 2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i * 2 + 1]));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("F1TenthHardwareInterface"),
    "Exported %zu state interfaces (position and velocity only)", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> F1TenthHardwareInterface::export_command_interfaces()
{
  // EMPTY: No command interfaces exported - this hardware interface is READ-ONLY
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  RCLCPP_INFO(
    rclcpp::get_logger("F1TenthHardwareInterface"),
    "No command interfaces exported - this is a STATE-ONLY hardware interface");

  return command_interfaces;
}

hardware_interface::CallbackReturn F1TenthHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Ensure all states are initialized to zero
  for (auto i = 0u; i < hw_states_.size(); i++)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0.0;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("F1TenthHardwareInterface"),
    "Successfully activated F1TENTH hardware interface (STATE ONLY)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn F1TenthHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // REMOVED: No commands to send since this is state-only
  // No motor_speed_pub_ or servo_position_pub_ to publish to

  // Stop executor thread
  if (executor_thread_.joinable())
  {
    executor_->cancel();
    executor_thread_.join();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("F1TenthHardwareInterface"),
    "Successfully deactivated F1TENTH hardware interface (STATE ONLY)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type F1TenthHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Update joint states based on actual joint names from your URDF
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const std::string& joint_name = info_.joints[i].name;
    
    // Handle wheel joints - both front and back wheels
    if (joint_name.find("wheel") != std::string::npos)
    {
      if (joint_name.find("back") != std::string::npos)
      {
        // Back wheels - driven by motor (INVERTED to match real wheel direction)
        hw_states_[i * 2] += (-linear_velocity_ / wheel_radius_) * period.seconds(); // position
        hw_states_[i * 2 + 1] = -linear_velocity_ / wheel_radius_; // velocity
      }
      else if (joint_name.find("front") != std::string::npos)
      {
        // Front wheels - affected by steering (INVERTED to match real wheel direction)
        double effective_velocity = linear_velocity_ * std::cos(current_steering_angle_);
        hw_states_[i * 2] += (-effective_velocity / wheel_radius_) * period.seconds(); // position
        hw_states_[i * 2 + 1] = -effective_velocity / wheel_radius_; // velocity
      }
    }
    // Handle steering hinge joints
    else if (joint_name.find("hinge") != std::string::npos)
    {
      hw_states_[i * 2] = current_steering_angle_; // position
      hw_states_[i * 2 + 1] = 0.0; // velocity (quasi-static steering)
    }
    else
    {
      // Unknown joint type - keep current values
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("F1TenthHardwareInterface"),
        *node_->get_clock(), 5000,
        "Unknown joint type: %s", joint_name.c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type F1TenthHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // EMPTY: No write operations needed since this is a STATE-ONLY interface
  // All control commands are handled by separate VESC nodes
  
  return hardware_interface::return_type::OK;
}

// Callback implementations
void F1TenthHardwareInterface::vesc_state_callback(
  const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  // Update motor speed from ERPM
  current_speed_ = erpm_to_speed(msg->state.speed);
  linear_velocity_ = current_speed_;
}

void F1TenthHardwareInterface::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update position and orientation from odometry
  position_x_ = msg->pose.pose.position.x;
  position_y_ = msg->pose.pose.position.y;
  
  // Extract yaw from quaternion
  double qw = msg->pose.pose.orientation.w;
  double qz = msg->pose.pose.orientation.z;
  orientation_z_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
  
  // Update velocities
  linear_velocity_ = msg->twist.twist.linear.x;
  angular_velocity_ = msg->twist.twist.angular.z;
  
  // Calculate steering angle from angular velocity using Ackermann model (only as backup)
  if (std::abs(linear_velocity_) > 0.001)
  {
    double calculated_steering = std::atan(wheel_base_ * angular_velocity_ / linear_velocity_);
    // Only update if we don't have direct servo feedback or if the calculation is significantly different
    if (std::abs(current_steering_angle_) < 0.001 || 
        std::abs(calculated_steering - current_steering_angle_) > 0.1)
    {
      current_steering_angle_ = calculated_steering;
    }
  }
}

// NEW: Direct servo position callback for immediate steering updates
void F1TenthHardwareInterface::servo_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  // Convert servo position back to steering angle for immediate feedback
  current_steering_angle_ = servo_to_steering_angle(msg->data);
}

// Utility function implementations (kept for internal calculations)
double F1TenthHardwareInterface::speed_to_erpm(double speed)
{
  return speed * speed_to_erpm_gain_ + speed_to_erpm_offset_;
}

double F1TenthHardwareInterface::erpm_to_speed(double erpm)
{
  return (erpm - speed_to_erpm_offset_) / speed_to_erpm_gain_;
}

double F1TenthHardwareInterface::steering_angle_to_servo(double angle)
{
  return angle * steering_angle_to_servo_gain_ + steering_angle_to_servo_offset_;
}

double F1TenthHardwareInterface::servo_to_steering_angle(double servo)
{
  return (servo - steering_angle_to_servo_offset_) / steering_angle_to_servo_gain_;
}

}  // namespace f1tenth_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  f1tenth_hardware::F1TenthHardwareInterface, hardware_interface::SystemInterface)