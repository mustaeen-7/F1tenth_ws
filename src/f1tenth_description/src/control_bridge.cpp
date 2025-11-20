#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

class ControlBridge : public rclcpp::Node
{
public:
  ControlBridge()
  : Node("control_bridge")
  {
    // Declare parameters for topic names and wheelbase
    this->declare_parameter<std::string>("twist_topic", "/cmd_vel");
    this->declare_parameter<std::string>("ackermann_topic", "/drive");
    this->declare_parameter<double>("wheelbase", 0.25);
    this->declare_parameter<double>("update_rate", 50.0);
    this->declare_parameter<double>("max_steering_angle", 0.5236); // 30 degrees in radians
    this->declare_parameter<bool>("allow_reverse", true);          // NEW: Allow reverse parameter
    this->declare_parameter<double>("max_speed", 2.0);             // NEW: Max speed limit
    this->declare_parameter<double>("max_reverse_speed", 1.0);     // NEW: Max reverse speed
    
    // Get the parameter values
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    update_rate_ = this->get_parameter("update_rate").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    allow_reverse_ = this->get_parameter("allow_reverse").as_bool();
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_reverse_speed_ = this->get_parameter("max_reverse_speed").as_double();
    auto twist_topic = this->get_parameter("twist_topic").as_string();
    auto ackermann_topic = this->get_parameter("ackermann_topic").as_string();
    
    // Publisher for AckermannDriveStamped messages
    pub_ackermann_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);
    
    // Subscribe to the cmd_vel topic (geometry_msgs::msg::Twist)
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      twist_topic, 10,
      std::bind(&ControlBridge::twist_callback, this, std::placeholders::_1));
    
    // Create a timer to check and process cmd_vel messages at a higher frequency
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
      std::bind(&ControlBridge::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Control Bridge initialized - Reverse %s!", 
                allow_reverse_ ? "ENABLED" : "DISABLED");
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Store the most recent message to process later
    latest_msg_ = msg;
  }

  void timer_callback()
  {
    if (!latest_msg_)
      return; // If no message has been received, exit early

    // Process the latest Twist message
    const auto& msg = latest_msg_;
   
    // If both linear.x and angular.z are near zero, stop the robot
    if (std::abs(msg->linear.x) < 1e-6 && std::abs(msg->angular.z) < 1e-6)
    {
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = this->now();
      drive_msg.header.frame_id = "base_link";
      drive_msg.drive.speed = 0.0; // Stop speed
      drive_msg.drive.steering_angle = 0.0; // Stop steering
      
      // Publish the stop message
      pub_ackermann_->publish(drive_msg);
      return;
    }

    double v = msg->linear.x; // Linear velocity (now supports reverse!)
    double omega = msg->angular.z; // Angular velocity
    double steering = 0.0;

    // FIXED: Apply speed limits with reverse support
    if (allow_reverse_) {
      // Clamp velocity to allowed range (both forward and reverse)
      v = std::max(-max_reverse_speed_, std::min(max_speed_, v));
    } else {
      // Original behavior - forward only
      v = std::max(0.0, std::min(max_speed_, v));
    }

    // FIXED: Handle forward, reverse, and turning correctly
    if (std::abs(omega) > 1e-5 && std::abs(v) > 1e-5)
    {
      // Calculate steering angle using Ackermann geometry
      double radius = v / omega;
      steering = std::atan(wheelbase_ / radius);
      
      // Clamp steering angle to maximum allowed
      steering = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering));
      
      RCLCPP_DEBUG(this->get_logger(), "Motion: v=%.2f, omega=%.2f, steering=%.2f", v, omega, steering);
    }
    else if (std::abs(omega) > 1e-5 && std::abs(v) <= 1e-5)
    {
      // Pure rotation - set maximum steering angle in direction of rotation
      steering = (omega > 0) ? max_steering_angle_ : -max_steering_angle_;
      
      // FIXED: Use small speed for in-place rotation (respecting reverse setting)
      if (allow_reverse_) {
        v = 0.1; // Small forward velocity for rotation (can be changed to -0.1 if needed)
      } else {
        v = 0.1; // Small forward velocity for rotation
      }
      
      RCLCPP_DEBUG(this->get_logger(), "Pure rotation: omega=%.2f, steering=%.2f, v=%.2f", omega, steering, v);
    }

    // Create and populate the AckermannDriveStamped message
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.speed = v; // Set speed (positive for forward, negative for reverse)
    drive_msg.drive.steering_angle = steering; // Set steering angle
    
    // Debug output with direction indication
    const char* direction = (v > 0) ? "FORWARD" : (v < 0) ? "REVERSE" : "STOP";
    RCLCPP_DEBUG(this->get_logger(), "Publishing: speed=%.2f (%s), steering=%.2f", v, direction, steering);

    // Publish the Ackermann drive message
    pub_ackermann_->publish(drive_msg);
  }

  double wheelbase_;
  double update_rate_;
  double max_steering_angle_;
  bool allow_reverse_;          // NEW: Allow reverse flag
  double max_speed_;            // NEW: Max forward speed
  double max_reverse_speed_;    // NEW: Max reverse speed
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_ackermann_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist::SharedPtr latest_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlBridge>());
  rclcpp::shutdown();
  return 0;
}