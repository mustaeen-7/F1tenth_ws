#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class SteeringJointStatePublisher : public rclcpp::Node
{
public:
  SteeringJointStatePublisher()
  : Node("steering_joint_state_publisher"), steering_angle_(0.0)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/sensors/servo_position_command", 10,
      std::bind(&SteeringJointStatePublisher::steering_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SteeringJointStatePublisher::publish_joint_state, this)
    );
  }

private:
  void steering_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    steering_angle_ = msg->data;
  }

  void publish_joint_state()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"front_right_hinge_to_wheel", "front_right_hinge_to_wheel"};  // match your URDF joint names
    msg.position = {steering_angle_, steering_angle_};
    joint_pub_->publish(msg);
  }

  double steering_angle_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
