#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>

class RealSenseIMUCombiner : public rclcpp::Node
{
public:
    RealSenseIMUCombiner() : Node("realsense_imu_combiner")
    {
        // Publisher for combined IMU data
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/camera/imu", 10);
        
        // After: Explicitly uses Best Effort QoS
        accel_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/realsense2_camera_node/accel/sample",
            rclcpp::QoS(10).best_effort(), // QoS Profile
            std::bind(&RealSenseIMUCombiner::accel_callback, this, std::placeholders::_1)
        );
        
        gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/realsense2_camera_node/gyro/sample",
            rclcpp::QoS(10).best_effort(), // QoS Profile
            std::bind(&RealSenseIMUCombiner::gyro_callback, this, std::placeholders::_1)
        );
        
        // Timer for publishing combined IMU at consistent rate (100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RealSenseIMUCombiner::publish_combined_imu, this)
        );
        
        // Initialize pointers
        latest_accel_ = nullptr;
        latest_gyro_ = nullptr;
        
        RCLCPP_INFO(this->get_logger(), "RealSense IMU Combiner Node started");
    }

private:
    void accel_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_accel_ = msg;
    }
    
    void gyro_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_gyro_ = msg;
    }
    
    void publish_combined_imu()
    {
        // Check if we have data from both sensors
        if (!latest_accel_ || !latest_gyro_) {
            return;
        }
        
        // Create combined IMU message
        auto combined_imu = sensor_msgs::msg::Imu();
        
        // <<< FIX START >>>
        // Create rclcpp::Time objects for comparison
        rclcpp::Time accel_stamp(latest_accel_->header.stamp);
        rclcpp::Time gyro_stamp(latest_gyro_->header.stamp);
        
        // Use the more recent timestamp
        if (accel_stamp > gyro_stamp) {
            combined_imu.header = latest_accel_->header;
        } else {
            combined_imu.header = latest_gyro_->header;
        }
        // <<< FIX END >>>
        
        combined_imu.header.frame_id = "camera_imu_optical_frame";
        
        // Linear acceleration from accelerometer
        combined_imu.linear_acceleration = latest_accel_->linear_acceleration;
        combined_imu.linear_acceleration_covariance = latest_accel_->linear_acceleration_covariance;
        
        // Angular velocity from gyroscope
        combined_imu.angular_velocity = latest_gyro_->angular_velocity;
        combined_imu.angular_velocity_covariance = latest_gyro_->angular_velocity_covariance;
        
        // Orientation (not provided by RealSense, set to identity quaternion)
        combined_imu.orientation.x = 0.0;
        combined_imu.orientation.y = 0.0;
        combined_imu.orientation.z = 0.0;
        combined_imu.orientation.w = 1.0;
        
        // High covariance indicates orientation is not reliable
        std::array<double, 9> orientation_cov = {
            999999.0, 0.0, 0.0,
            0.0, 999999.0, 0.0,
            0.0, 0.0, 999999.0
        };
        combined_imu.orientation_covariance = orientation_cov;
        
        // Publish combined message
        imu_pub_->publish(combined_imu);
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub_;
    
    // Timer for consistent publishing
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Storage for latest sensor data
    sensor_msgs::msg::Imu::SharedPtr latest_accel_;
    sensor_msgs::msg::Imu::SharedPtr latest_gyro_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseIMUCombiner>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}