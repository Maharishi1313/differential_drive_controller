#include <iostream>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std;

class WheelRPMCalculator : public rclcpp::Node {
public:
    WheelRPMCalculator() : Node("repub_node") {
        this->declare_parameter("wheel_base", 0.17);
        this->declare_parameter("wheel_radius", 0.033);
        this->declare_parameter("max_rpm", 100.0);
        
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WheelRPMCalculator::cmdVelCallback, this, std::placeholders::_1));
        
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_velocity = msg->linear.x; // m/s
        double angular_velocity = msg->angular.z; // rad/s

        // Define the speed conversion matrix
        std::array<std::array<double, 2>, 2> speed_conversion = {{
            {wheel_radius_ / 2, wheel_radius_ / 2},
            {wheel_radius_ / wheel_base_, -wheel_radius_ / wheel_base_}
        }};

        // Compute determinant
        double det = (speed_conversion[0][0] * speed_conversion[1][1]) - 
                     (speed_conversion[0][1] * speed_conversion[1][0]);

        // Compute inverse of 2x2 matrix (if determinant is non-zero)
        if (det == 0) {
            RCLCPP_ERROR(this->get_logger(), "Matrix inversion failed! Determinant is zero.");
            return;
        }

        std::array<std::array<double, 2>, 2> speed_conversion_inv = {{
            {speed_conversion[1][1] / det, -speed_conversion[0][1] / det},
            {-speed_conversion[1][0] / det, speed_conversion[0][0] / det}
        }};

        // Define robot speed as a vector
        std::array<double, 2> robot_speed = {linear_velocity, angular_velocity};

        // Perform matrix-vector multiplication: wheel_speed = speed_conversion_inv * robot_speed
        std::array<double, 2> wheel_speed = {
            (speed_conversion_inv[0][0] * robot_speed[0]) + (speed_conversion_inv[0][1] * robot_speed[1]),
            (speed_conversion_inv[1][0] * robot_speed[0]) + (speed_conversion_inv[1][1] * robot_speed[1])
        };

        // Convert wheel speeds to RPM
        double left_rpm = wheel_speed[1] * (60 / (2 * 3.14));
        double right_rpm = wheel_speed[0] * (60 / (2 * 3.14));

        // Limit RPM values
        left_rpm = std::max(std::min(left_rpm, max_rpm_), -max_rpm_);
        right_rpm = std::max(std::min(right_rpm, max_rpm_), -max_rpm_);

        // Publish RPM values
        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;
        left_msg.data = left_rpm;
        right_msg.data = right_rpm;

        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);
    }
    
    double wheel_base_;
    double wheel_radius_;
    double max_rpm_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelRPMCalculator>());
    rclcpp::shutdown();
    return 0;
}
