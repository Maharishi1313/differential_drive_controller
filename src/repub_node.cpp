#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <algorithm>

class WheelRPMCalculator : public rclcpp::Node {
public:
    WheelRPMCalculator() : Node("repub_node") {
        this->declare_parameter("wheel_base", 0.287);
        this->declare_parameter("wheel_radius", 0.033);
        this->declare_parameter("max_rpm", 150.0);
        
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
        
        double v_left = linear_velocity - (angular_velocity * wheel_base_ / 2.0);
        double v_right = linear_velocity + (angular_velocity * wheel_base_ / 2.0);
        
        double left_rpm = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
        double right_rpm = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;
        
        left_rpm = std::max(std::min(left_rpm, max_rpm_), -max_rpm_);
        right_rpm = std::max(std::min(right_rpm, max_rpm_), -max_rpm_);
        
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
