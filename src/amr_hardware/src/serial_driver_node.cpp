#include "amr_hardware/serial_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace amr_hardware {

class SerialDriverNode : public rclcpp::Node {
public:
    SerialDriverNode() : Node("serial_driver_node") {
        // ─── Parameters ─────────────────────────────────────────
        // CẦN ĐIỀU CHỈNH các thông số này theo phần cứng thực tế
        this->declare_parameter("serial_port",    "/dev/ttyUSB0");
        this->declare_parameter("baud_rate",      115200);
        this->declare_parameter("wheel_radius",   0.04);     // mét
        this->declare_parameter("wheel_base",     0.22);     // mét
        this->declare_parameter("encoder_ppr",    11);       // pulses per revolution (motor shaft)
        this->declare_parameter("gear_ratio",     90.0);     // tỉ số truyền JGB37-520
        this->declare_parameter("publish_rate_hz", 20.0);

        port_  = this->get_parameter("serial_port").as_string();
        baud_  = this->get_parameter("baud_rate").as_int();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_   = this->get_parameter("wheel_base").as_double();
        double ppr       = this->get_parameter("encoder_ppr").as_double();
        double gear      = this->get_parameter("gear_ratio").as_double();
        ticks_per_rev_   = ppr * gear;  // encoder ticks mỗi vòng bánh
        double hz        = this->get_parameter("publish_rate_hz").as_double();

        // ─── Serial ─────────────────────────────────────────────
        if (!driver_.open(port_, baud_)) {
            RCLCPP_ERROR(this->get_logger(),
                "Không mở được serial port: %s — kiểm tra /dev/ttyUSB*", port_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port mở thành công: %s @ %d", port_.c_str(), baud_);
        }

        // ─── Publishers / Subscribers ───────────────────────────
        odom_pub_     = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        cmd_vel_sub_  = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialDriverNode::cmdVelCallback, this, std::placeholders::_1));
        steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/steering_angle", 10,
            std::bind(&SerialDriverNode::steeringCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ─── Timer đọc encoder và publish odom ──────────────────
        auto period = std::chrono::duration<double>(1.0 / hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SerialDriverNode::timerCallback, this));

        last_time_ = this->now();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg) {
        // Chuyển linear.x (m/s) sang mm/s cho STM32
        float linear_mm_s = static_cast<float>(msg.linear.x * 1000.0);

        // angular.z → góc lái được tính bởi amr_control node
        // ở đây chỉ forward command, steering_angle_ được set qua topic riêng
        driver_.sendCommand(linear_mm_s, steering_angle_);
        // NOTE: STM32 nhận lệnh qua UART rồi điều khiển Hiwonder 4-Ch Encoder Motor Driver
        // qua I2C (không phải PWM trực tiếp). Firmware STM32 cần implement I2C master
        // để ghi speed command vào register của module Hiwonder.
        // Tra cứu I2C address và register map trong datasheet module Hiwonder.
    }

    void steeringCallback(const std_msgs::msg::Float32& msg) {
        steering_angle_ = msg.data;
    }

    void timerCallback() {
        OdomData odom_data;
        if (!driver_.readOdom(odom_data)) {
            return;
        }

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 1.0) return;

        // Tính delta distance từ encoder ticks
        double meters_per_tick = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;
        double dl = (odom_data.left_ticks  - prev_left_ticks_)  * meters_per_tick;
        double dr = (odom_data.right_ticks - prev_right_ticks_) * meters_per_tick;
        prev_left_ticks_  = odom_data.left_ticks;
        prev_right_ticks_ = odom_data.right_ticks;

        // Differential drive odometry (approximation cho Ackermann)
        double d_center = (dl + dr) / 2.0;
        double d_theta  = (dr - dl) / wheel_base_;

        x_   += d_center * std::cos(theta_ + d_theta / 2.0);
        y_   += d_center * std::sin(theta_ + d_theta / 2.0);
        theta_ += d_theta;

        double vx = d_center / dt;
        double vth = d_theta / dt;

        // ─── Publish Odometry ────────────────────────────────
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";
        odom_msg.pose.pose.position.x  = x_;
        odom_msg.pose.pose.position.y  = y_;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x  = vx;
        odom_msg.twist.twist.angular.z = vth;
        odom_pub_->publish(odom_msg);

        // ─── Broadcast TF odom → base_link ───────────────────
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id  = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // Serial
    SerialDriver driver_;
    std::string  port_;
    int          baud_;

    // Robot params
    double wheel_radius_   = 0.04;
    double wheel_base_     = 0.22;
    double ticks_per_rev_  = 990.0;  // 11 PPR × 90 gear ratio

    // State
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    int32_t prev_left_ticks_ = 0, prev_right_ticks_ = 0;
    float steering_angle_ = 0.0f;
    rclcpp::Time last_time_;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amr_hardware

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<amr_hardware::SerialDriverNode>());
    rclcpp::shutdown();
    return 0;
}
