#include "amr_control/ackermann_kinematics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>

namespace amr_control {

class AckermannControllerNode : public rclcpp::Node {
public:
    AckermannControllerNode() : Node("ackermann_controller_node") {
        // ─── Parameters ─────────────────────────────────────────
        this->declare_parameter("wheelbase_m",      0.21);
        this->declare_parameter("track_width_m",    0.217);
        // Góc lái tối đa HTS-20H: kiểm tra spec servo
        this->declare_parameter("max_steering_deg", 30.0);

        double L     = this->get_parameter("wheelbase_m").as_double();
        double W     = this->get_parameter("track_width_m").as_double();
        double max_d = this->get_parameter("max_steering_deg").as_double();
        double max_r = max_d * M_PI / 180.0;

        kinematics_ = std::make_unique<AckermannKinematics>(L, W, max_r);

        RCLCPP_INFO(this->get_logger(),
            "Ackermann controller: L=%.3fm W=%.3fm max_steer=%.1fdeg", L, W, max_d);

        // ─── Subscribe /cmd_vel từ Nav2 hoặc teleop ─────────────
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&AckermannControllerNode::cmdVelCallback, this, std::placeholders::_1));

        // ─── Publish góc lái và tốc độ từng bánh ────────────────
        steering_pub_     = this->create_publisher<std_msgs::msg::Float32>("/steering_angle", 10);
        left_speed_pub_   = this->create_publisher<std_msgs::msg::Float32>("/left_wheel_speed", 10);
        right_speed_pub_  = this->create_publisher<std_msgs::msg::Float32>("/right_wheel_speed", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg) {
        auto cmd = kinematics_->compute(msg.linear.x, msg.angular.z);

        // Publish góc lái (độ) để serial_driver_node gửi sang STM32
        std_msgs::msg::Float32 steer_msg;
        steer_msg.data = cmd.steering_angle_rad * 180.0f / static_cast<float>(M_PI);
        steering_pub_->publish(steer_msg);

        // Publish tốc độ từng bánh (m/s)
        std_msgs::msg::Float32 lv, rv;
        lv.data = cmd.left_speed_mps;
        rv.data = cmd.right_speed_mps;
        left_speed_pub_->publish(lv);
        right_speed_pub_->publish(rv);
    }

    std::unique_ptr<AckermannKinematics> kinematics_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub_;
};

}  // namespace amr_control

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<amr_control::AckermannControllerNode>());
    rclcpp::shutdown();
    return 0;
}
