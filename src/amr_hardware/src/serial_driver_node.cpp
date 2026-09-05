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

        this->declare_parameter("serial_port",    "/dev/stm32");
        this->declare_parameter("baud_rate",       115200);
        this->declare_parameter("wheel_radius",    0.10);
        this->declare_parameter("wheel_base",      0.21);
        this->declare_parameter("encoder_ppr",     11.0);
        this->declare_parameter("gear_ratio",      90.0);
        this->declare_parameter("publish_rate_hz", 20.0);
        // Bù lệch tâm servo lái (rad/s, cộng vào angular.z trước khi gửi
        // xuống firmware) — CẦN TUNE THỰC NGHIỆM. Servo lái vật lý không
        // canh giữa tuyệt đối ở steer_deg=0 nên xe đi thẳng bị lệch hướng;
        // sửa ở đây (phía Jetson) để không cần build lại firmware STM32.
        this->declare_parameter("steering_trim_angular_z", 0.0);
        // Dấu encoder (+1.0 hoặc -1.0) — nhân vào tick thô trước khi tính
        // odometry, để "tick tăng" luôn nghĩa là "bánh đó lăn tiến".
        // Firmware $ODO gửi tick THÔ đúng như timer đếm được, KHÔNG bù dấu
        // (firmware chỉ bù nội bộ cho vòng PID qua LEFT_ENCODER_SIGN).
        // Với wiring DRV8871 hiện tại (đo thực nghiệm 2026-09-05 trên Jetson:
        // lệnh tiến -> enc_l chạy ÂM, enc_r chạy DƯƠNG, độ lớn khớp nhau) thì
        // bánh trái cần -1.0. Nếu KHÔNG bù, d=(dl+dr)/2 triệt tiêu về ~0 và
        // /odom đứng yên dù xe chạy thật.
        // ⚠️ Quy ước dấu đổi theo mỗi lần đấu lại dây — luôn đo lại bằng cách
        // gửi lệnh tiến và xem dấu enc_l/enc_r trong $ODO, đừng giả định.
        this->declare_parameter("left_encoder_sign",  -1.0);
        this->declare_parameter("right_encoder_sign",  1.0);

        port_         = this->get_parameter("serial_port").as_string();
        baud_         = this->get_parameter("baud_rate").as_int();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_   = this->get_parameter("wheel_base").as_double();
        double ppr    = this->get_parameter("encoder_ppr").as_double();
        double gear   = this->get_parameter("gear_ratio").as_double();
        ticks_per_rev_ = ppr * gear;
        double hz     = this->get_parameter("publish_rate_hz").as_double();
        steering_trim_angular_z_ = this->get_parameter("steering_trim_angular_z").as_double();
        left_encoder_sign_  = this->get_parameter("left_encoder_sign").as_double();
        right_encoder_sign_ = this->get_parameter("right_encoder_sign").as_double();

        if (!driver_.open(port_, baud_)) {
            RCLCPP_ERROR(this->get_logger(),
                "Không mở được port %s — cắm STM32 và kiểm tra ls /dev/ttyACM*",
                port_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Serial OK: %s @ %d baud", port_.c_str(), baud_);
        }

        odom_pub_            = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        steering_angle_pub_  = this->create_publisher<std_msgs::msg::Float32>("/steering_angle", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialDriverNode::cmdVelCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto period = std::chrono::duration<double>(1.0 / hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SerialDriverNode::timerCallback, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "serial_driver_node khởi động xong.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg) {
        double angular_trimmed = msg.angular.z + steering_trim_angular_z_;
        if (!driver_.sendCmdVel(
                static_cast<float>(msg.linear.x),
                static_cast<float>(angular_trimmed))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Gửi cmd_vel thất bại — kiểm tra serial port");
        }
    }

    void timerCallback() {
        OdomData od;
        if (!driver_.readOdom(od)) return;

        std_msgs::msg::Float32 steer_msg;
        steer_msg.data = od.steer_deg;  // độ, có dấu — khớp giá trị STM32 gửi qua $ODO
        steering_angle_pub_->publish(steer_msg);

        // Bù dấu encoder NGAY khi đọc — từ dòng này trở xuống, quy ước thống
        // nhất là "tick tăng == bánh đó lăn tiến", bất kể dây đấu thế nào.
        const double left_ticks  = od.left_ticks  * left_encoder_sign_;
        const double right_ticks = od.right_ticks * right_encoder_sign_;

        // Bộ đếm tick trên STM32 KHÔNG tự reset khi node ROS khởi động lại
        // -> lần đọc đầu tiên có thể mang giá trị tích lũy từ những lần chạy
        // trước. Lấy giá trị đọc đầu tiên làm baseline thay vì giả định 0,
        // tránh nhảy vọt vị trí giả tạo ngay khi node khởi động.
        if (!got_first_odom_) {
            prev_left_      = left_ticks;
            prev_right_     = right_ticks;
            got_first_odom_ = true;
            last_time_      = this->now();
            return;
        }

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 1.0) return;

        double m_per_tick = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;
        double dl = (left_ticks  - prev_left_)  * m_per_tick;
        double dr = (right_ticks - prev_right_) * m_per_tick;
        prev_left_  = left_ticks;
        prev_right_ = right_ticks;

        double d      = (dl + dr) / 2.0;
        double dtheta = (dr - dl) / wheel_base_;

        x_     += d * std::cos(theta_ + dtheta / 2.0);
        y_     += d * std::sin(theta_ + dtheta / 2.0);
        theta_ += dtheta;

        double vx  = d / dt;
        double vth = dtheta / dt;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp            = now;
        odom.header.frame_id         = "odom";
        odom.child_frame_id          = "base_link";
        odom.pose.pose.position.x    = x_;
        odom.pose.pose.position.y    = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x    = vx;
        odom.twist.twist.angular.z   = vth;
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp            = now;
        tf.header.frame_id         = "odom";
        tf.child_frame_id          = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation      = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    SerialDriver driver_;
    std::string  port_;
    int          baud_;

    double  wheel_radius_  = 0.10;
    double  wheel_base_    = 0.21;
    double  ticks_per_rev_ = 990.0;
    double  steering_trim_angular_z_ = 0.0;
    double  left_encoder_sign_  = -1.0;
    double  right_encoder_sign_ =  1.0;

    double  x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    // Tick đã bù dấu (double, giữ nguyên giá trị nguyên của tick) — không
    // dùng int32_t nữa vì tick thô được nhân với hệ số dấu kiểu double.
    double  prev_left_ = 0.0, prev_right_ = 0.0;
    bool    got_first_odom_ = false;
    rclcpp::Time last_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
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
