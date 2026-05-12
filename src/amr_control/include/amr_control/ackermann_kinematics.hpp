#pragma once

#include <cmath>

namespace amr_control {

// Tính toán Ackermann steering geometry
// Tài liệu: https://en.wikipedia.org/wiki/Ackermann_steering_geometry
//
// Robot configuration:
//   - 2 bánh sau dẫn động (rear-wheel drive)
//   - 2 bánh trước lái (front-wheel steering via HTS-20H)
//   - wheelbase (L): khoảng cách trục trước-sau
//   - track_width (W): khoảng cách giữa 2 bánh (cùng trục)

struct AckermannCommand {
    float steering_angle_rad;   // góc lái trung bình (rad)
    float inner_wheel_angle_rad;  // bánh trong (góc lớn hơn)
    float outer_wheel_angle_rad;  // bánh ngoài (góc nhỏ hơn)
    float left_speed_mps;
    float right_speed_mps;
};

class AckermannKinematics {
public:
    // wheelbase_m: khoảng cách trục (mét)
    // track_width_m: khoảng cách 2 bánh sau (mét)
    // max_steering_rad: giới hạn góc lái (rad)
    AckermannKinematics(double wheelbase_m,
                        double track_width_m,
                        double max_steering_rad)
        : L_(wheelbase_m)
        , W_(track_width_m)
        , max_steer_(max_steering_rad)
    {}

    // Input:  linear_x (m/s), angular_z (rad/s) từ cmd_vel
    // Output: AckermannCommand
    AckermannCommand compute(double linear_x, double angular_z) const {
        AckermannCommand cmd{};

        if (std::abs(linear_x) < 1e-6 && std::abs(angular_z) < 1e-6) {
            return cmd;  // dừng
        }

        double steer = 0.0;
        double left_v = linear_x;
        double right_v = linear_x;

        if (std::abs(angular_z) > 1e-6) {
            // Bán kính quay R = v / ω
            double R = linear_x / angular_z;

            // Góc lái Ackermann: tan(δ) = L / R
            steer = std::atan2(L_, R);
            steer = clamp(steer, -max_steer_, max_steer_);

            // Tốc độ 2 bánh sau theo bán kính quay
            left_v  = angular_z * (R - W_ / 2.0);
            right_v = angular_z * (R + W_ / 2.0);
        }

        // Góc bánh trong/ngoài theo công thức Ackermann đầy đủ
        double inner = 0.0, outer = 0.0;
        if (std::abs(steer) > 1e-6) {
            double R_center = L_ / std::tan(steer);
            inner = std::atan2(L_, R_center - W_ / 2.0);
            outer = std::atan2(L_, R_center + W_ / 2.0);
            if (steer < 0) {
                inner = -inner;
                outer = -outer;
            }
        }

        cmd.steering_angle_rad   = static_cast<float>(steer);
        cmd.inner_wheel_angle_rad = static_cast<float>(inner);
        cmd.outer_wheel_angle_rad = static_cast<float>(outer);
        cmd.left_speed_mps       = static_cast<float>(left_v);
        cmd.right_speed_mps      = static_cast<float>(right_v);
        return cmd;
    }

private:
    double L_, W_, max_steer_;

    static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};

}  // namespace amr_control
