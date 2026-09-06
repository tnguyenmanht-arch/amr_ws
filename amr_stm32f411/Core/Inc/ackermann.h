#ifndef __ACKERMANN_H__
#define __ACKERMANN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===== Thông số hình học robot Ackermann =====
 * Đo trên xe thực tế (xem CLAUDE.md). Điều chỉnh nếu cơ khí thay đổi. */
#define ACK_WHEELBASE_M         0.210f  /* H: khoảng cách trục trước-sau (m) */
#define ACK_TRACK_WIDTH_M       0.217f  /* D: khoảng cách 2 bánh trái-phải (m) */
#define ACK_WHEEL_RADIUS_M      0.050f  /* Bán kính bánh xe (m) — bánh Ø100mm */
#define ACK_MAX_SPEED_MS        0.5f    /* Vận tốc thẳng ứng với speed = 100 (m/s) */
#define ACK_MAX_STEER_DEG       30.0f   /* Góc lái tối đa cho phép (độ) */

/* Ngưỡng coi như "đứng yên": dưới mức này không suy được góc lái từ (ω, v)
 * vì công thức atan(H·ω/v) chia cho 0. Xem xử lý trong ackermann.c. */
#define ACK_MIN_SPEED_MS        1e-4f

/* Bù lệch cơ khí servo (calibration thực nghiệm) — cộng vào steer_deg trước
 * khi clamp, để angular_z=0 cho ra đúng vị trí "thẳng" thực tế thay vì 0°
 * theo lý thuyết. CẦN TUNE lại nếu tháo/lắp lại servo. */
#define ACK_STEER_TRIM_DEG      1.5f

/**
 * @brief  Chuyển cmd_vel (Twist) -> lệnh phần cứng cho xe Ackermann.
 * @param  linear_x   Vận tốc thẳng mong muốn (m/s); dương = tiến
 * @param  angular_z  Vận tốc góc mong muốn (rad/s); dương = quay trái
 * @param  speed_l    [out] Tốc độ bánh trái  (-100..100)
 * @param  speed_r    [out] Tốc độ bánh phải (-100..100)
 * @param  steer_deg  [out] Góc lái servo (độ); dương = TRÁI, âm = PHẢI
 *                    (xác nhận thực nghiệm 2026-09-05: angular_z=+0.5 ->
 *                     steer=+16.5 -> bánh lái chỉ sang TRÁI. Khớp chuẩn ROS
 *                     REP-103 "angular_z dương = quay trái". Comment cũ ghi
 *                     "dương = phải" là SAI, đã sửa.)
 *
 * @note   Từ 2026-09-05 đã BÙ VI SAI 2 bánh sau khi vào cua (trước đó ép 2
 *         bánh cùng tốc độ, gây trượt lốp -> understeer gấp ~2 lần: lệnh lái
 *         30° lẽ ra cho vòng tròn Ø0.73m, thực đo Ø1.45m). Công thức lấy từ
 *         `reference/2 Motion Control Course/1. Kinematics Analysis.pdf`
 *         (Hiwonder) trang 6, đã tự dẫn lại kiểm chứng:
 *             V_L = V·(1 − D·tanθ/2H)  ,  V_R = V·(1 + D·tanθ/2H)
 *         Góc lái cũng đổi sang công thức đúng θ = atan(H·ω/v) — bản cũ dùng
 *         hằng số tuyến tính `angular_z * 30` bỏ quên vận tốc, chỉ đúng tại
 *         v ≈ 0.4 m/s.
 */
void CALC_Ackermann(float linear_x, float angular_z,
                    int8_t *speed_l, int8_t *speed_r,
                    float *steer_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ACKERMANN_H__ */
