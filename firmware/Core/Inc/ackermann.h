#ifndef __ACKERMANN_H__
#define __ACKERMANN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===== Thông số hình học robot Ackermann =====
 * Đo trên xe thực tế (xem CLAUDE.md). Điều chỉnh nếu cơ khí thay đổi. */
#define ACK_WHEELBASE_M         0.210f  /* Khoảng cách trục trước-sau (m) */
#define ACK_WHEEL_RADIUS_M      0.050f  /* Bán kính bánh xe (m) */
#define ACK_MAX_SPEED_MS        0.5f    /* Vận tốc thẳng ứng với speed = 100 (m/s) */
#define ACK_MAX_STEER_DEG       30.0f   /* Góc lái tối đa cho phép (độ) */

/**
 * @brief  Chuyển cmd_vel (Twist) -> lệnh phần cứng cho xe Ackermann.
 * @param  linear_x   Vận tốc thẳng mong muốn (m/s); dương = tiến
 * @param  angular_z  Vận tốc góc mong muốn (rad/s); dương = quay trái
 * @param  speed_l    [out] Tốc độ bánh trái  (-100..100)
 * @param  speed_r    [out] Tốc độ bánh phải (-100..100)
 * @param  steer_deg  [out] Góc lái servo (độ); dương = phải, âm = trái
 *
 * @note   Mô hình ĐƠN GIẢN HÓA: 2 bánh sau cùng tốc độ (chưa bù vi sai
 *         tốc độ trái/phải khi vào cua). Chỉ servo trước thực hiện lái.
 *         Đủ dùng ở tốc độ thấp; nâng cấp bù vi sai sau nếu cần.
 */
void CALC_Ackermann(float linear_x, float angular_z,
                    int8_t *speed_l, int8_t *speed_r,
                    float *steer_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ACKERMANN_H__ */
