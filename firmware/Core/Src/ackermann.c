#include "ackermann.h"

/* Hệ số quy đổi vận tốc góc (rad/s) -> góc lái (độ).
 * Mô hình bicycle: với bán kính cua R, góc lái delta = atan(L / R),
 * và angular_z = linear_x / R  =>  delta phụ thuộc cả linear_x.
 * Để ĐƠN GIẢN và ổn định ở tốc độ thấp, ta quy đổi tuyến tính:
 *   steer_deg = angular_z * K_ANGULAR_TO_DEG
 * rồi clamp về +-ACK_MAX_STEER_DEG.
 * CẦN TUNE thực nghiệm: lái xe vòng tròn, so góc servo với bán kính thực. */
#define K_ANGULAR_TO_DEG    30.0f

void CALC_Ackermann(float linear_x, float angular_z,
                    int8_t *speed_l, int8_t *speed_r,
                    float *steer_deg)
{
    /* ---- 1. Vận tốc thẳng -> tốc độ motor (-100..100) ----
     * Tỉ lệ tuyến tính theo vận tốc tối đa, sau đó clamp. */
    float speed_f = (linear_x / ACK_MAX_SPEED_MS) * 100.0f;
    if (speed_f >  100.0f) speed_f =  100.0f;
    if (speed_f < -100.0f) speed_f = -100.0f;

    /* Mô hình đơn giản hóa: 2 bánh sau cùng tốc độ, chưa bù vi sai khi cua */
    int8_t spd = (int8_t)speed_f;
    *speed_l = spd;
    *speed_r = spd;

    /* ---- 2. Vận tốc góc -> góc lái servo (độ), clamp +-ACK_MAX_STEER_DEG ---- */
    float angle = angular_z * K_ANGULAR_TO_DEG;
    if (angle >  ACK_MAX_STEER_DEG) angle =  ACK_MAX_STEER_DEG;
    if (angle < -ACK_MAX_STEER_DEG) angle = -ACK_MAX_STEER_DEG;
    *steer_deg = angle;
}
