#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Driver: BTS7960 (RPWM/LPWM), điều khiển qua TIM3 =====
 * Trái  : RPWM=PA6 (TIM3_CH1), LPWM=PA7 (TIM3_CH2)
 * Phải  : RPWM=PB0 (TIM3_CH3), LPWM=PB1 (TIM3_CH4)
 * R_EN/L_EN của cả 2 board nối cứng lên 5V (luôn bật), không điều khiển từ STM32.
 *
 * Encoder JGB37-520 đấu THẲNG vào STM32 (không qua BTS7960):
 * Trái  : A=PA0, B=PA1  -> TIM2 encoder mode (16-bit)
 * Phải  : A=PB6, B=PB7  -> TIM4 encoder mode (16-bit)
 *         (F103 không có timer 32-bit nào -> CẢ 2 bên đều cần cộng dồn tràn
 *          số trong code, khác F411 chỉ cần cộng dồn bên phải vì TIM2 32-bit)
 */

/**
 * @brief  Khởi tạo motor driver: start PWM (duty=0) + start encoder timers.
 * @note   Gọi 1 lần sau MX_TIM2_Init()/MX_TIM3_Init()/MX_TIM4_Init() trong main.c.
 */
HAL_StatusTypeDef DRV_Motor_Init(void);

/**
 * @brief  Đặt tốc độ cho 2 bánh (điều khiển RPWM/LPWM của BTS7960).
 * @param  left   Tốc độ bánh trái:  -100 (lùi full) .. 0 .. 100 (tiến full)
 * @param  right  Tốc độ bánh phải: -100 (lùi full) .. 0 .. 100 (tiến full)
 * @note   Nếu bánh chạy ngược chiều mong muốn, đảo dấu ở đây (không cần tháo dây).
 */
HAL_StatusTypeDef DRV_Motor_SetSpeed(int8_t left, int8_t right);

/**
 * @brief  Đọc tổng xung encoder tích lũy từ 2 bánh.
 * @param  left   [out] Xung encoder bánh trái  (int32, tích lũy)
 * @param  right  [out] Xung encoder bánh phải (int32, tích lũy)
 */
HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right);

/**
 * @brief  Reset encoder về 0 (cả bộ đếm phần cứng lẫn biến cộng dồn phần mềm).
 */
HAL_StatusTypeDef DRV_Motor_ResetEncoder(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H__ */
