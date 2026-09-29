#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Driver: YX-4055AM on-board (Hiwonder ROS Robot Controller), PWM qua TIM1 =====
 * Trái  = cổng M1: PE13 (TIM1_CH3), PE14 (TIM1_CH4)
 * Phải  = cổng M2: PE9  (TIM1_CH1), PE11 (TIM1_CH2)
 * Kênh nào là "tiến" cấu hình bằng LEFT/RIGHT_CH_FWD/REV trong motor_driver.c.
 * Cổng M3/M4 không dùng: chân PWM của chúng (PE5/PE6, PB8/PB9) để GPIO mức 0.
 *
 * Encoder JGB37-520 qua header motor 6 chân on-board (cấp 5V cho encoder):
 * Trái  : A=PA0,  B=PA1  -> TIM5 encoder mode (32-bit)
 * Phải  : A=PA15, B=PB3  -> TIM2 encoder mode (32-bit)
 * PA15/PB3 là chân JTAG -> SYS Debug trong .ioc PHẢI là Serial Wire.
 */

/**
 * @brief  Khởi tạo motor driver: start PWM (duty=0) + start encoder timers.
 * @note   Gọi 1 lần sau MX_TIM1_Init()/MX_TIM2_Init()/MX_TIM5_Init() trong main.c.
 */
HAL_StatusTypeDef DRV_Motor_Init(void);

/**
 * @brief  Đặt TỐC ĐỘ MỤC TIÊU cho 2 bánh (không phải duty trực tiếp nữa).
 * @param  left   Tốc độ mục tiêu bánh trái:  -100 (lùi full) .. 0 .. 100 (tiến full)
 * @param  right  Tốc độ mục tiêu bánh phải: -100 (lùi full) .. 0 .. 100 (tiến full)
 * @note   Từ 2026-09: closed-loop qua PID (xem motor_pid.h) — hàm này chỉ LƯU
 *         mục tiêu, PWM thực tế được DRV_Motor_UpdatePID() tính lại mỗi chu kỳ
 *         dựa trên encoder, để bù ma sát/tải không đều giữa các vòng quay
 *         (giảm giật cục so với ánh xạ thẳng % -> duty trước đây).
 *         Interface (-100..100) giữ nguyên nên jetson_comm.c/ackermann.c
 *         không cần sửa gì.
 * @note   Nếu bánh chạy ngược chiều mong muốn, đảo dấu ở đây (không cần tháo dây).
 */
HAL_StatusTypeDef DRV_Motor_SetSpeed(int8_t left, int8_t right);

/**
 * @brief  Chạy 1 bước vòng lặp PID tốc độ cho cả 2 bánh.
 * @note   Tự throttle bên trong theo chu kỳ cố định (xem PID_INTERVAL_MS
 *         trong motor_driver.c) — gọi hàm này ở MỌI vòng lặp while(1) trong
 *         main.c là an toàn và không tốn gì khi chưa tới chu kỳ.
 */
void DRV_Motor_UpdatePID(void);

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
