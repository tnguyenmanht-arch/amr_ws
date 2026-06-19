#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Địa chỉ I2C =====
 * Hiwonder 4-Ch Encoder Motor Driver, địa chỉ mặc định 0x34 (7-bit)
 * HAL_I2C_Mem_Write/Read dùng định dạng 8-bit (địa chỉ đã shift trái 1 bit) */
#define MOTOR_DRV_ADDR_7BIT     0x34
#define MOTOR_DRV_I2C_ADDR      (MOTOR_DRV_ADDR_7BIT << 1)

/* ===== Register map =====
 * Tham khảo tài liệu Hiwonder 4-Ch Encoder Motor Driver
 * Cần xác nhận lại số byte mỗi register với phần cứng thực tế */
#define REG_MOTOR_TYPE          0x14    /* Loại motor: 3 = JGB37-520R90 */
#define REG_ENCODER_POLARITY    0x15    /* Chiều encoder: 0 = thuận */
#define REG_FIXED_SPEED         0x33    /* Tốc độ closed-loop: -100 ~ 100 */
#define REG_ENCODER_TOTAL       0x3C    /* Tổng xung encoder (int32/kênh, ghi 0 để reset) */

/* Loại motor JGB37-520 với gear ratio 90:1 */
#define MOTOR_TYPE_JGB37_520R90     3
#define MOTOR_NUM_CHANNELS          4   /* Board hỗ trợ 4 kênh, ta dùng 2 */

/* Timeout I2C (ms) */
#define MOTOR_I2C_TIMEOUT_MS        100

/* ===== Debug: trạng thái I2C của lần đọc encoder gần nhất =====
 * 0=HAL_OK, 1=HAL_ERROR, 2=HAL_BUSY, 3=HAL_TIMEOUT */
extern volatile uint8_t  g_last_enc_i2c_err;
extern volatile uint32_t g_last_enc_i2c_ec;    /* hi2c1.ErrorCode (HAL_I2C_ERROR_*) */
extern volatile char     g_last_enc_i2c_step;  /* 'B'=bit-bang; pha lỗi trong g_last_enc_i2c_err */

/* Probe chẩn đoán đọc voltage (0x00) — xem bus/read có hoạt động không */
extern volatile uint16_t g_probe_volt;         /* mV đọc được từ reg 0x00 */
extern volatile uint8_t  g_probe_rc;           /* 0=OK, else mã HAL khi đọc lỗi */
void DRV_Motor_ProbeVoltage(void);

/* Scan bus I2C: thiết bị nào ACK trong 0x08..0x77 */
extern volatile uint8_t  g_i2c_scan_found;     /* địa chỉ 7-bit đầu tiên ACK, 0xFF=none */
extern volatile uint8_t  g_i2c_scan_count;     /* số thiết bị ACK */
void DRV_Motor_ScanBus(void);

/**
 * @brief  Khởi tạo motor driver: cài motor type, encoder polarity, reset encoder.
 * @note   Gọi 1 lần sau MX_I2C1_Init() trong main.c.
 *         Kênh 1 = bánh trái, kênh 2 = bánh phải (điều chỉnh nếu đấu dây ngược).
 */
HAL_StatusTypeDef DRV_Motor_Init(void);

/**
 * @brief  Đặt tốc độ closed-loop cho 2 bánh.
 * @param  left   Tốc độ bánh trái:  -100 (lùi full) .. 0 .. 100 (tiến full)
 * @param  right  Tốc độ bánh phải: -100 (lùi full) .. 0 .. 100 (tiến full)
 */
HAL_StatusTypeDef DRV_Motor_SetSpeed(int8_t left, int8_t right);

/**
 * @brief  Đọc tổng xung encoder từ 2 bánh.
 * @param  left   [out] Xung encoder bánh trái  (int32, tích lũy)
 * @param  right  [out] Xung encoder bánh phải (int32, tích lũy)
 */
HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right);

/**
 * @brief  Reset encoder về 0 (ghi 0 vào REG_ENCODER_TOTAL).
 */
HAL_StatusTypeDef DRV_Motor_ResetEncoder(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H__ */
