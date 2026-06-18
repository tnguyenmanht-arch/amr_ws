#include "motor_driver.h"
#include "i2c.h"
#include <string.h>

HAL_StatusTypeDef DRV_Motor_Init(void)
{
    uint8_t buf[1];

    /* ---- Bước 1: Dừng motor TRƯỚC TIÊN ----
     * Driver có thể đang chạy với tốc độ lưu từ EEPROM của lần trước.
     * Phải dừng ngay để xe không lao đi khi vừa cấp nguồn. */
    DRV_Motor_SetSpeed(0, 0);
    HAL_Delay(300);

    /* ---- Bước 2: Cài loại motor JGB37-520R90 = 3 (CHỈ 1 byte) ----
     * BÀI HỌC PHẦN CỨNG: ghi quá 1 byte vào 0x14 làm hỏng cấu hình EEPROM.
     * KHÔNG return error nếu fail: driver có thể từ chối ghi config khi
     * đang ở trạng thái chạy, nhưng SetSpeed vẫn hoạt động bình thường. */
    buf[0] = MOTOR_TYPE_JGB37_520R90;
    (void)HAL_I2C_Mem_Write(&hi2c1, MOTOR_DRV_I2C_ADDR,
                            REG_MOTOR_TYPE, I2C_MEMADD_SIZE_8BIT,
                            buf, 1, MOTOR_I2C_TIMEOUT_MS);
    HAL_Delay(200); /* Chờ driver lưu cấu hình vào EEPROM nội */

    /* ---- Bước 3: Chiều encoder = 0 (không đảo) (CHỈ 1 byte) ----
     * Tương tự 0x14: chỉ 1 byte, bỏ qua lỗi nếu có. */
    buf[0] = 0;
    (void)HAL_I2C_Mem_Write(&hi2c1, MOTOR_DRV_I2C_ADDR,
                            REG_ENCODER_POLARITY, I2C_MEMADD_SIZE_8BIT,
                            buf, 1, MOTOR_I2C_TIMEOUT_MS);
    HAL_Delay(200);

    /* ---- Bước 4: Reset encoder về 0 ---- */
    return DRV_Motor_ResetEncoder();
}

HAL_StatusTypeDef DRV_Motor_SetSpeed(int8_t left, int8_t right)
{
    /* Kênh 1 = bánh trái, kênh 2 = bánh phải, kênh 3-4 không dùng
     * Ép kiểu int8_t → uint8_t: biểu diễn bù hai giữ nguyên bit pattern */
    int8_t speeds[MOTOR_NUM_CHANNELS] = { left, right, 0, 0 };

    return HAL_I2C_Mem_Write(&hi2c1, MOTOR_DRV_I2C_ADDR,
                             REG_FIXED_SPEED, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)speeds, MOTOR_NUM_CHANNELS,
                             MOTOR_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right)
{
    /* Board trả về 4 kênh × 4 byte = 16 byte, little-endian
     * Kênh 1 (byte 0-3) = trái, kênh 2 (byte 4-7) = phải */
    uint8_t buf[MOTOR_NUM_CHANNELS * sizeof(int32_t)];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c1, MOTOR_DRV_I2C_ADDR,
                           REG_ENCODER_TOTAL, I2C_MEMADD_SIZE_8BIT,
                           buf, sizeof(buf), MOTOR_I2C_TIMEOUT_MS);
    if (ret != HAL_OK) return ret;

    memcpy(left,  &buf[0], sizeof(int32_t));
    memcpy(right, &buf[4], sizeof(int32_t));
    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_ResetEncoder(void)
{
    /* Ghi 16 byte 0 để reset tất cả encoder về 0 */
    uint8_t zeros[MOTOR_NUM_CHANNELS * sizeof(int32_t)];
    memset(zeros, 0, sizeof(zeros));

    return HAL_I2C_Mem_Write(&hi2c1, MOTOR_DRV_I2C_ADDR,
                             REG_ENCODER_TOTAL, I2C_MEMADD_SIZE_8BIT,
                             zeros, sizeof(zeros), MOTOR_I2C_TIMEOUT_MS);
}
