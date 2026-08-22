#include "bno055_test.h"

BNO055_TestResult BNO055_Test_Init(I2C_HandleTypeDef *hi2c)
{
    BNO055_TestResult r = {0};

    /* Thử 0x28 trước, rồi 0x29 -- dùng HAL_I2C_IsDeviceReady (chỉ gửi địa chỉ,
     * chờ ACK) để quét mà không cần biết trước thanh ghi nào tồn tại. */
    if (HAL_I2C_IsDeviceReady(hi2c, BNO055_ADDR_A, 3, 100) == HAL_OK) {
        r.addr8 = BNO055_ADDR_A;
    } else if (HAL_I2C_IsDeviceReady(hi2c, BNO055_ADDR_B, 3, 100) == HAL_OK) {
        r.addr8 = BNO055_ADDR_B;
    } else {
        r.found = 0;
        return r;   /* Không ACK ở cả 2 địa chỉ -> dừng, KHÔNG đoán mò thêm */
    }
    r.found = 1;

    /* Đọc CHIP_ID -- lưu lại dù đúng hay sai, không tự phán đoán ở đây */
    {
        uint8_t reg = BNO055_CHIP_ID_REG;
        uint8_t chip_id = 0;
        if (HAL_I2C_Mem_Read(hi2c, r.addr8, reg, I2C_MEMADD_SIZE_8BIT,
                              &chip_id, 1, 100) == HAL_OK) {
            r.chip_id = chip_id;
        }
    }

    /* Chuyển NDOF mode -- chỉ làm nếu CHIP_ID đúng 0xA0, tránh ghi bừa vào
     * thanh ghi lạ nếu thực ra đang nói chuyện với thiết bị I2C khác. */
    if (r.chip_id == BNO055_CHIP_ID_VAL) {
        uint8_t mode_reg = BNO055_OPR_MODE_REG;
        uint8_t mode_val = BNO055_OPR_MODE_NDOF;
        HAL_I2C_Mem_Write(hi2c, r.addr8, mode_reg, I2C_MEMADD_SIZE_8BIT,
                           &mode_val, 1, 100);
        HAL_Delay(20);  /* Datasheet Bosch: đợi >=19-20ms sau khi đổi OPR_MODE */
    }

    return r;
}

HAL_StatusTypeDef BNO055_Test_ReadHeading(I2C_HandleTypeDef *hi2c, uint8_t addr8, float *heading_deg)
{
    uint8_t reg = BNO055_EUL_HEADING_LSB_REG;
    uint8_t buf[2] = {0};

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c, addr8, reg, I2C_MEMADD_SIZE_8BIT,
                                             buf, 2, 100);
    if (st != HAL_OK) {
        return st;
    }

    /* LSB tại offset thấp, MSB tại offset cao (little-endian, theo datasheet) */
    int16_t raw = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
    *heading_deg = (float)raw / 16.0f;  /* 1 LSB = 1/16 độ */
    return HAL_OK;
}
