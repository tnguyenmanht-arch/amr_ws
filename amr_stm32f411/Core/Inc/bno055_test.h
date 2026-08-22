#ifndef __BNO055_TEST_H__
#define __BNO055_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Test độc lập BNO055 (module GY-BNO055 clone) qua I2C1 =====
 * CHỈ để xác nhận module sống + đọc được số liệu trước khi quyết định tích
 * hợp vào ackermann.c/jetson_comm.c. KHÔNG dùng cho production.
 * I2C1: SCL=PB8, SDA=PB9, Standard mode 100kHz, polling (không NVIC). */

#define BNO055_ADDR_A          (0x28U << 1)  /* ADD nối GND/thả nổi -> 0x28 */
#define BNO055_ADDR_B          (0x29U << 1)  /* ADD nối VDD -> 0x29 */
#define BNO055_CHIP_ID_REG     0x00U
#define BNO055_CHIP_ID_VAL     0xA0U
#define BNO055_OPR_MODE_REG    0x3DU
#define BNO055_OPR_MODE_NDOF   0x0CU
#define BNO055_EUL_HEADING_LSB_REG  0x1AU

typedef struct {
    uint8_t found;   /* 1 nếu ACK được ở 1 trong 2 địa chỉ, 0 nếu cả 2 đều NAK */
    uint8_t addr8;   /* địa chỉ 8-bit (đã shift) đã ACK; chỉ hợp lệ nếu found=1 */
    uint8_t chip_id; /* giá trị đọc từ thanh ghi CHIP_ID; chỉ hợp lệ nếu found=1 */
} BNO055_TestResult;

/**
 * @brief  Quét địa chỉ 0x28 rồi 0x29, đọc CHIP_ID nếu ACK được, chuyển sang
 *         NDOF mode nếu CHIP_ID đúng 0xA0.
 * @note   KHÔNG đọc thêm thanh ghi nào nếu cả 2 địa chỉ đều không ACK — module
 *         GY-BNO055 giá rẻ có rủi ro đã biết là mặc định ở mode UART thay vì
 *         I2C (jumper cấu hình ẩn trên board). Đây là kết quả cần báo lại.
 */
BNO055_TestResult BNO055_Test_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Đọc góc heading (độ) từ thanh ghi EUL_HEADING. Chỉ gọi khi
 *         BNO055_Test_Init() đã báo found=1.
 * @param  addr8       Địa chỉ 8-bit đã xác nhận từ BNO055_Test_Init().
 * @param  heading_deg [out] Góc heading, độ (0.0 .. 360.0 xấp xỉ).
 */
HAL_StatusTypeDef BNO055_Test_ReadHeading(I2C_HandleTypeDef *hi2c, uint8_t addr8, float *heading_deg);

#ifdef __cplusplus
}
#endif

#endif /* __BNO055_TEST_H__ */
