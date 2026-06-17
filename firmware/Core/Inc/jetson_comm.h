#ifndef __JETSON_COMM_H__
#define __JETSON_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Giao thức UART2: STM32 ↔ Jetson Orin Nano =====
 *
 * Frame format:
 *   [0xAA][0x55][CMD][LEN][DATA[0]..DATA[LEN-1]][CRC]
 *
 *   HDR1 = 0xAA, HDR2 = 0x55  — 2 byte đồng bộ đầu frame
 *   CMD  = mã lệnh (1 byte)
 *   LEN  = số byte dữ liệu (1 byte, không tính CRC)
 *   DATA = dữ liệu, little-endian
 *   CRC  = XOR của (CMD XOR LEN XOR DATA[0] XOR ... XOR DATA[LEN-1])
 *
 * Baud rate: 115200, 8N1 (khớp với USART2 trong CubeMX) */

#define COMM_HDR1               0xAAU
#define COMM_HDR2               0x55U

/* ---- Mã lệnh ---- */
#define CMD_SET_VEL             0x01U   /* Jetson → STM32: linear_x(4B) + angular_z(4B) */
#define CMD_ODOM                0x02U   /* STM32 → Jetson: enc_left(4B)+enc_right(4B)+steering(2B) */

/* ---- Độ dài dữ liệu (LEN field) ---- */
#define SET_VEL_DATA_LEN        8U      /* float linear_x (4) + float angular_z (4) */
#define ODOM_DATA_LEN           10U     /* int32 enc_l (4) + int32 enc_r (4) + uint16 steer (2) */

/* ---- Kích thước buffer nhận nội bộ ---- */
#define COMM_RX_BUF_SIZE        64U
#define COMM_MAX_DATA_LEN       16U     /* Tối đa byte data trong 1 frame */

/**
 * @brief  Callback thông báo khi nhận được lệnh cmd_vel hợp lệ từ Jetson.
 * @param  linear_x   Vận tốc thẳng (m/s); dương = tiến
 * @param  angular_z  Vận tốc góc   (rad/s); dương = quay trái (theo ROS REP-103)
 */
typedef void (*comm_cmd_vel_cb_t)(float linear_x, float angular_z);

/**
 * @brief  Khởi tạo module giao tiếp Jetson.
 *         Kích hoạt UART2 nhận interrupt từng byte.
 * @param  callback  Hàm được gọi khi nhận đủ lệnh SET_VEL hợp lệ.
 *                   Truyền NULL nếu muốn xử lý thủ công.
 */
void APP_Comm_Init(comm_cmd_vel_cb_t callback);

/**
 * @brief  Xử lý dữ liệu trong buffer nhận, gọi trong vòng lặp while(1).
 *         Dùng máy trạng thái parse frame, kiểm tra CRC.
 */
void APP_Comm_Parse(void);

/**
 * @brief  Gửi gói odometry lên Jetson qua UART2.
 * @param  enc_left      Xung encoder bánh trái  (int32, tích lũy từ STM32)
 * @param  enc_right     Xung encoder bánh phải (int32, tích lũy từ STM32)
 * @param  steering_pos  Vị trí servo lái hiện tại (0 ~ 1000)
 * @return HAL_OK nếu truyền thành công
 */
HAL_StatusTypeDef APP_Comm_SendOdom(int32_t enc_left, int32_t enc_right,
                                    uint16_t steering_pos);

#ifdef __cplusplus
}
#endif

#endif /* __JETSON_COMM_H__ */
