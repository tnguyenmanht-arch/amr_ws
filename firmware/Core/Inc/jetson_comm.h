#ifndef __JETSON_COMM_H__
#define __JETSON_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Giao thức UART2 (ASCII): STM32 <-> Jetson Orin Nano =====
 *
 * Jetson -> STM32 (lệnh vận tốc):
 *   "$VEL,<linear_x>,<angular_z>\n"
 *   Ví dụ: "$VEL,0.50,-0.30\n"   (linear_x = 0.5 m/s, angular_z = -0.3 rad/s)
 *
 * STM32 -> Jetson (odometry):
 *   "$ODO,<enc_left>,<enc_right>,<steer_angle>\n"
 *   Ví dụ: "$ODO,1234,-1230,15.5\n"
 *
 * Baud rate: 115200, 8N1 (khớp với USART2 trong CubeMX).
 *
 * LƯU Ý BUILD: parse/format dùng sscanf/snprintf với "%f" -> CẦN bật
 * float trong printf/scanf. Trong STM32CubeIDE:
 *   Project > Properties > C/C++ Build > Settings > MCU Settings
 *   -> tick "Use float with printf" và "Use float with scanf".
 */

/* ---- Kích thước buffer ---- */
#define COMM_RX_BUF_SIZE        128U    /* Circular buffer nhận từ ISR */
#define COMM_LINE_BUF_SIZE      128U    /* Buffer ghép 1 dòng đến khi gặp '\n' */
#define COMM_TX_BUF_SIZE        64U     /* Buffer định dạng chuỗi gửi đi */

/**
 * @brief  Callback khi nhận được lệnh vận tốc hợp lệ từ Jetson.
 * @param  linear   Vận tốc thẳng (m/s); dương = tiến
 * @param  angular  Vận tốc góc   (rad/s); dương = quay trái (ROS REP-103)
 */
typedef void (*comm_cmd_vel_cb_t)(float linear, float angular);

/**
 * @brief  Khởi tạo module giao tiếp Jetson, kích hoạt UART2 nhận interrupt.
 * @param  cb  Hàm gọi khi parse được 1 lệnh "$VEL" hợp lệ. NULL = bỏ qua.
 */
void APP_Comm_Init(comm_cmd_vel_cb_t cb);

/**
 * @brief  Xử lý dữ liệu trong buffer nhận; gọi liên tục trong vòng lặp while(1).
 *         Ghép byte thành dòng, khi gặp '\n' thì parse bằng sscanf.
 */
void APP_Comm_Parse(void);

/**
 * @brief  Gửi gói odometry lên Jetson qua UART2 (blocking).
 * @param  enc_l      Xung encoder bánh trái  (int32, tích lũy)
 * @param  enc_r      Xung encoder bánh phải (int32, tích lũy)
 * @param  steer_deg  Góc lái hiện tại (độ); dương = phải, âm = trái
 */
void APP_Comm_SendOdom(int32_t enc_l, int32_t enc_r, float steer_deg);

#ifdef __cplusplus
}
#endif

#endif /* __JETSON_COMM_H__ */
