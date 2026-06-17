#ifndef __SERVO_BUSLINKER_H__
#define __SERVO_BUSLINKER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Cấu hình servo HTS-20H =====
 * Ket noi: STM32 USART1 TX(PA9)/RX(PA10) → Hiwonder TTL Bus Servo Debugging Board → HTS-20H */

/* ID servo, mặc định từ nhà sản xuất = 1 (cần xác nhận bằng phần mềm debug) */
#define SERVO_ID                1

/* Vùng vị trí servo: 0 ~ 1000 tương ứng 0° ~ 240° toàn bộ dải cơ khí */
#define SERVO_POS_MIN           0
#define SERVO_POS_MAX           1000
#define SERVO_POS_CENTER        500     /* Vị trí giữa = lái thẳng (120°) */

/* Góc cực đại cho phép so với vị trí trung tâm.
 * CẦN ĐIỀU CHỈNH theo giới hạn cơ khí thực tế của khung xe!
 * Vượt quá giới hạn này có thể làm kẹt bộ phận lái. */
#define SERVO_ANGLE_MAX_DEG     120.0f

/* ===== Giao thức Hiwonder TTL Bus Servo =====
 * Frame: [0x55][0x55][ID][LEN][CMD][params...][CHECKSUM]
 * LEN  = LEN(1) + CMD(1) + params(n) + CHECKSUM(1) = n + 3
 *        Ví dụ SERVO_MOVE_TIME_WRITE (4 params): LEN = 7
 * CHECKSUM = ~(ID + LEN + CMD + params...) & 0xFF */
#define SERVO_HDR               0x55
#define SERVO_CMD_MOVE          1       /* SERVO_MOVE_TIME_WRITE: di chuyển đến vị trí trong t ms */

/* Timeout truyền UART1 (ms) */
#define SERVO_TX_TIMEOUT_MS     50

/**
 * @brief  Khởi tạo servo về vị trí trung tâm (lái thẳng).
 * @note   Gọi sau MX_USART1_UART_Init(). Servo di chuyển chậm trong 500ms.
 */
void DRV_Servo_Init(void);

/**
 * @brief  Điều khiển servo đến vị trí tuyệt đối.
 * @param  position    Vị trí đích: 0 ~ 1000 (0° ~ 240°); trung tâm = 500
 * @param  duration_ms Thời gian di chuyển (ms); 0 = di chuyển nhanh nhất
 */
HAL_StatusTypeDef DRV_Servo_SetPosition(uint16_t position, uint16_t duration_ms);

/**
 * @brief  Điều khiển servo theo góc lái so với trung tâm.
 * @param  angle_deg  Góc lái (độ):  0.0 = thẳng, dương = phải, âm = trái
 *                    Phạm vi: -SERVO_ANGLE_MAX_DEG .. +SERVO_ANGLE_MAX_DEG
 * @note   Chiều dương/âm phụ thuộc vào chiều lắp servo trên xe.
 *         Nếu lái ngược, đổi dấu angle_deg khi gọi hàm này.
 */
HAL_StatusTypeDef DRV_Servo_SetAngle(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_BUSLINKER_H__ */
