#include "servo_buslinker.h"
#include "usart.h"

void DRV_Servo_Init(void)
{
    /* Di chuyển chậm về vị trí trung tâm khi khởi động (500ms) */
    DRV_Servo_SetPosition(SERVO_POS_CENTER, 500);
}

HAL_StatusTypeDef DRV_Servo_SetPosition(uint16_t position, uint16_t duration_ms)
{
    /* Giới hạn vị trí trong dải hợp lệ */
    if (position > SERVO_POS_MAX) position = SERVO_POS_MAX;

    /* ---- Đóng gói frame giao thức Hiwonder TTL Bus ----
     *
     * Cấu trúc frame SERVO_MOVE_TIME_WRITE (CMD=1):
     *  Byte:  0     1     2    3    4    5    6    7    8        9
     *  Data: [0x55][0x55][ID][LEN][CMD][posL][posH][tL][tH][CHECKSUM]
     *
     * LEN = 7: tính từ chính byte LEN đến hết CHECKSUM (LEN+CMD+4params+CHECKSUM)
     * CHECKSUM = ~(ID + LEN + CMD + posL + posH + tL + tH) & 0xFF
     */
    uint8_t id  = SERVO_ID;
    uint8_t cmd = SERVO_CMD_MOVE;
    uint8_t len = 7;
    uint8_t pos_lo = (uint8_t)(position & 0xFF);
    uint8_t pos_hi = (uint8_t)((position >> 8) & 0xFF);
    uint8_t t_lo   = (uint8_t)(duration_ms & 0xFF);
    uint8_t t_hi   = (uint8_t)((duration_ms >> 8) & 0xFF);

    uint8_t checksum = (uint8_t)(~((uint32_t)id + len + cmd +
                                   pos_lo + pos_hi + t_lo + t_hi) & 0xFF);

    uint8_t packet[10] = {
        SERVO_HDR, SERVO_HDR,
        id, len, cmd,
        pos_lo, pos_hi, t_lo, t_hi,
        checksum
    };

    return HAL_UART_Transmit(&huart1, packet, sizeof(packet), SERVO_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef DRV_Servo_SetAngle(float angle_deg)
{
    /* Giới hạn trong dải cơ khí cho phép */
    if (angle_deg >  SERVO_ANGLE_MAX_DEG) angle_deg =  SERVO_ANGLE_MAX_DEG;
    if (angle_deg < -SERVO_ANGLE_MAX_DEG) angle_deg = -SERVO_ANGLE_MAX_DEG;

    /* Chuyển đổi góc lái → vị trí servo:
     *   angle =  0°           → position = 500 (thẳng)
     *   angle = +ANGLE_MAX    → position = 1000 (lái hết phải)
     *   angle = -ANGLE_MAX    → position = 0    (lái hết trái)
     *
     * CẦN ĐIỀU CHỈNH: nếu servo lắp ngược, đổi dấu trước angle_deg.
     * SERVO_ANGLE_MAX_DEG mặc định = 120° (toàn dải servo); thu hẹp lại
     * theo góc lái thực tế của cơ cấu Ackermann trên xe. */
    int16_t pos = (int16_t)((float)SERVO_POS_CENTER +
                            (angle_deg / SERVO_ANGLE_MAX_DEG) * (float)SERVO_POS_CENTER);

    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;

    /* Thời gian di chuyển 100ms để tránh giật đột ngột */
    return DRV_Servo_SetPosition((uint16_t)pos, 100);
}
