#include "servo_buslinker.h"
#include "main.h"

/* ===== Bộ đệm half-duplex trên board (74LVC2G125, U14 — schematic V1.1 trang 3) =====
 *   Nhánh 1: PC6 (USART6_TX) --[1OE = PE7 SERVO_TX_EN]--> SERVO_SIGNAL
 *   Nhánh 2: SERVO_SIGNAL    --[2OE = PE8 SERVO_RX_EN]--> PC7 (USART6_RX)
 *   SERVO_SIGNAL kéo lên 5V qua R14 1k -> khi cả 2 nhánh tắt, dây vẫn ở mức
 *   cao = trạng thái nghỉ của UART.
 * Chân OE của 74LVC2G125 TÍCH CỰC MỨC THẤP (theo datasheet TI).
 * ⚠️ CẦN XÁC MINH trên board thật (bring-up): nếu servo không nhúc nhích dù
 * ID/nguồn đúng, thử đảo 2 macro dưới đây trước khi nghi phần cứng. */
#define SERVO_BUF_ON    GPIO_PIN_RESET
#define SERVO_BUF_OFF   GPIO_PIN_SET

/* Chế độ NGHE: tắt nhánh TX trước rồi mới bật nhánh RX (không bao giờ bật
 * cả 2 cùng lúc -> tránh tự nhận lại chính byte mình vừa gửi). */
static void servo_bus_rx_mode(void)
{
    HAL_GPIO_WritePin(SERVO_TX_EN_GPIO_Port, SERVO_TX_EN_Pin, SERVO_BUF_OFF);
    HAL_GPIO_WritePin(SERVO_RX_EN_GPIO_Port, SERVO_RX_EN_Pin, SERVO_BUF_ON);
}

/* Chế độ GỬI: tắt nhánh RX trước rồi mới bật nhánh TX. */
static void servo_bus_tx_mode(void)
{
    HAL_GPIO_WritePin(SERVO_RX_EN_GPIO_Port, SERVO_RX_EN_Pin, SERVO_BUF_OFF);
    HAL_GPIO_WritePin(SERVO_TX_EN_GPIO_Port, SERVO_TX_EN_Pin, SERVO_BUF_ON);
}

void DRV_Servo_Init(void)
{
    servo_bus_rx_mode();   /* Mặc định nghe, đường bus ở mức nghỉ */

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

    /* HAL_UART_Transmit chỉ trả về sau khi cờ TC bật (byte cuối đã ra hết
     * dây) -> chuyển về chế độ nghe ngay sau đó là an toàn, không cắt cụt
     * byte cuối. USART6 KHÔNG dùng Receive_IT nên không dính bẫy HAL lock
     * đã gặp ở UART Jetson. */
    servo_bus_tx_mode();
    HAL_StatusTypeDef st = HAL_UART_Transmit(&SERVO_HUART, packet, sizeof(packet),
                                             SERVO_TX_TIMEOUT_MS);
    servo_bus_rx_mode();
    return st;
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
