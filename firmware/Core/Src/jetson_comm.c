#include "jetson_comm.h"
#include "usart.h"
#include <string.h>

/* ===== Circular buffer nhận UART2 =====
 * HAL_UART_RxCpltCallback đẩy từng byte vào đây.
 * APP_Comm_Parse() tiêu thụ từ tail.
 * Cặp head/tail là volatile để đảm bảo an toàn giữa ISR và main loop. */
static uint8_t  rx_circ_buf[COMM_RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

/* Buffer 1 byte dùng với HAL_UART_Receive_IT */
static uint8_t rx_single;

/* ===== Máy trạng thái parser ===== */
typedef enum {
    ST_HDR1 = 0,
    ST_HDR2,
    ST_CMD,
    ST_LEN,
    ST_DATA,
    ST_CRC,
} ParseState_t;

static ParseState_t parse_state   = ST_HDR1;
static uint8_t      parse_cmd     = 0;
static uint8_t      parse_len     = 0;
static uint8_t      parse_buf[COMM_MAX_DATA_LEN];
static uint8_t      parse_idx     = 0;

static comm_cmd_vel_cb_t g_cmd_vel_cb = NULL;

/* ===== Nội bộ ===== */

static uint8_t calc_crc(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    /* CRC = XOR của CMD, LEN và toàn bộ byte data */
    uint8_t crc = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

static void handle_packet(uint8_t cmd, const uint8_t *data)
{
    if (cmd == CMD_SET_VEL && g_cmd_vel_cb != NULL) {
        float linear_x, angular_z;
        /* Giải mã little-endian an toàn qua memcpy (tránh unaligned access) */
        memcpy(&linear_x,  &data[0], sizeof(float));
        memcpy(&angular_z, &data[4], sizeof(float));
        g_cmd_vel_cb(linear_x, angular_z);
    }
}

/* ===== Public API ===== */

void APP_Comm_Init(comm_cmd_vel_cb_t callback)
{
    g_cmd_vel_cb = callback;
    parse_state  = ST_HDR1;
    rx_head      = 0;
    rx_tail      = 0;

    /* Kích hoạt nhận interrupt 1 byte đầu tiên.
     * HAL_UART_RxCpltCallback sẽ tự động tái kích hoạt sau mỗi byte. */
    HAL_UART_Receive_IT(&huart2, &rx_single, 1);
}

void APP_Comm_Parse(void)
{
    /* Tiêu thụ tất cả byte đang có trong circular buffer */
    while (rx_tail != rx_head) {
        uint8_t b = rx_circ_buf[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1u) % COMM_RX_BUF_SIZE);

        switch (parse_state) {
        case ST_HDR1:
            if (b == COMM_HDR1) parse_state = ST_HDR2;
            break;

        case ST_HDR2:
            /* Nếu không gặp HDR2, kiểm tra lại từ đầu */
            parse_state = (b == COMM_HDR2) ? ST_CMD : ST_HDR1;
            break;

        case ST_CMD:
            parse_cmd   = b;
            parse_state = ST_LEN;
            break;

        case ST_LEN:
            if (b == 0 || b > COMM_MAX_DATA_LEN) {
                /* Độ dài không hợp lệ → bỏ frame, đợi frame mới */
                parse_state = ST_HDR1;
            } else {
                parse_len   = b;
                parse_idx   = 0;
                parse_state = ST_DATA;
            }
            break;

        case ST_DATA:
            parse_buf[parse_idx++] = b;
            if (parse_idx >= parse_len) parse_state = ST_CRC;
            break;

        case ST_CRC:
            /* Kiểm tra CRC; nếu sai, frame bị bỏ qua (không có retry) */
            if (b == calc_crc(parse_cmd, parse_len, parse_buf)) {
                handle_packet(parse_cmd, parse_buf);
            }
            parse_state = ST_HDR1;
            break;

        default:
            parse_state = ST_HDR1;
            break;
        }
    }
}

HAL_StatusTypeDef APP_Comm_SendOdom(int32_t enc_left, int32_t enc_right,
                                    uint16_t steering_pos)
{
    /* ---- Đóng gói frame ODOM ----
     * [0xAA][0x55][0x02][10][enc_l(4B)][enc_r(4B)][steer(2B)][CRC]
     * Tổng 15 byte */
    uint8_t data[ODOM_DATA_LEN];
    memcpy(&data[0], &enc_left,     sizeof(int32_t));
    memcpy(&data[4], &enc_right,    sizeof(int32_t));
    memcpy(&data[8], &steering_pos, sizeof(uint16_t));

    uint8_t crc = calc_crc(CMD_ODOM, ODOM_DATA_LEN, data);

    uint8_t packet[4u + ODOM_DATA_LEN + 1u];
    packet[0] = COMM_HDR1;
    packet[1] = COMM_HDR2;
    packet[2] = CMD_ODOM;
    packet[3] = ODOM_DATA_LEN;
    memcpy(&packet[4], data, ODOM_DATA_LEN);
    packet[4u + ODOM_DATA_LEN] = crc;

    /* Blocking transmit; thời gian tối đa 100ms (<<1 frame ở 115200 baud) */
    return HAL_UART_Transmit(&huart2, packet, sizeof(packet), 100);
}

/* ===== HAL Callback (override weak symbol) =====
 * Được gọi từ USART2_IRQHandler → HAL_UART_IRQHandler trong stm32f4xx_it.c.
 * Không cần chỉnh sửa stm32f4xx_it.c vì hàm này override __weak definition. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* Đẩy byte nhận được vào circular buffer nếu chưa đầy */
        uint8_t next_head = (uint8_t)((rx_head + 1u) % COMM_RX_BUF_SIZE);
        if (next_head != rx_tail) {
            rx_circ_buf[rx_head] = rx_single;
            rx_head = next_head;
        }
        /* Tái kích hoạt nhận byte tiếp theo */
        HAL_UART_Receive_IT(&huart2, &rx_single, 1);
    }
}
