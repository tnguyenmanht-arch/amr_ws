#include "jetson_comm.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* ===== Circular buffer nhận UART2 =====
 * HAL_UART_RxCpltCallback (ISR) đẩy từng byte vào đây.
 * APP_Comm_Parse() (main loop) tiêu thụ từ tail.
 * head/tail là volatile để an toàn giữa ISR và main loop. */
static uint8_t          rx_circ_buf[COMM_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* Buffer 1 byte dùng với HAL_UART_Receive_IT */
static uint8_t rx_single;

/* Buffer ghép từng byte thành 1 dòng ASCII đến khi gặp '\n' */
static char    line_buf[COMM_LINE_BUF_SIZE];
static uint16_t line_idx = 0;

static comm_cmd_vel_cb_t g_cmd_vel_cb = NULL;

/* ===== Nội bộ ===== */

/* Parse 1 dòng hoàn chỉnh: "$VEL,<linear>,<angular>" */
static void process_line(const char *line)
{
    float linear = 0.0f, angular = 0.0f;

    /* sscanf trả về số trường đọc được; cần đúng 2 trường float */
    if (sscanf(line, "$VEL,%f,%f", &linear, &angular) == 2) {
        if (g_cmd_vel_cb != NULL) {
            g_cmd_vel_cb(linear, angular);
        }
    }
    /* Dòng không khớp định dạng -> bỏ qua, không báo lỗi */
}

/* ===== Public API ===== */

void APP_Comm_Init(comm_cmd_vel_cb_t cb)
{
    g_cmd_vel_cb = cb;
    rx_head  = 0;
    rx_tail  = 0;
    line_idx = 0;

    /* Kích hoạt nhận interrupt byte đầu tiên;
     * HAL_UART_RxCpltCallback tự tái kích hoạt sau mỗi byte. */
    HAL_UART_Receive_IT(&huart2, &rx_single, 1);
}

void APP_Comm_Parse(void)
{
    /* Tiêu thụ tất cả byte hiện có trong circular buffer */
    while (rx_tail != rx_head) {
        uint8_t b = rx_circ_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % COMM_RX_BUF_SIZE);

        if (b == '\n') {
            /* Kết thúc dòng -> chốt chuỗi và parse */
            line_buf[line_idx] = '\0';
            process_line(line_buf);
            line_idx = 0;
        } else if (b == '\r') {
            /* Bỏ qua CR (trường hợp Jetson gửi CRLF) */
        } else {
            /* Tích lũy byte; nếu tràn buffer thì reset (dòng lỗi/quá dài) */
            if (line_idx < (COMM_LINE_BUF_SIZE - 1u)) {
                line_buf[line_idx++] = (char)b;
            } else {
                line_idx = 0;
            }
        }
    }
}

void APP_Comm_SendOdom(int32_t enc_l, int32_t enc_r, float steer_deg)
{
    char    tx_buf[COMM_TX_BUF_SIZE];
    int     n;

    /* Định dạng: "$ODO,<enc_l>,<enc_r>,<steer_deg>\n"
     * snprintf chống tràn buffer; "%.1f" -> 1 chữ số thập phân cho góc lái. */
    n = snprintf(tx_buf, sizeof(tx_buf), "$ODO,%ld,%ld,%.1f\n",
                 (long)enc_l, (long)enc_r, (double)steer_deg);

    if (n > 0) {
        if (n > (int)sizeof(tx_buf)) n = (int)sizeof(tx_buf); /* an toàn */
        /* Blocking transmit; 50ms >> thời gian gửi ~40 byte ở 115200 baud */
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, (uint16_t)n, 50);
    }
}

/* ===== HAL Callback (override weak symbol) =====
 * Gọi từ USART2_IRQHandler -> HAL_UART_IRQHandler trong stm32f4xx_it.c.
 * Override __weak definition nên không cần sửa stm32f4xx_it.c. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* Đẩy byte vào circular buffer nếu chưa đầy */
        uint16_t next_head = (uint16_t)((rx_head + 1u) % COMM_RX_BUF_SIZE);
        if (next_head != rx_tail) {
            rx_circ_buf[rx_head] = rx_single;
            rx_head = next_head;
        }
        /* Tái kích hoạt nhận byte tiếp theo */
        HAL_UART_Receive_IT(&huart2, &rx_single, 1);
    }
}
