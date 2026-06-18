#include "jetson_comm.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>      /* strtof */
#include <stdio.h>       /* snprintf */

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

/* ===== TX cấp thanh ghi (KHÔNG dùng HAL) =====
 * LÝ DO: HAL_UART_Transmit blocking giữ huart->Lock suốt thời gian gửi.
 * Nếu HAL_UART_RxCpltCallback (ISR) chạy đúng lúc đó, lời gọi
 * HAL_UART_Receive_IT bên trong nó sẽ thấy lock BUSY và KHÔNG re-arm
 * được interrupt RX -> reception chết vĩnh viễn (TX vẫn chạy).
 * Ghi thẳng vào USART2->DR né hoàn toàn lock này nên RX không bao giờ chết. */
static void uart2_tx_raw(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        while (!(USART2->SR & USART_SR_TXE)) { /* chờ thanh ghi gửi rỗng */ }
        USART2->DR = (uint16_t)(data[i] & 0xFFU);
    }
    while (!(USART2->SR & USART_SR_TC)) { /* chờ truyền xong byte cuối */ }
}

/* ===== Nội bộ ===== */

/* Đảm bảo RX interrupt luôn được kích hoạt; tự hồi phục nếu đã dừng
 * (do lỗi overrun hoặc lock race trước đó). Gọi an toàn bất cứ lúc nào. */
static void rx_ensure_armed(void)
{
    if (huart2.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&huart2, &rx_single, 1);
    }
}

/* Parse 1 dòng hoàn chỉnh: "$VEL,<linear>,<angular>"
 * Dùng strtof thay sscanf để KHÔNG phụ thuộc linker flag scanf-float. */
static void process_line(const char *line)
{
    if (strncmp(line, "$VEL,", 5) != 0) {
        return; /* Không phải lệnh vận tốc -> bỏ qua */
    }

    char *endptr;
    const char *p = line + 5;               /* trỏ tới sau "$VEL," */

    float linear = strtof(p, &endptr);
    if (endptr == p || *endptr != ',') {
        return;                             /* thiếu số thứ nhất hoặc dấu phẩy */
    }

    p = endptr + 1;                         /* bỏ qua dấu ',' */
    float angular = strtof(p, &endptr);
    if (endptr == p) {
        return;                             /* thiếu số thứ hai */
    }

    if (g_cmd_vel_cb != NULL) {
        g_cmd_vel_cb(linear, angular);
    }
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
    /* An toàn: nếu RX đã dừng vì bất kỳ lý do gì, khởi động lại */
    rx_ensure_armed();

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
    char tx_buf[COMM_TX_BUF_SIZE];
    int  n;

    /* "$ODO,<enc_l>,<enc_r>,<steer_deg>\n"; "%.1f" -> 1 chữ số thập phân.
     * (%.1f cần printf-float; $ODO hiển thị đúng "0.0" -> đã bật.) */
    n = snprintf(tx_buf, sizeof(tx_buf), "$ODO,%ld,%ld,%.1f\n",
                 (long)enc_l, (long)enc_r, (double)steer_deg);

    if (n > 0) {
        if (n > (int)sizeof(tx_buf)) n = (int)sizeof(tx_buf);
        uart2_tx_raw((const uint8_t *)tx_buf, (uint16_t)n);
    }
}

void APP_Comm_DebugPrint(const char *str)
{
    if (str != NULL) {
        uart2_tx_raw((const uint8_t *)str, (uint16_t)strlen(str));
    }
}

/* ===== HAL Callbacks (override weak symbol) ===== */

/* RXNE complete: gọi từ USART2_IRQHandler -> HAL_UART_IRQHandler. */
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

/* Lỗi UART (ORE/FE/NE/PE): xóa cờ và kích hoạt lại RX để không chết. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* Đọc SR rồi DR để xóa cờ overrun/framing trên dòng STM32F4 */
        volatile uint32_t tmp;
        tmp = huart->Instance->SR;
        tmp = huart->Instance->DR;
        (void)tmp;
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart2, &rx_single, 1);
    }
}
