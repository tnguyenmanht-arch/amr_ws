#include "motor_driver.h"
#include "i2c.h"
#include <string.h>

/* ============================================================================
 * Bit-bang I2C — OPEN-DRAIN thật sự (theo lý thuyết I2C chuẩn)
 * ----------------------------------------------------------------------------
 * PB8 = SCL, PB9 = SDA. Cấu hình GPIO_MODE_OUTPUT_OD + pull-up:
 *   - "Kéo LOW"  : ghi 0 vào ODR (BSRR reset) -> chân kéo xuống đất.
 *   - "Thả HIGH" : ghi 1 vào ODR (BSRR set)   -> open-drain NHẢ, pull-up kéo lên.
 *     (open-drain KHÔNG bao giờ đẩy cao chủ động -> không xung đột với slave)
 *   - Đọc mức bus: đọc IDR (luôn hợp lệ vì OD).
 * Không cần đổi chiều IN/OUT; không dùng push-pull ở bất kỳ bước nào.
 *
 * Trước khi cấu hình, HAL_I2C_DeInit(&hi2c1) để nhả hẳn AF I2C1 khỏi PB8/PB9.
 * ========================================================================== */

#define SCL_BIT     8u
#define SDA_BIT     9u

#define SCL_SET()   (GPIOB->BSRR = (1u << SCL_BIT))            /* nhả SCL (pull-up) */
#define SCL_L()     (GPIOB->BSRR = (1u << (SCL_BIT + 16u)))    /* kéo SCL thấp      */
#define SCL_READ()  ((GPIOB->IDR >> SCL_BIT) & 1u)
#define SDA_H()     (GPIOB->BSRR = (1u << SDA_BIT))
#define SDA_L()     (GPIOB->BSRR = (1u << (SDA_BIT + 16u)))
#define SDA_READ()  ((GPIOB->IDR >> SDA_BIT) & 1u)

/* ===== Trạng thái lỗi I2C đọc encoder (giữ lại; ENCERR print có thể bật ở main.c) =====
 * g_last_enc_i2c_err: 0=OK, 1=NAK addr+W, 2=NAK reg, 3=NAK addr+R */
volatile uint8_t  g_last_enc_i2c_err  = 0;
volatile uint32_t g_last_enc_i2c_ec   = 0;
volatile char     g_last_enc_i2c_step = 'B';

/* ===== Delay micro giây (DWT) ===== */
static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static void DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}
#define T_HALF   25u   /* chậm để dư thời gian sườn lên (chống rise-time pull-up yếu) */

/* Nhả SCL rồi CHỜ nó thực sự lên cao — xử lý clock-stretching:
 * slave soft-I2C có thể giữ SCL thấp để câu giờ. Bắt buộc với SCL open-drain. */
static void scl_high(void)
{
    SCL_SET();                 /* nhả SCL (pull-up kéo lên) */
    uint16_t t = 0;
    while (!SCL_READ()) {       /* chờ slave nhả SCL */
        if (++t > 2000u) break; /* timeout an toàn, tránh treo */
    }
}

/* ===== Nguyên thủy bit-bang (open-drain) ===== */
static void i2c_start(void)
{
    SDA_H(); scl_high(); DelayUs(T_HALF);
    SDA_L(); DelayUs(T_HALF);
    SCL_L(); DelayUs(T_HALF);
}
static void i2c_stop(void)
{
    SCL_L(); SDA_L(); DelayUs(T_HALF);
    scl_high(); DelayUs(T_HALF);
    SDA_H(); DelayUs(T_HALF);
}
/* 1 = NAK, 0 = ACK */
static uint8_t i2c_wait_ack(void)
{
    uint16_t t = 0;
    SDA_H();                  /* nhả SDA cho slave kéo xuống nếu ACK */
    DelayUs(T_HALF);
    scl_high();                  /* cạnh clock thứ 9 */
    DelayUs(T_HALF);
    while (SDA_READ()) {
        if (++t > 255) { SCL_L(); i2c_stop(); return 1; }
    }
    SCL_L();
    DelayUs(T_HALF);
    return 0;
}
static void i2c_send_byte(uint8_t txd)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (txd & 0x80u) SDA_H(); else SDA_L();   /* thả=1 / kéo=0 */
        txd <<= 1;
        DelayUs(T_HALF);
        scl_high(); DelayUs(T_HALF);
        SCL_L(); DelayUs(T_HALF);
    }
    SDA_H();   /* nhả SDA trước pha ACK */
}
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t recv = 0;
    SDA_H();                  /* nhả SDA để slave đẩy data */
    for (uint8_t i = 0; i < 8; i++) {
        recv <<= 1;
        scl_high(); DelayUs(T_HALF);
        if (SDA_READ()) recv++;
        SCL_L(); DelayUs(T_HALF);
    }
    /* Master phát ACK (kéo SDA thấp) hoặc NACK (thả) */
    if (ack) SDA_L(); else SDA_H();
    DelayUs(T_HALF);
    scl_high(); DelayUs(T_HALF);
    SCL_L(); DelayUs(T_HALF);
    SDA_H();
    return recv;
}

/* 0=OK, 1=NAK addr+W, 2=NAK reg/data, 3=NAK addr+R
 * KHÔNG tắt ngắt: GPIO slew-rate LOW mới là thứ làm bit-bang tin cậy; I2C cho
 * phép master kéo dài clock khi ISR chen vào -> để ngắt chạy bình thường (UART RX). */
static uint8_t soft_i2c_write_len(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_send_byte((uint8_t)((MOTOR_DRV_ADDR_7BIT << 1) | 0u));
    if (i2c_wait_ack()) { i2c_stop(); return 1; }
    i2c_send_byte(reg);
    if (i2c_wait_ack()) { i2c_stop(); return 2; }
    for (uint8_t i = 0; i < len; i++) {
        i2c_send_byte(buf[i]);
        if (i2c_wait_ack()) { i2c_stop(); return 2; }
    }
    i2c_stop();
    return 0;
}
static uint8_t soft_i2c_read_len(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_send_byte((uint8_t)((MOTOR_DRV_ADDR_7BIT << 1) | 0u));
    if (i2c_wait_ack()) { i2c_stop(); return 1; }
    i2c_send_byte(reg);
    if (i2c_wait_ack()) { i2c_stop(); return 2; }
    i2c_start();                                              /* repeated-START */
    i2c_send_byte((uint8_t)((MOTOR_DRV_ADDR_7BIT << 1) | 1u));
    if (i2c_wait_ack()) { i2c_stop(); return 3; }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2c_read_byte((i != (uint8_t)(len - 1)));
    }
    i2c_stop();
    return 0;
}

/* DeInit I2C1 (nhả AF) + cấu hình PB8/PB9 = OPEN-DRAIN + pull-up */
static void soft_i2c_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    HAL_I2C_DeInit(&hi2c1);              /* nhả hẳn AF I2C1 khỏi PB8/PB9 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;       /* OPEN-DRAIN (không push-pull) */
    g.Pull  = GPIO_PULLUP;               /* pull-up nội (board cũng có pull-up ngoài) */
    g.Speed = GPIO_SPEED_FREQ_LOW;       /* sườn xuống chậm/sạch hơn, chống ringing */
    HAL_GPIO_Init(GPIOB, &g);

    scl_high(); SDA_H();                    /* bus idle: cả 2 nhả cao */
    dwt_delay_init();
}

/* ============================================================================
 * API công khai
 * ========================================================================== */
HAL_StatusTypeDef DRV_Motor_Init(void)
{
    uint8_t b;
    soft_i2c_gpio_init();

    DRV_Motor_SetSpeed(0, 0);
    HAL_Delay(300);

    b = MOTOR_TYPE_JGB37_520R90;
    (void)soft_i2c_write_len(REG_MOTOR_TYPE, &b, 1);
    HAL_Delay(200);

    b = 0;
    (void)soft_i2c_write_len(REG_ENCODER_POLARITY, &b, 1);
    HAL_Delay(200);

    return DRV_Motor_ResetEncoder();
}

HAL_StatusTypeDef DRV_Motor_SetSpeed(int8_t left, int8_t right)
{
    int8_t speeds[MOTOR_NUM_CHANNELS] = { left, right, 0, 0 };
    return soft_i2c_write_len(REG_FIXED_SPEED, (uint8_t *)speeds,
                              MOTOR_NUM_CHANNELS) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right)
{
    uint8_t buf[MOTOR_NUM_CHANNELS * sizeof(int32_t)];
    uint8_t r = soft_i2c_read_len(REG_ENCODER_TOTAL, buf, sizeof(buf));

    g_last_enc_i2c_err = r;
    if (r) { g_last_enc_i2c_step = 'B'; g_last_enc_i2c_ec = 0; return HAL_ERROR; }

    memcpy(left,  &buf[0], sizeof(int32_t));
    memcpy(right, &buf[4], sizeof(int32_t));
    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_ResetEncoder(void)
{
    uint8_t zeros[MOTOR_NUM_CHANNELS * sizeof(int32_t)];
    memset(zeros, 0, sizeof(zeros));
    return soft_i2c_write_len(REG_ENCODER_TOTAL, zeros,
                              sizeof(zeros)) ? HAL_ERROR : HAL_OK;
}
