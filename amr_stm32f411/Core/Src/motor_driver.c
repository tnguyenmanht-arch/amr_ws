#include "motor_driver.h"
#include "main.h"

/* ARR của TIM3 = 4999 (PWM ~20kHz, timer clock 100MHz trên F411) */
#define PWM_ARR         4999u

/* TIM4 chỉ đếm 16-bit (encoder phải, F411 không có TIM8) -> cần cộng dồn
 * tràn số trong phần mềm. TIM2 (encoder trái) đếm 32-bit nên đọc thẳng
 * CNT là đủ, không cần cộng dồn. */
static int32_t  right_enc_accum   = 0;
static uint16_t right_enc_last_cnt = 0;

HAL_StatusTypeDef DRV_Motor_Init(void)
{
    /* Encoder trái (TIM2, 32-bit) + encoder phải (TIM4, 16-bit) */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* PWM 4 kênh: CH1/CH2 = RPWM/LPWM trái, CH3/CH4 = RPWM/LPWM phải */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    DRV_Motor_SetSpeed(0, 0);
    return DRV_Motor_ResetEncoder();
}

/* speed: -100..100 -> duty ra RPWM (thuận) hoặc LPWM (nghịch), kênh còn lại = 0.
 * ch_rpwm/ch_lpwm: TIM_CHANNEL_x tương ứng RPWM/LPWM của motor đó. */
static void set_channel_speed(int8_t speed, uint32_t ch_rpwm, uint32_t ch_lpwm)
{
    int32_t s = speed;
    if (s > 100)  s = 100;
    if (s < -100) s = -100;

    uint32_t duty = (uint32_t)((s < 0 ? -s : s) * (int32_t)PWM_ARR / 100);

    if (s >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, ch_rpwm, duty);
        __HAL_TIM_SET_COMPARE(&htim3, ch_lpwm, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, ch_rpwm, 0);
        __HAL_TIM_SET_COMPARE(&htim3, ch_lpwm, duty);
    }
}

HAL_StatusTypeDef DRV_Motor_SetSpeed(int8_t left, int8_t right)
{
    /* Xác nhận thực nghiệm 2026-08-19 (dây M+/M- đổi sang DRV8871, quan sát
     * trực tiếp bánh xe quay): bánh phải quay NGƯỢC so với bánh trái khi
     * cùng lệnh dấu -> đảo dấu bên phải tại đây để 2 bánh cùng chiều thật. */
    set_channel_speed(left,          TIM_CHANNEL_1, TIM_CHANNEL_2);
    set_channel_speed((int8_t)(-right), TIM_CHANNEL_3, TIM_CHANNEL_4);
    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right)
{
    /* Trái: TIM2 32-bit, đọc thẳng CNT (ép kiểu int32 phản ánh đúng dấu tiến/lùi) */
    *left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    /* Phải: TIM4 16-bit, cộng dồn delta để không mất số khi tràn */
    uint16_t cur = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t  delta = (int16_t)(cur - right_enc_last_cnt);
    right_enc_accum += delta;
    right_enc_last_cnt = cur;
    *right = right_enc_accum;

    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_ResetEncoder(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    right_enc_accum    = 0;
    right_enc_last_cnt = 0;
    return HAL_OK;
}
