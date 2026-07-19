#include "motor_driver.h"
#include "main.h"

/* ARR của TIM3 = 3599 (PWM 20kHz, timer clock 72MHz trên F103) */
#define PWM_ARR         3599u

/* F103 không có timer 32-bit nào -> CẢ TIM2 (trái) lẫn TIM4 (phải) chỉ đếm
 * 16-bit, cần cộng dồn tràn số trong phần mềm cho cả 2 bên (khác F411 chỉ
 * cần cộng dồn bên phải vì TIM2 ở đó là 32-bit). */
static int32_t  left_enc_accum    = 0;
static uint16_t left_enc_last_cnt = 0;
static int32_t  right_enc_accum   = 0;
static uint16_t right_enc_last_cnt = 0;

HAL_StatusTypeDef DRV_Motor_Init(void)
{
    /* Encoder trái (TIM2, 16-bit) + encoder phải (TIM4, 16-bit) */
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
    /* Dây đấu lại từ đầu cho F103 (giống quy trình F411) -> CHƯA rõ có cần
     * đảo dấu bên nào không, phải đo thực nghiệm enc_l/enc_r khi tiến thẳng
     * rồi chỉnh lại đây nếu 2 bánh ngược dấu nhau. */
    set_channel_speed(left,  TIM_CHANNEL_1, TIM_CHANNEL_2);
    set_channel_speed(right, TIM_CHANNEL_3, TIM_CHANNEL_4);
    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right)
{
    /* Trái: TIM2 16-bit, cộng dồn delta để không mất số khi tràn */
    uint16_t cur_l = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t  delta_l = (int16_t)(cur_l - left_enc_last_cnt);
    left_enc_accum += delta_l;
    left_enc_last_cnt = cur_l;
    *left = left_enc_accum;

    /* Phải: TIM4 16-bit, cộng dồn delta để không mất số khi tràn */
    uint16_t cur_r = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t  delta_r = (int16_t)(cur_r - right_enc_last_cnt);
    right_enc_accum += delta_r;
    right_enc_last_cnt = cur_r;
    *right = right_enc_accum;

    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_ResetEncoder(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    left_enc_accum     = 0;
    left_enc_last_cnt  = 0;
    right_enc_accum    = 0;
    right_enc_last_cnt = 0;
    return HAL_OK;
}
