#include "motor_pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->integral = 0.0f;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
}

float PID_Update(PID_t *pid, float setpoint, float measured, float dt_s)
{
    float error = setpoint - measured;

    pid->integral += error * dt_s;

    float output = pid->kp * error + pid->ki * pid->integral;

    /* Anti-windup: nếu output đã vượt giới hạn (motor đã "hết ga" theo hướng
     * đó), kẹp output lại VÀ rút phần vừa cộng dồn ra khỏi integral. Nếu
     * không làm bước này, integral sẽ tiếp tục phình to vô hạn trong lúc
     * output bị kẹp (ví dụ setpoint quá cao so với khả năng motor) -> khi
     * error đổi dấu, integral khổng lồ đó khiến motor phản ứng trễ và giật
     * mạnh theo hướng ngược lại. */
    if (output > pid->out_max) {
        output = pid->out_max;
        if (error > 0.0f) pid->integral -= error * dt_s;
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (error < 0.0f) pid->integral -= error * dt_s;
    }

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
}
