#include "motor_driver.h"
#include "motor_pid.h"
#include "main.h"

/* ARR của TIM3 = 4999 (PWM ~20kHz, timer clock 100MHz trên F411) */
#define PWM_ARR         4999u

/* TIM4 chỉ đếm 16-bit (encoder phải, F411 không có TIM8) -> cần cộng dồn
 * tràn số trong phần mềm. TIM2 (encoder trái) đếm 32-bit nên đọc thẳng
 * CNT là đủ, không cần cộng dồn. */
static int32_t  right_enc_accum   = 0;
static uint16_t right_enc_last_cnt = 0;

/* ===================== Vòng PID tốc độ (closed-loop) =====================
 * Xem giải thích PI/cách chỉnh Kp,Ki bằng thực nghiệm trong motor_pid.h.
 * Toàn bộ hằng số dưới đây là SỐ TẠM, CHƯA TUNE — bắt buộc phải đo/chỉnh lại
 * trên xe thật, không dùng thẳng cho việc chạy thật trước khi test kỹ. */

#define PID_INTERVAL_MS        10u     /* Khớp đúng nhịp đọc encoder cho $ODO
                                         * (100Hz) đã có sẵn trong main.c, để
                                         * không cần thêm timer/ngắt mới. */

/* Đã verify thực nghiệm 2026-09-05 (sau khi hiệu chuẩn MAX_TICKS_PER_INTERVAL
 * đúng bằng 69): chạy 30s ở target 50% cho sai số bám chỉ ~2% dưới mục tiêu,
 * độ dao động bánh trái ±0.3% / bánh phải ±1.7%, không overshoot, tiếng động
 * cơ đều. Với thang đo hiện tại, Kp=1.5 nghĩa là sai số bằng cả dải tốc độ
 * (69 tick) mới đẩy output tới trần 100 -> tỉ lệ hợp lý, không bão hoà sớm.
 * KHÔNG tự ý tăng Ki để triệt nốt 2% sai số dư: rủi ro mang overshoot/dao
 * động quay lại, không đáng với ứng dụng này. */
#define PID_KP                  1.5f
#define PID_KI                  8.0f

/* Số tick encoder đo được trong 1 chu kỳ PID_INTERVAL_MS khi chạy hết ga
 * (target=100%). ĐÃ ĐO THỰC NGHIỆM 2026-09-05 (bánh nhấc khỏi đất, chạy
 * $VEL,0.500 liên tục 7s, bỏ 2.5s tăng tốc, lấy trung bình 4.49s ổn định):
 *   bánh trái 68.7 tick/10ms, bánh phải 70.0 tick/10ms
 * Lấy 69 (bánh chậm hơn) để CẢ 2 bánh đều bám được setpoint — nếu lấy số
 * cao hơn, bánh chậm sẽ không bao giờ đạt tới và PID bão hoà 100% duty vĩnh
 * viễn (thoái hoá thành open-loop full ga, đúng lỗi đã gặp khi để tạm 400).
 * ⚠️ ĐO LẠI nếu đổi motor/pin/bánh hoặc điện áp nguồn thay đổi đáng kể. */
#define MAX_TICKS_PER_INTERVAL  69.0f

/* ⚠️ AN TOÀN: chiều PWM bánh phải đã bị đảo dấu ở tầng output (xem
 * DRV_Motor_SetSpeed) do lắp DRV8871 ngược chiều cơ khí so với bánh trái —
 * đây là quy ước Ở TẦNG OUTPUT. Chiều ĐẾM của encoder bánh phải (tick tăng
 * hay giảm khi tiến) là 1 mạch tín hiệu HOÀN TOÀN KHÁC, ĐỘC LẬP với chiều
 * PWM, và CHƯA được xác nhận thực nghiệm là có bị đảo theo hay không kể từ
 * lần đấu lại dây DRV8871 (2026-08-19).
 * Nếu đoán sai giá trị này, vòng PID bánh phải sẽ "nhìn nhầm" chiều chạy ->
 * thay vì giảm ga khi đã đủ tốc độ, nó sẽ tiếp tục TĂNG GA tới khi kẹp trần
 * (chạy vọt hết cỡ, không hãm lại được) — nguy hiểm hơn giật cục nhiều.
 * BẮT BUỘC test an toàn trước khi tin dùng: nhấc bánh phải khỏi mặt đất, gửi
 * target thấp (~20-30%), quan sát:
 *   - Đúng: bánh quay ổn định ở tốc độ vừa phải, không tăng dần tới max.
 *   - Sai (cần đổi -1.0f thành +1.0f dưới đây): bánh tăng tốc dần tới hết cỡ
 *     dù target thấp, hoặc rung/kêu bất thường ngay khi vừa cấp lệnh -> CẮT
 *     NGUỒN NGAY, đổi hằng số này rồi build/nạp lại. */
#define RIGHT_ENCODER_SIGN      1.0f

/* ⚠️ Xác nhận thực nghiệm 2026-09-04 (test PID lần đầu, xem $PIDDBG log):
 * encoder TRÁI đếm NGƯỢC chiều với PWM dương -- ra lệnh +100% duty liên tục
 * (out_l dính cứng ở +100 suốt 6 giây) nhưng delta_l luôn ÂM ổn định
 * (~-72 tick/10ms không đổi), tức PID không bao giờ "kéo được" measured về
 * gần setpoint -- bánh trái quay full ga không kiểm soát suốt bài test.
 * Đây là bằng chứng thực nghiệm khá chắc (nhất quán qua hàng chục chu kỳ
 * PID liên tiếp), không phải suy đoán -- đảo dấu tại đây để vòng PID trái
 * đọc đúng chiều. */
#define LEFT_ENCODER_SIGN      -1.0f

static PID_t    pid_l, pid_r;
static float    target_l_pct = 0.0f, target_r_pct = 0.0f; /* Mục tiêu hiện tại, -100..100 */
static int32_t  pid_last_l_total = 0, pid_last_r_total = 0; /* Baseline riêng cho vòng PID,
                                                              * độc lập với việc $ODO đọc
                                                              * tổng tick — không ảnh hưởng
                                                              * lẫn nhau. */
static uint32_t last_pid_tick_ms = 0;

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

    PID_Init(&pid_l, PID_KP, PID_KI, -100.0f, 100.0f);
    PID_Init(&pid_r, PID_KP, PID_KI, -100.0f, 100.0f);
    last_pid_tick_ms = HAL_GetTick();

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
    /* Chỉ LƯU mục tiêu -- PWM thực tế do DRV_Motor_UpdatePID() tính mỗi chu
     * kỳ dựa trên encoder (closed-loop, xem giải thích ở đầu file). */
    target_l_pct = (float)left;
    target_r_pct = (float)right;

    /* Dừng hẳn (target=0): cắt PWM NGAY LẬP TỨC, không chờ chu kỳ PID kế
     * tiếp (tối đa PID_INTERVAL_MS = 10ms sau), để giữ đúng hành vi an toàn
     * đã verify trước đây (watchdog $VEL dừng xe trong ~85ms). Đồng thời xóa
     * sạch phần tích lũy (I) của PID, tránh "trí nhớ" cũ làm xe trôi/giật
     * khi cấp lệnh chạy lại. */
    if (left == 0 && right == 0) {
        set_channel_speed(0, TIM_CHANNEL_1, TIM_CHANNEL_2);
        set_channel_speed(0, TIM_CHANNEL_3, TIM_CHANNEL_4);
        PID_Reset(&pid_l);
        PID_Reset(&pid_r);
    }

    return HAL_OK;
}

void DRV_Motor_UpdatePID(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_pid_tick_ms < PID_INTERVAL_MS) {
        return; /* Chưa tới chu kỳ PID kế tiếp -- không làm gì cả. */
    }
    float dt_s = (float)(now - last_pid_tick_ms) / 1000.0f;
    last_pid_tick_ms = now;

    /* Đọc tổng tick hiện tại (dùng lại đúng hàm mà $ODO cũng gọi) rồi tự
     * tính delta riêng cho vòng PID -- không đụng/ảnh hưởng tới cách $ODO
     * đang báo cáo tổng tick lên Jetson. */
    int32_t total_l = 0, total_r = 0;
    DRV_Motor_GetEncoder(&total_l, &total_r);

    float delta_l = (float)(total_l - pid_last_l_total) * LEFT_ENCODER_SIGN;
    float delta_r = (float)(total_r - pid_last_r_total) * RIGHT_ENCODER_SIGN;
    pid_last_l_total = total_l;
    pid_last_r_total = total_r;

    /* ⚠️ AN TOÀN BẮT BUỘC (fix 2026-09-04, sau sự cố bánh trái quay không
     * dừng được): nếu target = 0 (dừng hẳn -- dù do lệnh dừng thật hay do
     * watchdog $VEL trip), TUYỆT ĐỐI ép duty = 0 trực tiếp, KHÔNG chạy qua
     * PID_Update() dù chỉ 1 lần. Lý do: PID_Update() luôn tin encoder tuyệt
     * đối -- nếu encoder đọc sai (nhiễu, đứt dây, lỗi dấu như bánh trái vừa
     * gặp), nó có thể "tưởng" còn sai số cần sửa và tiếp tục đạp ga dù target
     * đã về 0, ĐÈ LÊN watchdog/lệnh dừng -- xe không thể dừng bằng phần mềm.
     * Kiểm tra target TRƯỚC, không phụ thuộc bất kỳ phép tính nào từ
     * delta_l/delta_r, để lệnh dừng luôn có hiệu lực vô điều kiện. */
    float out_l, out_r;

    if (target_l_pct == 0.0f) {
        out_l = 0.0f;
        PID_Reset(&pid_l);
    } else {
        float setpoint_l = (target_l_pct / 100.0f) * MAX_TICKS_PER_INTERVAL;
        out_l = PID_Update(&pid_l, setpoint_l, delta_l, dt_s);
    }

    if (target_r_pct == 0.0f) {
        out_r = 0.0f;
        PID_Reset(&pid_r);
    } else {
        float setpoint_r = (target_r_pct / 100.0f) * MAX_TICKS_PER_INTERVAL;
        out_r = PID_Update(&pid_r, setpoint_r, delta_r, dt_s);
    }

    /* Đảo dấu output bên phải ở ĐÚNG VỊ TRÍ CŨ (tầng ghi PWM) -- giữ nguyên
     * quy ước "đảo dấu chỉ ở output" như code open-loop trước đây, PID ở
     * trên hoàn toàn không biết/không cần biết chuyện đảo dấu này. */
    set_channel_speed((int8_t)out_l,        TIM_CHANNEL_1, TIM_CHANNEL_2);
    set_channel_speed((int8_t)(-out_r),     TIM_CHANNEL_3, TIM_CHANNEL_4);
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

    /* Đồng bộ lại baseline + xóa "trí nhớ" PID, tránh 1 delta ảo khổng lồ
     * (so với tổng tick vừa bị đưa về 0) làm PID giật ngay sau reset. */
    pid_last_l_total = 0;
    pid_last_r_total = 0;
    PID_Reset(&pid_l);
    PID_Reset(&pid_r);

    return HAL_OK;
}
