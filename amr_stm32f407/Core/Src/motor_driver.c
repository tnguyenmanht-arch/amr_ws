#include "motor_driver.h"
#include "motor_pid.h"
#include "main.h"
#include "dwt_log.h"   /* Đo timing baseline (branch baseline-dwt) */

/* ===== Gán kênh PWM (board Hiwonder ROS Robot Controller, schematic V1.1 trang 3) =====
 * TIM1 (APB2 timer clock 168 MHz, ARR = 8399 -> PWM 20 kHz), cả 4 kênh:
 *   Bánh TRÁI  = cổng motor M1: PE13 = TIM1_CH3, PE14 = TIM1_CH4 (driver U5)
 *   Bánh PHẢI  = cổng motor M2: PE9  = TIM1_CH1, PE11 = TIM1_CH2 (driver U10)
 * "FWD" = kênh nhận PWM khi speed > 0, "REV" = kênh nhận PWM khi speed < 0.
 * ⚠️ CHƯA XÁC MINH chân nào là FI/BI của YX-4055AM, và bánh nào cắm cổng nào.
 * Nếu 1 bánh quay NGƯỢC chiều khi ra lệnh tiến -> HOÁN ĐỔI FWD/REV của bánh
 * đó ở đây (thay cho việc đảo dấu output như trên F411). Quy ước dấu KHÔNG cố
 * định qua các lần đấu dây — luôn quan sát lại bằng mắt (bánh nhấc khỏi đất). */
#define LEFT_CH_FWD     TIM_CHANNEL_3   /* PE13 */
#define LEFT_CH_REV     TIM_CHANNEL_4   /* PE14 */
#define RIGHT_CH_FWD    TIM_CHANNEL_1   /* PE9  */
#define RIGHT_CH_REV    TIM_CHANNEL_2   /* PE11 */

/* Encoder: bánh TRÁI (M1) = TIM5 (PA0/PA1), bánh PHẢI (M2) = TIM2 (PA15/PB3).
 * Cả 2 đều là timer 32-bit -> đọc thẳng CNT, KHÔNG cần cộng dồn tràn số
 * (khác F411 phải cộng dồn cho TIM4 16-bit). */
#define ENC_LEFT_HTIM   htim5
#define ENC_RIGHT_HTIM  htim2

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
 * ⚠️ ĐO LẠI nếu đổi motor/pin/bánh hoặc điện áp nguồn thay đổi đáng kể.
 * ⚠️ 2026-09-29: số 69 đo trên DRV8871 (F411). Board mới dùng YX-4055AM +
 * cầu chì 2A + pin LiPo 3S -> tốc độ tối đa có thể khác -> ĐO LẠI khi
 * bring-up (chạy hết ga, bánh nhấc, lấy bánh chậm hơn), rồi mới verify
 * lại Kp/Ki (không tự đổi Kp/Ki — cần tune thực nghiệm). */
#define MAX_TICKS_PER_INTERVAL  69.0f

/* ⚠️⚠️ AN TOÀN — DẤU ENCODER CHƯA ĐO TRÊN BOARD MỚI (F407, 2026-09-29) ⚠️⚠️
 * Giá trị F411 cũ (trái -1, phải +1) KHÔNG áp dụng được: đổi board, đổi
 * driver, đổi cổng encoder. Để tạm +1.0f cả hai, BẮT BUỘC đo lại trước khi
 * tin dùng vòng PID.
 * Nếu sai dấu, PID "nhìn nhầm" chiều chạy -> TĂNG GA tới kẹp trần thay vì
 * hãm lại (đã xảy ra thật 2026-09-04 với bánh trái trên F411). Lệnh dừng vẫn
 * có hiệu lực nhờ nhánh target=0 bypass PID, nhưng xe sẽ chạy hết ga khi có
 * lệnh khác 0.
 * Cách đo AN TOÀN (bánh nhấc khỏi đất, có xác nhận của user): cho mỗi bánh
 * quay TIẾN ở duty thấp bằng firmware bring-up (open-loop, không qua PID),
 * đọc delta encoder:
 *   - delta dương khi tiến -> giữ +1.0f
 *   - delta âm khi tiến    -> đổi thành -1.0f
 * Làm SAU khi đã chỉnh LEFT/RIGHT_CH_FWD/REV để bánh tiến đúng chiều. */
#define RIGHT_ENCODER_SIGN      1.0f
#define LEFT_ENCODER_SIGN       1.0f

static PID_t    pid_l, pid_r;
static float    target_l_pct = 0.0f, target_r_pct = 0.0f; /* Mục tiêu hiện tại, -100..100 */
static int32_t  pid_last_l_total = 0, pid_last_r_total = 0; /* Baseline riêng cho vòng PID,
                                                              * độc lập với việc $ODO đọc
                                                              * tổng tick — không ảnh hưởng
                                                              * lẫn nhau. */
static uint32_t last_pid_tick_ms = 0;

HAL_StatusTypeDef DRV_Motor_Init(void)
{
    /* Encoder trái (TIM5) + phải (TIM2), cả 2 đều 32-bit */
    HAL_TIM_Encoder_Start(&ENC_LEFT_HTIM,  TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENC_RIGHT_HTIM, TIM_CHANNEL_ALL);

    /* PWM 4 kênh TIM1 (duty = 0 từ MX_TIM1_Init). TIM1 là timer "advanced":
     * HAL_TIM_PWM_Start tự bật MOE (main output enable) -> không cần làm tay. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    PID_Init(&pid_l, PID_KP, PID_KI, -100.0f, 100.0f);
    PID_Init(&pid_r, PID_KP, PID_KI, -100.0f, 100.0f);
    last_pid_tick_ms = HAL_GetTick();

    DRV_Motor_SetSpeed(0, 0);
    return DRV_Motor_ResetEncoder();
}

/* speed: -100..100 -> duty ra kênh FWD (thuận) hoặc REV (nghịch), kênh còn lại = 0.
 * Cùng kiểu "PWM 1 chân, chân kia = 0" như DRV8871 trên F411.
 * ⚠️ Bảng chân lý YX-4055AM chưa có datasheet: chưa biết 0/0 là thả trôi hay
 * phanh -> xác minh ở bring-up (quan sát bánh khi về 0). */
static void set_channel_speed(int8_t speed, uint32_t ch_fwd, uint32_t ch_rev)
{
    int32_t s = speed;
    if (s > 100)  s = 100;
    if (s < -100) s = -100;

    /* Đọc ARR thật từ timer thay vì hằng số -> đổi tần số PWM trong .ioc
     * thì code tự đúng (bài học F446->F411: ARR phải tính lại theo clock). */
    uint32_t arr  = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t duty = (uint32_t)(s < 0 ? -s : s) * arr / 100u;

    if (s >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, ch_fwd, duty);
        __HAL_TIM_SET_COMPARE(&htim1, ch_rev, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, ch_fwd, 0);
        __HAL_TIM_SET_COMPARE(&htim1, ch_rev, duty);
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
        set_channel_speed(0, LEFT_CH_FWD,  LEFT_CH_REV);
        set_channel_speed(0, RIGHT_CH_FWD, RIGHT_CH_REV);
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
    /* [ĐO] Mốc bắt đầu thân PID: đặt SAU đoạn gate thời gian, nên chỉ ghi
     * những lần PID chạy thật. Hiệu t_start giữa 2 bản ghi EV_PID liên tiếp
     * = chu kỳ PID thực tế (script tính jitter từ đây). */
    uint32_t dwt_t0 = DWT_Now();

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

    /* Chiều quay từng bánh do cặp kênh FWD/REV quyết định (xem đầu file) —
     * thay cho việc đảo dấu out_r như trên F411. PID ở trên không cần biết. */
    set_channel_speed((int8_t)out_l, LEFT_CH_FWD,  LEFT_CH_REV);
    set_channel_speed((int8_t)out_r, RIGHT_CH_FWD, RIGHT_CH_REV);

    /* [ĐO] Kết thúc thân PID (đã ghi PWM xong) */
    DWT_Log(EV_PID, dwt_t0, DWT_Now(), 0u);
}

HAL_StatusTypeDef DRV_Motor_GetEncoder(int32_t *left, int32_t *right)
{
    /* Cả 2 bánh: timer 32-bit, đọc thẳng CNT. Ép kiểu int32 để lùi ra số âm
     * đúng như cách $ODO đang gửi (tick THÔ, chưa bù dấu — bù dấu cho /odom
     * làm bên ROS bằng left/right_encoder_sign). */
    *left  = (int32_t)__HAL_TIM_GET_COUNTER(&ENC_LEFT_HTIM);
    *right = (int32_t)__HAL_TIM_GET_COUNTER(&ENC_RIGHT_HTIM);

    return HAL_OK;
}

HAL_StatusTypeDef DRV_Motor_ResetEncoder(void)
{
    __HAL_TIM_SET_COUNTER(&ENC_LEFT_HTIM,  0);
    __HAL_TIM_SET_COUNTER(&ENC_RIGHT_HTIM, 0);

    /* Đồng bộ lại baseline + xóa "trí nhớ" PID, tránh 1 delta ảo khổng lồ
     * (so với tổng tick vừa bị đưa về 0) làm PID giật ngay sau reset. */
    pid_last_l_total = 0;
    pid_last_r_total = 0;
    PID_Reset(&pid_l);
    PID_Reset(&pid_r);

    return HAL_OK;
}
