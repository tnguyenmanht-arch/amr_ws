#include "ackermann.h"
#include <math.h>

/* Quy đổi độ <-> radian (tự định nghĩa, không phụ thuộc M_PI của math.h) */
#define DEG_TO_RAD          0.01745329252f
#define RAD_TO_DEG          57.29577951f

/* Hệ số quy đổi tuyến tính CŨ (rad/s -> độ), CHỈ còn dùng cho trường hợp xe
 * đứng yên (v ≈ 0) — lúc đó công thức đúng atan(H·ω/v) chia cho 0.
 * Giữ lại để vẫn chỉnh trước được góc lái khi test trên bàn ($VEL,0,ω),
 * KHÔNG dùng cho lúc xe đang chạy.
 * ⚠️ Từ 2026-09-06 hằng số này mang đơn vị ĐỘ GÓC BÁNH (không phải độ lệnh
 * servo như trước), để đồng nhất đơn vị với nhánh atan(). Hệ quả trên bàn:
 * cùng một ω sẽ cho ra lệnh servo KHÁC trước, và ω ≥ 0.6 đều bão hoà ở giới
 * hạn góc bánh ±18.0° — đúng khả năng thật của cơ cấu, không phải lỗi.
 * (Comment cũ ghi "ω ≥ 0.5 bão hoà ±15.0°" là số của thời TRIM=−4.8, lỗi thời.) */
#define K_ANGULAR_TO_DEG    30.0f

void CALC_Ackermann(float linear_x, float angular_z,
                    int8_t *speed_l, int8_t *speed_r,
                    float *steer_deg)
{
    /* ---- 1. Vận tốc thẳng -> tốc độ danh nghĩa của xe (-100..100) ----
     * Đây là tốc độ tại TÂM TRỤC SAU (V trong công thức), chưa chia vi sai. */
    float speed_f = (linear_x / ACK_MAX_SPEED_MS) * 100.0f;
    if (speed_f >  100.0f) speed_f =  100.0f;
    if (speed_f < -100.0f) speed_f = -100.0f;

    /* ---- 2. Góc BÁNH XE vật lý cần có (độ) ----
     * Mô hình xe đạp: ω = (v/H)·tanθ  =>  θ = atan(H·ω / v).
     * Công thức này TỰ ĐÚNG cho cả lúc lùi (v<0): khi lùi, muốn thân xe quay
     * cùng chiều thì phải đánh lái ngược lại — phép chia có dấu lo việc đó. */
    float theta_deg;
    if (fabsf(linear_x) < ACK_MIN_SPEED_MS) {
        /* Đứng yên: không suy được góc lái từ (ω, v). Xe Ackermann cũng vốn
         * KHÔNG quay tại chỗ được, nên đây chỉ là chế độ "chỉnh trước góc lái"
         * dùng khi test trên bàn. Bánh sau vẫn đứng yên vì speed_f = 0. */
        theta_deg = angular_z * K_ANGULAR_TO_DEG;
    } else {
        theta_deg = atanf(ACK_WHEELBASE_M * angular_z / linear_x) * RAD_TO_DEG;
    }

    /* ---- 3. Giới hạn góc bánh, ĐỐI XỨNG 2 CHIỀU, theo CẢ HAI ràng buộc ----
     * (a) Từ giới hạn LỆNH SERVO: nếu tâm servo lệch (TRIM ≠ 0) thì cùng biên
     *     ±ACK_MAX_SERVO_DEG lại cho ra 2 góc bánh khác nhau. Lấy phía HẸP HƠN
     *     làm chung cho cả 2 chiều -> xe cua trái/phải như nhau, và lệnh servo
     *     chắc chắn nằm gọn trong ±ACK_MAX_SERVO_DEG.
     * (b) Từ giới hạn GÓC BÁNH (bánh chạm khung): ACK_MAX_WHEEL_DEG.
     * Lấy cái NHỎ HƠN.
     * Với bộ hằng số hiện tại (MAX_SERVO=35, TRIM=0.0, GAIN=0.597): (a) cho
     * ±20.9° đối xứng cả 2 chiều, nên (b)=18.0° mới là cái CHẶN TRƯỚC.
     * ⚠️ Đừng hardcode con số vào đây — tính runtime nên tự đúng lại khi đổi
     * TRIM/GAIN/giới hạn. (Comment cũ ghi "(a) chặn trước 15.0° < 30°" và
     * "+20.8°/−15.0°" là số của thời MAX_SERVO=30 + TRIM=−4.8, đã lỗi thời.) */
    float lim_l = ( ACK_MAX_SERVO_DEG - ACK_STEER_TRIM_DEG) * ACK_STEER_GAIN;
    float lim_r = (-ACK_MAX_SERVO_DEG - ACK_STEER_TRIM_DEG) * ACK_STEER_GAIN;
    float lim   = fminf(fabsf(lim_l), fabsf(lim_r));
    if (lim > ACK_MAX_WHEEL_DEG) lim = ACK_MAX_WHEEL_DEG;
    if (theta_deg >  lim) theta_deg =  lim;
    if (theta_deg < -lim) theta_deg = -lim;

    /* ---- 4. Góc bánh -> LỆNH servo ----
     * Đảo ngược quan hệ đã hiệu chuẩn  θ = GAIN·(servo − TRIM):
     *     servo = θ/GAIN + TRIM
     * Kiểm tra nhanh: θ=0 -> servo = TRIM = −4.8° = đúng vị trí bánh thẳng. */
    float servo_deg = theta_deg / ACK_STEER_GAIN + ACK_STEER_TRIM_DEG;

    /* Chốt an toàn cuối cùng bảo vệ tay đòn lái. Sau bước 3 thì về lý thuyết
     * không bao giờ chạm tới, nhưng giới hạn cơ khí thì luôn nên có 2 lớp. */
    if (servo_deg >  ACK_MAX_SERVO_DEG) servo_deg =  ACK_MAX_SERVO_DEG;
    if (servo_deg < -ACK_MAX_SERVO_DEG) servo_deg = -ACK_MAX_SERVO_DEG;
    *steer_deg = servo_deg;

    /* ---- 5. Vi sai 2 bánh sau (Kinematics Analysis.pdf trang 6) ----
     *   V_L = V·(1 − D·tanθ/2H)   ,   V_R = V·(1 + D·tanθ/2H)
     * ⚠️ θ ở đây PHẢI là góc BÁNH THẬT — `theta_deg` đã đúng nghĩa đó (đã
     * clamp ở bước 3, và không dính TRIM vì TRIM chỉ là bù lệch tâm của LỆNH
     * servo, không phải góc bánh). Nếu lỡ dùng `servo_deg` thì lúc đi thẳng
     * (θ=0 -> servo=−4.8°) sẽ sinh vi sai giả ~4.3% -> /odom báo quay ma ->
     * đi thẳng vài chục giây là lệch đủ để phá SLAM.
     * Dấu khớp sẵn với quy ước của ta (θ dương = rẽ TRÁI): khi rẽ trái, bánh
     * PHẢI là bánh ngoài nên quay nhanh hơn -> V_R > V_L (đã xác nhận thực
     * nghiệm 2026-09-06: cua trái tỉ số P/T = 1.736 vs lý thuyết 1.745). */
    float k  = (ACK_TRACK_WIDTH_M * tanf(theta_deg * DEG_TO_RAD)) / (2.0f * ACK_WHEELBASE_M);
    float vl = speed_f * (1.0f - k);
    float vr = speed_f * (1.0f + k);

    /* Nếu bánh ngoài vượt trần thì HẠ TỈ LỆ CẢ HAI, giữ nguyên tỉ số vi sai —
     * clamp cụt riêng bánh ngoài sẽ làm sai tỉ số và trượt lốp trở lại. */
    float vmax = (fabsf(vl) > fabsf(vr)) ? fabsf(vl) : fabsf(vr);
    if (vmax > 100.0f) {
        vl = vl * 100.0f / vmax;
        vr = vr * 100.0f / vmax;
    }

    *speed_l = (int8_t)vl;
    *speed_r = (int8_t)vr;
}
