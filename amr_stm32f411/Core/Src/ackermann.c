#include "ackermann.h"
#include <math.h>

/* Quy đổi độ <-> radian (tự định nghĩa, không phụ thuộc M_PI của math.h) */
#define DEG_TO_RAD          0.01745329252f
#define RAD_TO_DEG          57.29577951f

/* Hệ số quy đổi tuyến tính CŨ (rad/s -> độ), CHỈ còn dùng cho trường hợp xe
 * đứng yên (v ≈ 0) — lúc đó công thức đúng atan(H·ω/v) chia cho 0.
 * Giữ lại để vẫn chỉnh trước được góc lái khi test trên bàn ($VEL,0,ω),
 * KHÔNG dùng cho lúc xe đang chạy. */
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

    /* ---- 2. Góc lái VẬT LÝ cần có ----
     * Mô hình xe đạp: ω = (v/H)·tanθ  =>  θ = atan(H·ω / v).
     * Công thức này TỰ ĐÚNG cho cả lúc lùi (v<0): khi lùi, muốn thân xe quay
     * cùng chiều thì phải đánh lái ngược lại — phép chia có dấu lo việc đó. */
    float theta_phys_deg;
    if (fabsf(linear_x) < ACK_MIN_SPEED_MS) {
        /* Đứng yên: không suy được góc lái từ (ω, v). Xe Ackermann cũng vốn
         * KHÔNG quay tại chỗ được, nên đây chỉ là chế độ "chỉnh trước góc lái"
         * dùng khi test trên bàn — giữ nguyên cách quy đổi tuyến tính cũ để
         * không phá các bài test servo đang có. Bánh sau vẫn đứng yên vì
         * speed_f = 0. */
        theta_phys_deg = angular_z * K_ANGULAR_TO_DEG;
    } else {
        theta_phys_deg = atanf(ACK_WHEELBASE_M * angular_z / linear_x) * RAD_TO_DEG;
    }

    /* ---- 3. Cộng bù lệch cơ khí -> lệnh gửi servo, clamp theo giới hạn ---- */
    float servo_deg = theta_phys_deg + ACK_STEER_TRIM_DEG;
    if (servo_deg >  ACK_MAX_STEER_DEG) servo_deg =  ACK_MAX_STEER_DEG;
    if (servo_deg < -ACK_MAX_STEER_DEG) servo_deg = -ACK_MAX_STEER_DEG;
    *steer_deg = servo_deg;

    /* ---- 4. Góc VẬT LÝ thực sự đạt được, để tính vi sai ----
     * ⚠️ PHẢI TRỪ LẠI TRIM. `servo_deg` là LỆNH gửi servo, không phải góc bánh
     * thật: trim bù lệch tâm cơ khí, nên khi lệnh = trim thì bánh đang THẲNG.
     * Nếu lấy thẳng servo_deg để tính vi sai, lúc đi thẳng (ω=0 -> servo=1.5°)
     * sẽ sinh vi sai giả 1.35% -> /odom báo quay ma ~0.9°/s -> đi thẳng 1 phút
     * lệch ~54°, đủ phá SLAM. Cũng phải lấy góc SAU clamp, để vi sai luôn khớp
     * với góc lái thật sự đạt được (khác Hiwonder: họ từ chối lệnh khi quá
     * giới hạn, ta clamp nên phải tự đồng bộ lại). */
    float theta_ach_rad = (servo_deg - ACK_STEER_TRIM_DEG) * DEG_TO_RAD;

    /* ---- 5. Vi sai 2 bánh sau (Kinematics Analysis.pdf trang 6) ----
     *   V_L = V·(1 − D·tanθ/2H)   ,   V_R = V·(1 + D·tanθ/2H)
     * Dấu khớp sẵn với quy ước của ta (θ dương = rẽ TRÁI): khi rẽ trái, bánh
     * PHẢI là bánh ngoài nên phải quay nhanh hơn -> V_R > V_L. ⚠️ Vẫn phải
     * xác nhận lại bằng thực nghiệm sau khi nạp (bài học lặp lại nhiều lần
     * trong dự án: không tin suy luận dấu trên giấy). */
    float k  = (ACK_TRACK_WIDTH_M * tanf(theta_ach_rad)) / (2.0f * ACK_WHEELBASE_M);
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
