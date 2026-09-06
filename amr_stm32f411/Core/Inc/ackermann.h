#ifndef __ACKERMANN_H__
#define __ACKERMANN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===== Thông số hình học robot Ackermann =====
 * Đo trên xe thực tế (xem CLAUDE.md). Điều chỉnh nếu cơ khí thay đổi. */
#define ACK_WHEELBASE_M         0.210f  /* H: khoảng cách trục trước-sau (m) */
#define ACK_TRACK_WIDTH_M       0.217f  /* D: khoảng cách 2 bánh trái-phải (m) */
#define ACK_WHEEL_RADIUS_M      0.050f  /* Bán kính bánh xe (m) — bánh Ø100mm */
#define ACK_MAX_SPEED_MS        0.5f    /* Vận tốc thẳng ứng với speed = 100 (m/s) */

/* ===== HAI giới hạn cơ khí KHÁC NHAU — trước 2026-09-06 bị gộp làm một vì
 * tưởng góc servo = góc bánh. Từ khi biết ACK_STEER_GAIN thì phải tách. =====
 * Code áp dụng CẢ HAI, cái nào chạm trước thì cái đó chặn. */

/* (a) Giới hạn LỆNH gửi servo — chỉ là trần kỹ thuật, KHÔNG phải cữ cơ khí.
 * Bản thân servo đi được ±120° (vị trí 0..1000, xem servo_buslinker.h), nên
 * con số ở đây rộng rãi thoải mái. Ràng buộc thật nằm ở (b). */
#define ACK_MAX_SERVO_DEG       35.0f

/* (b) ⭐ GIỚI HẠN THẬT SỰ CỦA HỆ THỐNG LÁI: góc bánh trước trước khi BÁNH
 * CHẠM KHUNG XE. Quan sát trực tiếp 2026-09-06: ở góc bánh 15.04° bánh đã
 * "sắp chạm" khung → cữ thật nằm rất gần đó, KHÔNG phải 30° như bản trước
 * ghi (số 30 đó chưa hề được đo, đặt quá lỏng nên vô tác dụng).
 * ⚠️ ĐANG Ở CHẾ ĐỘ ĐO: đặt 18.0 để dò xem chạm ở đâu, phải chỉnh lại xuống
 * giá trị an toàn (điểm chạm TRỪ ~2° dự phòng cho rung/xóc/nén lốp/rơ lái)
 * ngay sau khi đo xong. KHÔNG để 18.0 khi chạy tự động. */
#define ACK_MAX_WHEEL_DEG       18.0f

/* Ngưỡng coi như "đứng yên": dưới mức này không suy được góc lái từ (ω, v)
 * vì công thức atan(H·ω/v) chia cho 0. Xem xử lý trong ackermann.c. */
#define ACK_MIN_SPEED_MS        1e-4f

/* Bù lệch tâm cơ khí: LỆNH servo ứng với lúc bánh xe THẲNG.
 * ĐO THỰC TẾ trên Jetson 2026-09-06 (test vòng tròn): xe đi thẳng khi
 * steer_deg = −4.8°, không phải 0°. Giá trị cũ +1.5f là SAI — nó ước lượng
 * bằng mắt, và sai số bị tham số ROS `steering_trim_angular_z=-0.206` che mất.
 * Sau khi firmware đổi sang θ = atan(H·ω/v) thì trim bên ROS không che được
 * nữa (trim tính bằng rad/s cho ra GÓC PHỤ THUỘC TỐC ĐỘ, còn lệch tâm cơ khí
 * là một GÓC KHÔNG ĐỔI) → phải sửa ở đây. `steering_trim_angular_z` đã đặt về
 * 0 vĩnh viễn bên ROS: trim nay gộp về đúng 1 chỗ duy nhất là hằng số này.
 * CẦN ĐO LẠI nếu tháo/lắp lại servo hoặc tay đòn lái. */
#define ACK_STEER_TRIM_DEG      -4.8f

/* Tỉ số truyền tay đòn lái: góc_bánh_thật = GAIN × (steer_deg − TRIM).
 * ĐO THỰC TẾ trên Jetson 2026-09-06 bằng 2 test vòng tròn ở 2 góc khác nhau
 * (servo +22.2° → Ø1.46m → bánh 16.05° → hệ số 0.594;
 *  servo +30.0° → Ø1.10m → bánh 20.90° → hệ số 0.601), cộng điểm gốc
 * (bánh thẳng tại servo −4.8°). Hai điểm độc lập lệch nhau 1% → tuyến tính.
 * ⚠️ Đây là hệ số HIỆU DỤNG, đã gộp cả trượt lốp bánh trước (khung này lái
 * SONG SONG, không phải Ackermann thật) — đo trên sàn cứng với lốp hiện tại.
 * Đổi mặt sàn (thảm) hoặc thay lốp thì phải đo lại. */
#define ACK_STEER_GAIN          0.597f

/**
 * @brief  Chuyển cmd_vel (Twist) -> lệnh phần cứng cho xe Ackermann.
 * @param  linear_x   Vận tốc thẳng mong muốn (m/s); dương = tiến
 * @param  angular_z  Vận tốc góc mong muốn (rad/s); dương = quay trái
 * @param  speed_l    [out] Tốc độ bánh trái  (-100..100)
 * @param  speed_r    [out] Tốc độ bánh phải (-100..100)
 * @param  steer_deg  [out] LỆNH gửi servo (độ); dương = TRÁI, âm = PHẢI.
 *                    ⚠️ Đây là lệnh servo, KHÔNG phải góc bánh thật — góc bánh
 *                    thật = ACK_STEER_GAIN × (steer_deg − ACK_STEER_TRIM_DEG).
 *                    Giữ nguyên ngữ nghĩa "lệnh servo" cho `$ODO` để không phá
 *                    các số hiệu chuẩn đã đo bên Jetson.
 *                    (Chiều xác nhận thực nghiệm 2026-09-05: angular_z=+0.5 ->
 *                     bánh lái sang TRÁI. Khớp chuẩn ROS REP-103.)
 *
 * @note   Vi sai 2 bánh sau (2026-09-06): V_L,R = V·(1 ∓ D·tanθ/2H), nguồn
 *         `reference/2 Motion Control Course/1. Kinematics Analysis.pdf` tr.6,
 *         đã tự dẫn lại kiểm chứng. θ dùng ở đây phải là góc BÁNH THẬT.
 * @note   Góc lái theo vận tốc (2026-09-06): θ = atan(H·ω/v), thay hằng số
 *         tuyến tính `angular_z * 30` cũ (bỏ quên vận tốc, chỉ đúng tại
 *         v ≈ 0.4 m/s).
 * @note   Giới hạn lái ĐỐI XỨNG (2026-09-06): vì tâm servo lệch −4.8°, nếu
 *         clamp thẳng lệnh servo ở ±30° thì bánh quay được +20.8° khi rẽ trái
 *         nhưng chỉ −15.0° khi rẽ phải → Nav2 nhận mô hình xe bất đối xứng.
 *         Nay clamp GÓC BÁNH ở phía hẹp hơn của 2 chiều rồi mới suy ngược ra
 *         lệnh servo → lệnh servo luôn nằm trong [−30°, +20.4°], KHÔNG BAO GIỜ
 *         vượt giới hạn cơ khí ở cả 2 chiều. Giới hạn bánh hiệu dụng hiện tại
 *         lấy theo cái NHỎ HƠN giữa (a) và (b) — hiện (b) chặn, vì bánh chạm
 *         khung mới là ràng buộc thật. Bán kính cua nhỏ nhất = H/tan(giới hạn).
 */
void CALC_Ackermann(float linear_x, float angular_z,
                    int8_t *speed_l, int8_t *speed_r,
                    float *steer_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ACKERMANN_H__ */
