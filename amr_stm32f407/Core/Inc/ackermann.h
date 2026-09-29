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

/* (a) ⭐ GIỚI HẠN ĐANG DÙNG THẬT — mức an toàn do user tự chọn khi test servo.
 *
 * ĐÂY KHÔNG PHẢI ĐIỂM CHẠM. User đã xác minh bằng mắt (2026-09-06, chiều):
 * ở lệnh servo ±35° bánh vẫn còn hở khung **5mm**, cả 2 bên như nhau, chạy
 * thoải mái không sao. Điểm chạm thật nằm đâu đó XA HƠN 35° — chưa ai dò tới,
 * và cũng không cần: 35° đã đủ dùng, còn 5mm dự phòng cho rung/xóc.
 *
 * Nếu sau này muốn cua gắt hơn nữa thì nới SỐ NÀY (không phải nới (b)), và
 * phải nhìn lại khe hở bằng mắt ở giá trị mới trước khi chạy tự động.
 *
 * Bản thân servo đi được ±120° (vị trí 0..1000, xem servo_buslinker.h) nên
 * phần cứng không phải là thứ chặn ở đây. */
#define ACK_MAX_SERVO_DEG       35.0f

/* (b) Chốt chặn phụ theo GÓC BÁNH — cố ý để RỘNG, hiện KHÔNG bind.
 *
 * Lý do: giới hạn thật đang được đặt ở PHÍA SERVO (mục (a)) — đó là nơi user
 * quan sát trực tiếp được khe hở bánh↔khung. Nếu đặt giới hạn ở phía bánh thì
 * phải quy đổi qua ACK_STEER_GAIN, mà GAIN nay đã biết là CHƯA ĐÁNG TIN (fit
 * trên điểm gốc trim sai 4.8°) → giới hạn sẽ sai theo hệ số sai.
 *
 * ⚠️ Con số "bánh 18.03°" ghi ở các bản trước KHÔNG phải số đo — nó chính là
 * servo 35° quy đổi qua GAIN. Đừng dùng nó như một phép đo cơ khí.
 *
 * Giá trị dưới đây rộng hơn mức servo ±35° có thể sinh ra (35 × 0.597 ≈ 20.9°)
 * nên (a) là cái bind. Nó chỉ còn vai trò lưới an toàn phòng khi GAIN được
 * hiệu chuẩn lại thành giá trị lớn bất thường. */
#define ACK_MAX_WHEEL_DEG       25.0f

/* Ngưỡng coi như "đứng yên": dưới mức này không suy được góc lái từ (ω, v)
 * vì công thức atan(H·ω/v) chia cho 0. Xem xử lý trong ackermann.c. */
#define ACK_MIN_SPEED_MS        1e-4f

/* Bù lệch tâm cơ khí: LỆNH servo ứng với lúc bánh xe THẲNG.
 *
 * ĐO TRỰC TIẾP DƯỚI SÀN trên Jetson 2026-09-06 (chiều, sau khi nạp firmware
 * GAIN) — hoá ra **tâm servo CHÍNH LÀ vị trí bánh thẳng, không cần bù gì**.
 *
 * Cách đo (dò nhị phân, không cần nạp lại firmware mỗi vòng): chạy thẳng
 * v=0.20 m/s, cộng thêm `angular.z` bias từ ROS, đo GÓC QUAY của xe:
 *     bias 0.000 -> lệch PHẢI 11.59°   (thước: 12cm / 1.19m)
 *     bias 0.034 -> lệch PHẢI  5.82°   (thước:  6cm / 1.18m)
 *     bias 0.051 -> lệch TRÁI   1.88°  (b−a = +1cm trên khung 304mm)
 *     bias 0.068 -> lệch TRÁI rõ
 *   -> nội suy 2 điểm kẹp sát: bias đi thẳng = 0.0468 rad/s
 *   -> ở bias đó firmware XUẤT LỆNH servo = −0.08° và xe đi THẲNG
 *   => vị trí servo cho bánh thẳng = −0.08° ≈ 0. Độ nhạy ±0.3°.
 *
 * ⭐ Lập luận này KHÔNG phụ thuộc ACK_STEER_GAIN có đúng hay không: ta chỉ
 * dùng con số servo mà firmware THỰC SỰ xuất ra. GAIN chỉ ảnh hưởng cách
 * firmware tính ra nó, không ảnh hưởng kết luận về giá trị đó.
 *
 * Lịch sử 2 giá trị SAI trước đây, ghi lại để không quay lại:
 *   +1.5f  (2026-07-07) — ước lượng bằng mắt, chưa từng đo.
 *   -4.8f  (2026-09-05) — suy ra từ test lệch ngang trên firmware CŨ, rồi
 *          giải ngược qua công thức `angular_z*30 + trim`. Chuỗi suy luận đó
 *          bị nhiễu và cho kết quả lệch tận 4.8°. Bài học: suy trim qua nhiều
 *          tầng công thức thì sai số dồn — ĐO TRỰC TIẾP góc quay khi chạy
 *          thẳng, đừng suy gián tiếp.
 *
 * `steering_trim_angular_z` bên ROS giữ 0 VĨNH VIỄN — trim gộp về đúng 1 chỗ
 * duy nhất là hằng số này (trim tính bằng rad/s cho ra góc phụ thuộc tốc độ,
 * còn lệch tâm cơ khí là góc không đổi, nên về nguyên tắc không bù được ở đó).
 * CẦN ĐO LẠI nếu tháo/lắp lại servo hoặc tay đòn lái. */
#define ACK_STEER_TRIM_DEG      0.0f

/* Tỉ số truyền tay đòn lái: góc_bánh_thật = GAIN × (steer_deg − TRIM).
 *
 * ĐO LẠI 2026-09-07 trên firmware đã có TRIM=0.0f (điểm gốc ĐÚNG), bằng test
 * vòng tròn bẻ hết lái, đo đường kính ở TÂM TRỤC SAU rồi trừ đoạn mồi ~0.2m:
 *
 *   chiều   đường kính   bán kính   góc bánh thực   GAIN
 *   TRÁI      1.19 m      0.578 m      19.96°       0.570
 *   TRÁI      1.15 m      0.558 m      20.64°       0.590   (lặp lại)
 *   PHẢI      1.23 m      0.599 m      19.33°       0.552
 *
 *   -> GAIN_L ≈ 0.580 ,  GAIN_R ≈ 0.552 ,  chốt TRUNG BÌNH = 0.566
 *
 * Sai số còn lại sau khi chốt 0.566: trái −2.4%, phải +2.5% (trước đó, với
 * 0.597: trái +2.9%, phải +8.1% — tệ nhất giảm từ 8.1% xuống 2.5%).
 *
 * ⚠️ BẤT ĐỐI XỨNG TRÁI/PHẢI ~5% — CÓ THẬT nhưng CHƯA tách thành 2 hằng số.
 * Một mình số liệu thước chưa đủ chắc: nhiễu 1 phép đo ±0.018 (đo lặp 2 lần
 * chiều trái), hiệu đo được 0.028 → chỉ 1.3 sai số chuẩn. Điều làm tin được
 * là BA quan sát ĐỘC LẬP cùng chỉ một hướng, cùng cỡ 5-6%:
 *   (1) cơ khí — bánh trái sát khung hơn ở cực trái (tầm lái trái xa hơn)
 *   (2) /odom  — tốc độ xoay phải 18.8°/s vs trái 20.0°/s (chênh 6%)
 *   (3) thước  — GAIN phải thấp hơn trái 5%
 * Muốn tách GAIN_L/GAIN_R phải đo thêm vài lần mỗi bên (nhiễu đang lớn so với
 * hiệu ứng). Lợi ích nhỏ: 0.566 đã kéo cả hai bên về trong ±2.5%, thừa sức cho
 * scan matching của slam_toolbox. Để lại làm sau nếu SLAM cho thấy cần.
 *
 * ⚠️ Hệ số HIỆU DỤNG: đã gộp cả trượt lốp bánh trước (khung này lái SONG SONG,
 * không phải Ackermann thật), đo trên sàn cứng với lốp hiện tại. Đổi mặt sàn
 * (thảm) hoặc thay lốp thì phải đo lại. */
#define ACK_STEER_GAIN          0.566f

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
