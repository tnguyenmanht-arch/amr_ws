#ifndef __MOTOR_PID_H__
#define __MOTOR_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bộ điều khiển PI (Tỉ lệ + Tích phân) dùng cho vòng tốc độ 1 bánh xe.
 *
 * Giải thích đơn giản (không cần biết lý thuyết điều khiển):
 * - Mỗi chu kỳ, ta so sánh "muốn chạy nhanh cỡ nào" (setpoint) với "thực tế
 *   đang chạy nhanh cỡ nào" (measured, đo từ encoder) -> ra sai số (error).
 * - Khâu P (Kp * error): phản ứng NGAY theo sai số hiện tại. Sai số càng lớn,
 *   đạp ga càng mạnh. Nếu chỉ dùng khâu này, xe sẽ luôn chạy CHẬM HƠN mục tiêu
 *   1 chút (vì khi gần tới đích, error nhỏ -> lực đạp cũng nhỏ -> không đủ
 *   thắng ma sát/tải).
 * - Khâu I (Ki * tổng dồn error theo thời gian): "nhớ" sai số còn sót lại từ
 *   khâu P và ga tăng dần cho tới khi hết sai số hẳn -> đây là phần chính giúp
 *   bù ma sát/tải không đều giữa các vòng quay, giảm giật cục.
 * - KHÔNG dùng khâu D (đạo hàm): với tín hiệu đếm tick rời rạc, đạo hàm rất dễ
 *   nhiễu (jitter 1-2 tick cũng bị khuếch đại) -> làm rung thêm chứ không giúp
 *   ích, nên bỏ hẳn cho đơn giản. Đa số firmware nhúng dùng PI là đủ mượt.
 *
 * Cách chỉnh Kp/Ki bằng thực nghiệm (không cần hiểu toán, chỉ cần quan sát):
 *   1. Đặt Ki = 0 trước. Tăng dần Kp tới khi bánh xe phản ứng nhanh với thay
 *      đổi tốc độ mà KHÔNG bị rung/dao động qua lại quanh tốc độ mục tiêu.
 *      Nếu thấy rung/kêu è è đổi chiều liên tục -> Kp đang quá cao, giảm bớt.
 *   2. Giữ nguyên Kp, tăng dần Ki từ 0 lên tới khi bánh hết "ì" (không còn
 *      chạy chậm hơn mục tiêu 1 chút khi đã ổn định). Nếu tăng Ki quá tay sẽ
 *      thấy tốc độ dao động chậm (vọt lên rồi tụt xuống theo chu kỳ ~1s) ->
 *      giảm Ki lại.
 *   3. LUÔN test với bánh xe NHẤC KHỎI MẶT ĐẤT trước, ở tốc độ thấp, để không
 *      va chạm/lật xe nếu gains sai làm motor chạy vọt tốc.
 */

typedef struct {
    float kp;
    float ki;
    float integral;   /* Sai số tích lũy (khâu I) - trạng thái nội bộ, đừng sửa tay */
    float out_min;    /* Giới hạn dưới của output (chống windup + giới hạn PWM) */
    float out_max;    /* Giới hạn trên của output (chống windup + giới hạn PWM) */
} PID_t;

/**
 * @brief  Khởi tạo bộ PID với 2 hằng số Kp/Ki và giới hạn output.
 * @param  out_min/out_max  Giới hạn output sau cùng (ví dụ -100..100 cho duty %).
 */
void PID_Init(PID_t *pid, float kp, float ki, float out_min, float out_max);

/**
 * @brief  Chạy 1 bước PID.
 * @param  setpoint  Giá trị mong muốn (cùng đơn vị với measured).
 * @param  measured  Giá trị đo được thực tế (ví dụ số tick encoder/chu kỳ).
 * @param  dt_s      Thời gian trôi qua từ lần gọi trước, tính bằng giây.
 * @retval Output đã kẹp trong [out_min, out_max].
 */
float PID_Update(PID_t *pid, float setpoint, float measured, float dt_s);

/**
 * @brief  Xóa sạch phần tích lũy (I). Gọi khi dừng hẳn hoặc reset encoder,
 *         tránh để "trí nhớ" cũ của PID làm giật khi chạy lại từ đầu.
 */
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PID_H__ */
