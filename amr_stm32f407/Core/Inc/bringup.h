#ifndef __BRINGUP_H__
#define __BRINGUP_H__

/* ===== Chế độ BRING-UP: kiểm tra board Hiwonder ROS Robot Controller lần đầu =====
 *
 * Mục đích: xác minh từng phần cứng trên board mới TRƯỚC khi chạy firmware
 * chính (PID, giao tiếp Jetson). Chạy riêng, KHÔNG bật APP_Comm, KHÔNG chạy PID.
 *
 * Cách vào: GIỮ nút K1 (PE1) trong lúc bật nguồn / bấm reset, thả ra sau khi
 * nghe 3 tiếng bíp. Hoặc đặt BRINGUP_FORCE = 1 rồi build lại.
 *
 * Output: in ASCII (không dấu) ra CẢ 2 cổng USB cùng lúc — USART1 (cổng số 7)
 * và USART3 (cổng số 4), 115200 8N1. Nhận lệnh 1 ký tự từ cổng nào cũng được.
 * Dòng banner in riêng cho từng cổng cho biết bạn đang đọc USART nào.
 *
 * ⚠️ AN TOÀN: motor/servo CHỈ chạy khi gõ lệnh. Mỗi lần test motor tự dừng
 * sau thời gian cố định; bấm phím bất kỳ trong lúc chạy cũng dừng ngay.
 * Nhấc bánh khỏi mặt đất trước khi gõ lệnh motor.
 */

#include <stdint.h>

/* 1 = luôn vào bring-up khi khởi động (không cần giữ K1) */
#define BRINGUP_FORCE   0

/** @brief  Trả về 1 nếu cần vào chế độ bring-up (K1 đang giữ hoặc BRINGUP_FORCE). */
uint8_t BRINGUP_Requested(void);

/** @brief  Vòng lặp bring-up tương tác. KHÔNG trả về (reset để thoát). */
void BRINGUP_Run(void);

#endif /* __BRINGUP_H__ */
