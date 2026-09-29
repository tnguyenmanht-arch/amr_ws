#ifndef __DWT_LOG_H__
#define __DWT_LOG_H__

/* ===== Hạ tầng đo timing bằng DWT CYCCNT (branch baseline-dwt) =====
 *
 * Mục đích: ĐO superloop hiện tại, KHÔNG thay đổi hành vi điều khiển.
 * - DWT CYCCNT đếm theo SYSCLK = 168 MHz (F407: HSE 8MHz / PLLM 8 * PLLN 336 / PLLP 2)
 *   -> 1 tick ≈ 5.95 ns, tràn sau ~25.6 s (script phân tích trừ theo modulo 2^32,
 *   chạy với --cpu-hz 168e6).
 * - Bản ghi được ghi vào RAM (log_buf), KHÔNG printf / KHÔNG gửi UART lúc đo,
 *   để tránh chính việc đo làm sai kết quả (probe effect).
 * - Buffer KHÔNG ghi đè: đầy thì ngừng ghi, giữ nguyên dữ liệu đầu phiên đo.
 * - Chỉ gọi DWT_Log() từ main loop (không gọi trong ISR) -> không cần khóa.
 *
 * Đọc dữ liệu ra: dump vùng RAM của log_buf qua ST-Link (xem
 * scripts/BASELINE_DWT.md), rồi chạy scripts/analyze_baseline.py.
 */

#include <stdint.h>
#include "stm32f4xx.h"

/* Mã sự kiện. 0 dành riêng = "ô trống" (RAM .bss khởi tạo 0) để script nhận
 * biết chỗ dữ liệu kết thúc. PHẢI khớp bảng EVENT_NAMES trong
 * scripts/analyze_baseline.py. */
typedef enum {
    EV_NONE    = 0,
    EV_PID     = 1,  /* Thân vòng PID (sau đoạn gate thời gian) */
    EV_ODO     = 2,  /* snprintf + gửi polling 1 khung $ODO; len = số byte */
    EV_IMUH    = 3,  /* Gửi polling 1 khung $IMUH;             len = số byte */
    EV_DUMMY   = 4,  /* Bản ghi rỗng: chính là lệnh ghi log bị đo chi phí */
    EV_LOG_OVH = 5,  /* Khoảng thời gian bao trọn 1 lần ghi EV_DUMMY
                      * = chi phí 1 lần DWT_Log() + 1 lần DWT_Now() */
} DWT_EventId;

/* 1 bản ghi = 12 byte, sắp xếp sẵn để KHÔNG có padding:
 *   offset 0 : t_start (u32)
 *   offset 4 : t_end   (u32)
 *   offset 8 : len     (u16)
 *   offset 10: id      (u8)
 *   offset 11: reserved(u8, luôn 0)
 * Script Python đọc bằng struct "<IIHBB" (little-endian, 12 byte). */
typedef struct {
    uint32_t t_start;
    uint32_t t_end;
    uint16_t len;
    uint8_t  id;
    uint8_t  reserved;
} LogRecord;

_Static_assert(sizeof(LogRecord) == 12, "LogRecord phai dung 12 byte (khop script Python)");

/* Dung lượng buffer. 5000 x 12 = 60 KB (F411 có 128 KB RAM).
 * Ước lượng tốc độ ghi: PID 100/s + ODO 100/s + IMUH ~7/s ≈ 207 bản ghi/s,
 * cộng 2 x DWT_OVH_SAMPLES bản ghi đo overhead
 * -> ~22 s dữ liệu, đủ cho phiên đo 15–20 s. */
#define DWT_LOG_CAPACITY   5000u

/* Số lần đo overhead ghi log (mỗi lần tốn 2 bản ghi), chỉ làm ở đầu phiên. */
#define DWT_OVH_SAMPLES    200u

/* Biến toàn cục (KHÔNG static) để tìm được địa chỉ trong file .map và đọc
 * trực tiếp qua ST-Link. */
extern LogRecord         log_buf[DWT_LOG_CAPACITY];
extern volatile uint32_t log_count;    /* Số bản ghi hợp lệ trong log_buf */
extern volatile uint32_t log_active;   /* Trạng thái phiên đo, xem DWT_LOG_* bên dưới */

/* Trạng thái phiên đo -- chỉ đi 1 chiều IDLE -> RUN -> DONE, không quay lại,
 * để 1 lần nạp/reset = đúng 1 phiên đo liền mạch. */
#define DWT_LOG_IDLE   0u   /* Chưa bắt đầu (chờ $VEL đầu tiên) */
#define DWT_LOG_RUN    1u   /* Đang ghi */
#define DWT_LOG_DONE   2u   /* Đã dừng (watchdog $VEL trip = script ngừng gửi) */

/* Bật bộ đếm chu kỳ DWT CYCCNT. Gọi 1 lần sau SystemClock_Config(). */
void DWT_Init(void);

/* IDLE -> RUN. Gọi ở trạng thái khác không có tác dụng. */
void DWT_LogStart(void);

/* RUN -> DONE. Sau đó log_buf đứng yên, dump an toàn. */
void DWT_LogStop(void);

/* Ghi 1 bản ghi; bỏ qua nếu chưa bật hoặc buffer đã đầy. */
void DWT_Log(uint8_t id, uint32_t t_start, uint32_t t_end, uint16_t len);

/* Đọc giá trị bộ đếm chu kỳ hiện tại (1 tick = 10 ns ở 100 MHz). */
static inline uint32_t DWT_Now(void)
{
    return DWT->CYCCNT;
}

#endif /* __DWT_LOG_H__ */
