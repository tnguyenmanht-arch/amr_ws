#include "dwt_log.h"

/* Nằm trong .bss -> startup code xóa về 0, nên id = EV_NONE ở mọi ô chưa ghi. */
LogRecord         log_buf[DWT_LOG_CAPACITY];
volatile uint32_t log_count  = 0;
volatile uint32_t log_active = DWT_LOG_IDLE;

void DWT_Init(void)
{
    /* Bật khối trace (TRCENA) -- bắt buộc trước khi DWT hoạt động được */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_LogStart(void)
{
    if (log_active == DWT_LOG_IDLE) {
        log_active = DWT_LOG_RUN;
    }
}

void DWT_LogStop(void)
{
    if (log_active == DWT_LOG_RUN) {
        log_active = DWT_LOG_DONE;
    }
}

void DWT_Log(uint8_t id, uint32_t t_start, uint32_t t_end, uint16_t len)
{
    /* Không ở trạng thái RUN, hoặc đã đầy -> ngừng ghi (KHÔNG ghi đè dữ liệu cũ) */
    if (log_active != DWT_LOG_RUN || log_count >= DWT_LOG_CAPACITY) {
        return;
    }
    LogRecord *r = &log_buf[log_count];
    r->t_start  = t_start;
    r->t_end    = t_end;
    r->len      = len;
    r->id       = id;
    r->reserved = 0u;
    log_count++;   /* Tăng SAU khi ghi xong -> đọc qua ST-Link không thấy bản ghi dở */
}
