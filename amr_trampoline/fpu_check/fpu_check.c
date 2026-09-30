#include "tpl_os.h"

/* Bien volatile de xem qua SWD/debugger khi co board:
 *   slow_err_count phai luon = 0 (ngu canh FPU cua slow duoc bao toan
 *   du bi fast cat ngang 1 lan/ms). */
volatile uint32_t slow_iter = 0;
volatile uint32_t slow_err_count = 0;
volatile float    fast_acc = 0.0f;

#define APP_Task_slow_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(slow)
{
  /* Vong lap dai giu gia tri trong thanh ghi s0-s31; ket qua biet truoc. */
  for (;;) {
    float a = 1.0f, b = 0.0f;
    for (int i = 0; i < 1000; i++) {
      a = a * 1.0001f;
      b = b + 0.5f;
    }
    if (b != 500.0f || a < 1.105f || a > 1.106f) {
      slow_err_count++;
    }
    slow_iter++;
  }
}
#define APP_Task_slow_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_fast_START_SEC_CODE
#include "tpl_memmap.h"
TASK(fast)
{
  /* Pha cac thanh ghi FP de lo loi neu kernel khong luu ngu canh cua slow */
  float x = fast_acc;
  for (int i = 0; i < 16; i++) {
    x = x * 0.5f + 123.0f;
  }
  fast_acc = x;
  TerminateTask();
}
#define APP_Task_fast_STOP_SEC_CODE
#include "tpl_memmap.h"
