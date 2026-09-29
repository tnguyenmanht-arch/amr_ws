#include "bringup.h"
#include "main.h"
#include "motor_driver.h"
#include "servo_buslinker.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ===== Thông số test (chỉnh ở đây nếu cần) ===== */
#define BU_TEST_DUTY        25      /* % duty khi test chiều từng bánh */
#define BU_TEST_MS          1000u   /* thời gian quay mỗi lần test chiều */
#define BU_MAX_DUTY         100     /* % duty khi đo tốc độ tối đa */
#define BU_MAX_WARMUP_MS    1500u   /* bỏ qua đoạn tăng tốc */
#define BU_MAX_MEASURE_MS   1000u   /* cửa sổ đo sau khi đã ổn định */
#define BU_STATUS_MS        500u    /* chu kỳ in dòng trạng thái */
#define BU_SERVO_TEST_DEG   15.0f   /* biên độ quét servo khi test */

/* Cầu chia điện áp pin (schematic V1.1 trang 3): VIN -> R27 100k -> PB0 -> R26 10k -> GND
 * => V_pin = V_adc * (100k + 10k) / 10k = V_adc * 11. ⚠️ Đối chiếu với đồng hồ đo. */
#define BU_VBAT_RATIO       11.0f
#define BU_VREF             3.3f

#define MPU6050_ADDR        0x68    /* AD0 kéo xuống GND qua R19 */
#define MPU6050_WHO_AM_I    0x75    /* thanh ghi WHO_AM_I, giá trị đúng = 0x68 */

static uint8_t servo_buf_active_low = 1;

/* ------------------------------------------------------------------ */
/* In ra cả USART1 và USART3 bằng thanh ghi (polling), không qua HAL.   */
/* ------------------------------------------------------------------ */
static void tx_raw(USART_TypeDef *u, const char *s)
{
    while (*s) {
        while (!(u->SR & USART_SR_TXE)) { }
        u->DR = (uint16_t)(uint8_t)*s++;
    }
    while (!(u->SR & USART_SR_TC)) { }
}

static void out(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    tx_raw(USART1, buf);
    tx_raw(USART3, buf);
}

/* Đọc 1 ký tự từ cổng nào có dữ liệu trước; trả về 0 nếu không có.
 * Đọc SR rồi DR cũng xóa luôn cờ lỗi ORE/FE nếu có. */
static char rx_char(void)
{
    USART_TypeDef *ports[2] = { USART1, USART3 };
    for (int i = 0; i < 2; i++) {
        uint32_t sr = ports[i]->SR;
        if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE)) {
            char c = (char)(ports[i]->DR & 0xFF);
            if (sr & USART_SR_RXNE) return c;
        }
    }
    return 0;
}

static void beep(uint32_t ms)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
/* Đọc cảm biến                                                        */
/* ------------------------------------------------------------------ */
static uint32_t read_vbat_raw(void)
{
    uint32_t raw = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return raw;
}

static void print_status(void)
{
    int32_t el = 0, er = 0;
    DRV_Motor_GetEncoder(&el, &er);
    uint32_t raw = read_vbat_raw();
    float vbat = (float)raw * BU_VREF / 4095.0f * BU_VBAT_RATIO;

    out("[ST] PD3(EN_SW)=%d K1(PE1)=%d K2(PE0)=%d MPU_INT(PB12)=%d | VBAT raw=%lu ~%.2fV | ENC L(TIM5)=%ld R(TIM2)=%ld\r\n",
        (int)HAL_GPIO_ReadPin(EN_SW_GPIO_Port, EN_SW_Pin),
        (int)HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin),
        (int)HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin),
        (int)HAL_GPIO_ReadPin(MPU_INT_GPIO_Port, MPU_INT_Pin),
        (unsigned long)raw, (double)vbat, (long)el, (long)er);
}

static void i2c_scan(void)
{
    out("[I2C2] Quet dia chi 0x08..0x77 (PB10=SCL, PB11=SDA):\r\n");
    int found = 0;
    for (uint16_t a = 0x08; a <= 0x77; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(a << 1), 2, 5) == HAL_OK) {
            out("   thay thiet bi o 0x%02X%s\r\n", a,
                a == MPU6050_ADDR ? " (MPU-6050?)" :
                (a == 0x28 || a == 0x29) ? " (BNO055?)" : "");
            found++;
        }
    }
    if (!found) out("   KHONG thay thiet bi nao -> kiem tra nguon 3V3 / pull-up\r\n");

    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR << 1, MPU6050_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who, 1, 20) == HAL_OK) {
        out("   MPU-6050 WHO_AM_I = 0x%02X (dung = 0x68) -> %s\r\n",
            who, who == 0x68 ? "OK" : "SAI");
    } else {
        out("   Doc WHO_AM_I that bai\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* Chờ ms mili-giây; trả về 1 nếu người dùng bấm phím (để dừng sớm).   */
/* ------------------------------------------------------------------ */
static uint8_t wait_or_abort(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        if (rx_char()) return 1;
    }
    return 0;
}

/* Quay 1 bánh open-loop, in delta encoder cả 2 bánh để xác định:
 *  (1) bánh nào nối cổng nào, (2) chiều FWD/REV, (3) dấu encoder. */
static void motor_dir_test(uint8_t right_wheel, int8_t duty)
{
    int32_t l0, r0, l1, r1;
    out("[MOTOR] Banh %s, duty %+d%% (kenh %s), %lu ms. Bam phim bat ky de DUNG.\r\n",
        right_wheel ? "PHAI (cong M2, PE9/PE11)" : "TRAI (cong M1, PE13/PE14)",
        duty, duty > 0 ? "FWD" : "REV", (unsigned long)BU_TEST_MS);
    DRV_Motor_GetEncoder(&l0, &r0);
    DRV_Motor_SetDutyRaw(right_wheel ? 0 : duty, right_wheel ? duty : 0);
    uint8_t aborted = wait_or_abort(BU_TEST_MS);
    DRV_Motor_SetDutyRaw(0, 0);
    DRV_Motor_GetEncoder(&l1, &r1);
    out("[MOTOR] %s. dENC trai=%ld phai=%ld\r\n",
        aborted ? "DA DUNG (bam phim)" : "Xong",
        (long)(l1 - l0), (long)(r1 - r0));
    out("        -> Ghi lai: banh nao quay, quay TIEN hay LUI, dENC duong hay am.\r\n");
    out("        -> Sau khi ve 0: banh dung SUNG (phanh) hay QUAY TROI (tha troi)?\r\n");
}

/* Đo tốc độ tối đa -> MAX_TICKS_PER_INTERVAL (tick / 10 ms) cho motor_driver.c */
static void motor_max_test(void)
{
    int32_t l0, r0, l1, r1;
    out("[MAX] Ca 2 banh %d%% FWD: %lu ms tang toc + %lu ms do. Bam phim de DUNG.\r\n",
        BU_MAX_DUTY, (unsigned long)BU_MAX_WARMUP_MS, (unsigned long)BU_MAX_MEASURE_MS);
    DRV_Motor_SetDutyRaw(BU_MAX_DUTY, BU_MAX_DUTY);
    if (wait_or_abort(BU_MAX_WARMUP_MS)) {
        DRV_Motor_SetDutyRaw(0, 0);
        out("[MAX] DA DUNG\r\n");
        return;
    }
    DRV_Motor_GetEncoder(&l0, &r0);
    uint8_t aborted = wait_or_abort(BU_MAX_MEASURE_MS);
    DRV_Motor_GetEncoder(&l1, &r1);
    DRV_Motor_SetDutyRaw(0, 0);
    if (aborted) {
        out("[MAX] DA DUNG\r\n");
        return;
    }
    float per10_l = (float)(l1 - l0) * 10.0f / (float)BU_MAX_MEASURE_MS;
    float per10_r = (float)(r1 - r0) * 10.0f / (float)BU_MAX_MEASURE_MS;
    out("[MAX] tick/10ms: trai=%.1f phai=%.1f (F411+DRV8871 cu: 68.7 / 70.0)\r\n",
        (double)per10_l, (double)per10_r);
    out("      -> MAX_TICKS_PER_INTERVAL = |gia tri NHO hon| (chi dung khi dau encoder da dung)\r\n");
}

static void servo_test(void)
{
    out("[SERVO] ID=%d, OE %s. Giua -> +%.0f -> -%.0f -> giua (moi buoc 0.7s)\r\n",
        SERVO_ID, servo_buf_active_low ? "tich cuc THAP" : "tich cuc CAO",
        (double)BU_SERVO_TEST_DEG, (double)BU_SERVO_TEST_DEG);
    const float seq[4] = { 0.0f, BU_SERVO_TEST_DEG, -BU_SERVO_TEST_DEG, 0.0f };
    for (int i = 0; i < 4; i++) {
        HAL_StatusTypeDef st = DRV_Servo_SetAngle(seq[i]);
        out("   goc %+.0f -> UART %s\r\n", (double)seq[i], st == HAL_OK ? "OK" : "LOI");
        if (wait_or_abort(700)) { out("[SERVO] DA DUNG\r\n"); return; }
    }
    out("   Servo khong nhuc nhich? Kiem tra: nguon VIN o cong, SERVO_ID, thu lenh 'p'.\r\n");
}

static void help(void)
{
    out("\r\n===== BRING-UP Hiwonder ROS Robot Controller (F407) =====\r\n");
    out(" h : tro giup          v : bat/tat dong trang thai [ST]\r\n");
    out(" b : bip               i : quet I2C2 + doc MPU-6050\r\n");
    out(" s : quet servo        p : doi cuc tinh OE bo dem servo\r\n");
    out(" 1 : trai TIEN %d%%    2 : trai LUI    3 : phai TIEN   4 : phai LUI\r\n", BU_TEST_DUTY);
    out(" m : do toc do toi da (ca 2 banh %d%%)\r\n", BU_MAX_DUTY);
    out(" x / phim cach : dung moi motor\r\n");
    out(" !! Nhac banh khoi mat dat truoc khi dung lenh motor (1-4, m) !!\r\n\r\n");
}

uint8_t BRINGUP_Requested(void)
{
#if BRINGUP_FORCE
    return 1;
#else
    /* K1 (PE1) kéo lên 10k, nhấn = mức THẤP (schematic V1.1 trang 1) */
    return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET;
#endif
}

void BRINGUP_Run(void)
{
    DRV_Motor_SetDutyRaw(0, 0);

    /* 3 tiếng bíp = đã vào bring-up */
    for (int i = 0; i < 3; i++) { beep(60); HAL_Delay(90); }

    /* Banner riêng cho từng cổng -> biết ngay cổng USB nào nối USART nào */
    tx_raw(USART1, "\r\n>>> Ban dang doc USART1 (PA9/PA10) = cong USB so 7 (serial 1/download)\r\n");
    tx_raw(USART3, "\r\n>>> Ban dang doc USART3 (PD8/PD9) = cong USB so 4 (serial 2)\r\n");
    help();

    uint8_t  status_on = 1;
    uint32_t last_status = 0;
    uint32_t last_led = 0;

    while (1) {
        uint32_t now = HAL_GetTick();

        /* LED nháy chậm = đang ở bring-up (PE10 tích cực THẤP) */
        if (now - last_led >= 500u) {
            last_led = now;
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
        if (status_on && now - last_status >= BU_STATUS_MS) {
            last_status = now;
            print_status();
        }

        char c = rx_char();
        switch (c) {
        case 0:   break;
        case 'h': help(); break;
        case 'v': status_on = !status_on; break;
        case 'b': beep(150); break;
        case 'i': i2c_scan(); break;
        case 's': servo_test(); break;
        case 'p':
            servo_buf_active_low = !servo_buf_active_low;
            DRV_Servo_SetBufferActiveLow(servo_buf_active_low);
            out("[SERVO] OE bo dem gio la tich cuc %s. Thu lai 's'.\r\n",
                servo_buf_active_low ? "THAP (mac dinh)" : "CAO");
            break;
        case '1': motor_dir_test(0,  BU_TEST_DUTY); break;
        case '2': motor_dir_test(0, -BU_TEST_DUTY); break;
        case '3': motor_dir_test(1,  BU_TEST_DUTY); break;
        case '4': motor_dir_test(1, -BU_TEST_DUTY); break;
        case 'm': motor_max_test(); break;
        case 'x':
        case ' ':
            DRV_Motor_SetDutyRaw(0, 0);
            out("[STOP] Moi motor = 0\r\n");
            break;
        case '\r':
        case '\n':
            break;
        default:
            DRV_Motor_SetDutyRaw(0, 0);   /* phím lạ: dừng cho chắc */
            out("? lenh '%c' khong co. Go 'h'.\r\n", c);
            break;
        }
    }
}
