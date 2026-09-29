# Checklist bring-up — Hiwonder ROS Robot Controller (F407)

Firmware: `amr_stm32f407/` (branch `f407-port`), build sẵn ở
`amr_stm32f407/Debug/amr_stm32f407.elf`. Làm **đúng thứ tự**, mỗi bước chỉ
thêm một thứ mới. Ghi kết quả vào cột cuối rồi gửi lại cho Claude để điền hằng số.

**Quy tắc chung:** cắm/rút cáp khi công tắc 20 ở OFF · laptop rút sạc ·
chỉ một máy nối vào board · motor chỉ quay khi đã nhấc bánh khỏi mặt đất.

Đọc serial trên Windows (pyserial có sẵn):
`python -m serial.tools.miniterm COMx 115200` (thoát bằng Ctrl+]).

## A. Chưa cấp pin, chưa nạp code

| # | Việc | Kỳ vọng | Kết quả |
|---|---|---|---|
| A1 | Xem silkscreen mặt sau | V1.0 hoặc V1.1 | |
| A2 | Nếu V1.0: có cầu nối VREF+/VDDA (chân 21/22 của chip)? | Có | |
| A3 | Ký hiệu cầu chì cạnh 4 driver motor | 2 A (schematic) hay 1.5 A (tài liệu Explanation) | |
| A4 | Dây motor JGB37-520 cắm vừa header 6 chân 2.0 mm? Thứ tự dây khớp M-, GND, A, B, 5V, M+? | Khớp | |

## B. Chỉ cắm USB (công tắc 20 OFF, chưa cắm motor/servo)

| # | Việc | Kỳ vọng | Kết quả |
|---|---|---|---|
| B1 | Cắm USB cổng số 7 → đo 5V, 3V3 trên header H1 | ~5 V / 3.3 V | |
| B2 | Đo **VIN** (terminal 21 hoặc chân 2 cổng servo) khi chỉ có USB | ~0 V. Nếu vài V → rail VIN bị nuôi ngược từ USB, **không** cắm servo/motor lúc chỉ có USB | |
| B3 | Nạp qua ST-Link ở H1: chân 7 = SWDIO (PA13), chân 3 = SWCLK (PA14), GND. **Không** nối 3.3V của ST-Link | `STM32_Programmer_CLI -c port=SWD -w ...elf -v -rst` báo thành công | |
| B4 | Sau reset | 1 tiếng bíp + LED user (PE10) sáng | |
| B5 | Mở miniterm ở cổng COM của **cổng số 4** | Thấy `$ODO,0,0,0.0` chạy liên tục | |

> Không có ST-Link? Cổng số 7 có mạch tự nạp qua bootloader (USART1). Thử
> `STM32_Programmer_CLI -c port=COMx br=115200` — chưa kiểm chứng, báo lại
> Claude nếu cần.

## C. Chế độ bring-up (giữ **K1** khi bấm reset, nghe 3 tiếng bíp)

| # | Việc | Kỳ vọng | Kết quả |
|---|---|---|---|
| C1 | Mở miniterm ở từng cổng, đọc dòng `>>> Ban dang doc ...` | Cổng số 4 = USART3, cổng số 7 = USART1 | |
| C2 | Gạt công tắc số 1 (enable) hai phía, xem `PD3(EN_SW)=` | Đổi 0 ↔ 1. Ghi: **phía nào = 0** | |
| C3 | Bấm K1/K2, xem `K1=`/`K2=` | 1 → 0 khi nhấn | |
| C4 | Gõ `b` | Bíp (xác nhận buzzer ở **PA8**) | |
| C5 | Gõ `i` | Thấy 0x68, `WHO_AM_I = 0x68 -> OK` | |
| C6 | Cắm motor (vẫn chưa bật pin). **Xoay tay** bánh trái theo chiều **TIẾN**, xem `ENC L(TIM5)` | Số thay đổi. Ghi: **tăng hay giảm** | |
| C7 | Tương tự bánh phải, xem `ENC R(TIM2)` | Ghi: **tăng hay giảm** | |

## D. Bật pin (công tắc 20 ON) — **nhấc bánh khỏi mặt đất**

| # | Việc | Kỳ vọng | Kết quả |
|---|---|---|---|
| D1 | So `VBAT ~x.xxV` với đồng hồ đo ở terminal 21 | Lệch < 0.2 V | |
| D2 | Gõ `1` (trái, FWD 25%, 1 s) | Ghi: bánh **nào** quay, **tiến/lùi**, `dENC` | |
| D3 | Gõ `3` (phải, FWD 25%, 1 s) | Ghi như trên | |
| D4 | Sau khi dừng: bánh **dừng sững** hay **quay trôi**? | (bảng chân lý YX-4055AM) | |
| D5 | (Tuỳ chọn) đồng hồ đo dòng nối tiếp dây pin, gõ `m` | Ghi `tick/10ms` trái/phải + dòng lớn nhất | |
| D6 | Đo chân 2 của **cả hai** cổng bus servo | ≈ điện áp pin (VIN) | |
| D7 | Cắm HTS-20H (ID=1) vào cổng có VIN, gõ `s` | Servo quay giữa → +15° → −15° → giữa | |
| D8 | Nếu D7 im lặng: gõ `p` rồi `s` lại | Ghi: cực tính nào chạy | |

## Sau khi xong

Gửi bảng kết quả cho Claude để điền vào `motor_driver.c` (`LEFT/RIGHT_CH_FWD/REV`,
`LEFT/RIGHT_ENCODER_SIGN`, `MAX_TICKS_PER_INTERVAL`), `main.c` (`EN_SW_USE`,
`EN_SW_RUN_LEVEL`) và cập nhật CLAUDE.md. Chỉ sau đó mới test PID bằng
`scripts/send_vel_loop.py` (vẫn nhấc bánh), rồi mới đo baseline DWT.
