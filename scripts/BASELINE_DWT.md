# Đo baseline superloop bằng DWT (branch `baseline-dwt`)

Firmware ghi timing vào RAM (`log_buf`), không gửi gì thêm qua UART lúc đo.
Phiên đo **tự bắt đầu** khi nhận `$VEL` hợp lệ đầu tiên và **tự dừng** khi
watchdog `$VEL` trip (300 ms sau lệnh cuối) hoặc buffer đầy (5000 bản ghi ≈ 22 s).
Mỗi lần reset/nạp = đúng 1 phiên đo.

## Quy trình

```bash
# 0. (sau khi được xác nhận) nạp firmware Debug
STM32_Programmer_CLI.exe -c port=SWD -w "c:/Users/admin/Documents/amr_ws/amr_stm32f411/Debug/amr_stm32f411.elf" -v -rst

# 1. Nhấc bánh khỏi mặt đất, rồi gửi $VEL mỗi 100 ms trong 20 s (script hỏi xác nhận)
python scripts/send_vel_loop.py --port COM9 --linear 0.2 --duration 20

# 2. Dump RAM (không dừng chip) -> in ra log_count và trạng thái log_active
python scripts/dump_baseline.py -o runs/baseline_A.bin

# 3. Phân tích + vẽ histogram (PNG lưu cạnh file .bin)
python scripts/analyze_baseline.py runs/baseline_A.bin --count <log_count>
```

`dump_baseline.py` phải báo `log_active = 2 (DONE)`. Nếu là `1 (RUN)` thì
phiên đo chưa kết thúc. Nếu `send_vel_loop.py` báo khoảng gửi > 300 ms thì
watchdog đã trip giữa chừng, phiên bị cắt sớm, cần đo lại.

## Làm tay (nếu script dump lỗi)

**Tìm địa chỉ** trong `amr_stm32f411/Debug/amr_stm32f411.map`. Địa chỉ đổi theo
từng lần build, nên phải dùng đúng file .map của bản đang chạy trên chip:

```
 .bss.log_buf   0x200001f0     0xea60 ./Core/Src/dwt_log.o      <- địa chỉ, kích thước (60000 B)
 .bss.log_count 0x2000ec50        0x4 ./Core/Src/dwt_log.o
```

**Đọc biến và dump** (bắt buộc `mode=HOTPLUG`: mode mặc định halt core và có
thể đọc ra toàn số 0, xem CLAUDE.md lỗi #11):

```bash
STM32_Programmer_CLI.exe -c port=SWD mode=HOTPLUG -r32 0x2000ec50 4           # log_count
STM32_Programmer_CLI.exe -c port=SWD mode=HOTPLUG -u 0x200001f0 60000 log.bin # log_buf -> file
```

**Hoặc CubeIDE** (khi đang debug): Window → Show View → Memory → thêm địa chỉ
`&log_buf` → nút *Export* → định dạng **RAW Binary**, độ dài 60000 byte. Lưu ý
phiên debug sẽ halt chip, nên chỉ export sau khi phiên đo đã DONE.

## Định dạng bản ghi (12 byte, little-endian, không padding)

| offset | trường | kiểu | ý nghĩa |
|---|---|---|---|
| 0 | t_start | u32 | DWT CYCCNT lúc bắt đầu (10 ns/tick) |
| 4 | t_end | u32 | DWT CYCCNT lúc kết thúc |
| 8 | len | u16 | độ dài khung (ODO/IMUH), 0 với sự kiện khác |
| 10 | id | u8 | 1 PID, 2 ODO, 3 IMUH, 4 DUMMY, 5 LOG_OVH (0 = ô trống) |
| 11 | reserved | u8 | luôn 0 |

## Lưu ý khi đọc kết quả

- Bản Debug build với `-O0`, và đây là bản đang chạy trên xe. Đo bản Release
  (`-Os`) sẽ ra số khác, nên báo cáo phải ghi rõ đo bản nào.
- `EV_ODO` gồm cả `snprintf` lẫn gửi polling. Script tách phần dư so với lý
  thuyết đường dây (86.8 µs/byte ở 115200 8N1).
- `EV_IMUH` chỉ đo phần gửi, **không** gồm đọc I2C BNO055. Phần đọc I2C cũng
  chặn CPU; nếu cần thì thêm một sự kiện riêng.
- Mỗi sự kiện được đo có thêm chi phí khoảng 1 lần `DWT_Log`. Con số này nằm ở
  mục `[Overhead đo]`.
