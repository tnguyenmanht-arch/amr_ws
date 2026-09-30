# Trampoline (OSEK) trên Windows: cài đặt và build

Dùng cho ĐATN (so sánh superloop và OSEK trên F407). Tất cả chỉ là **build**,
chưa nạp vào board.

## 1. Thành phần

| Thành phần | Vị trí | Ghi chú |
|---|---|---|
| Trampoline | `Documents/trampoline` (cạnh `amr_ws`), branch **`amr-f407`** | upstream `ff28702` + 2 bản vá trong `amr_trampoline/patches/` |
| goil 3.1.16 | `trampoline/goil/makefile-unix/goil.exe` | tự build từ source |
| GCC cho host | WinLibs GCC 16.1, `C:\winlibs\...\mingw64\bin` | goil cần C++17 (MinGW 6.3 cũ không build được) và DLL runtime của nó |
| GCC cho ARM | GNU Tools for STM32 14.3 (có sẵn trong CubeIDE 2.1.1) | dùng chung với `amr_stm32f407` |

## 2. Cài lại từ đầu (nếu mất bản clone)

```bash
cd ~/Documents
git clone https://github.com/TrampolineRTOS/trampoline.git
cd trampoline
git checkout -b amr-f407 ff28702
git submodule update --init --depth 1 machines/cortex-m/CMSIS_5   # BẮT BUỘC, thiếu thì lỗi core_cm4.h
git am ../amr_ws/amr_trampoline/patches/*.patch
# build goil (khoảng 2.5 phút), dùng GCC của WinLibs:
PATH="/c/winlibs/<thư mục winlibs>/mingw64/bin:$PATH" python goil/makefile-unix/build.py all
```

## 3. Build một app

```bash
scripts/trampoline_build.sh amr_trampoline/fpu_check
```

Script đặt PATH, chạy goil, rồi chạy `make.py`. Kết quả là `<app>_exe` (ELF) và `<app>_exe.bin`.
Mỗi app mới cần thêm dòng `<app>/<app>/` vào `amr_trampoline/.gitignore` (thư mục code do goil sinh).

## 4. Các bản vá so với upstream (lý do)

1. **`make.py` dùng `--target=` thay cho `-t=`**: khi make.py gọi goil từ Python, bản goil Windows bỏ qua `-t=`, báo "No target platform given".
2. **Startup stm32f407 bỏ lời gọi `_exit()`**: `script.ld` định nghĩa `_exit = .` (chỉ là một địa chỉ, không phải hàm). ld mới trong toolchain ST 14.3 từ chối lệnh `BL` tới symbol đó. `StartOS()` không bao giờ trả về nên dòng này vốn không chạy tới.
3. **Hard float + lưu ngữ cảnh FPU** (chép cách làm của port `stm32l432`):
   - `-mfloat-abi=hard`.
   - Template `process_specific` sinh `arm_float_context` cho task/ISR có `USEFLOAT = TRUE`.
   - Startup tắt ASPEN/LSPEN, bật CP10/11.

   Lý do: bản upstream của port F407 là **soft-float**. Superloop `amr_stm32f407` thì hard-float, và PID/Ackermann dùng float rất nhiều. Nếu hai bên khác float ABI thì phép so sánh jitter mất ý nghĩa.

## 5. Quy tắc khi viết app OSEK

- Task nào dùng float (PID, Ackermann, odometry) phải khai **`USEFLOAT = TRUE`**. Nếu không, task ưu tiên cao hơn cắt ngang sẽ ghi đè thanh ghi FPU của nó.
- **ISR không được dùng float**, vì đã tắt tự động lưu FPU khi vào ngắt. Mỗi lần thêm code vào (nhất là HAL) thì kiểm tra lại:
  ```bash
  arm-none-eabi-objdump -d <app>_exe | awk '/^[0-9a-f]+ <.*>:$/{fn=$2} /\tv[a-z]+/{c[fn]++} END{for(f in c) print c[f], f}'
  ```
  Chỉ được phép thấy các hàm task và `tpl_save/load_context*`.
- `fpu_check` (2026-09-30): đạt kiểm tra trên. VFP chỉ nằm trong `slow`/`fast` và 4 hàm chuyển ngữ cảnh.

## 6. Việc tiếp theo

- Thêm board Hiwonder: thay BSP Discovery (LED PE10, buzzer PA8), bỏ StdPeriph, ghép HAL.
- Cần xử lý khi ghép HAL:
  - Header CMSIS bị trùng giữa Trampoline và HAL.
  - `HAL_GetTick()` khi SysTick đã thuộc về OS.
  - Chỉ cấu hình clock ở một chỗ.
- OIL cho 4 task (PID, ODO, comm, servo), port logic từ `amr_stm32f407/`.
