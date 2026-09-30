#!/usr/bin/env python3
"""Dump log_buf từ RAM STM32F411 qua ST-Link, KHÔNG dừng chip (mode=HOTPLUG).

Tự tìm địa chỉ log_buf / log_count / log_active trong file .map của bản build
hiện tại (địa chỉ có thể đổi sau mỗi lần build), rồi gọi STM32_Programmer_CLI:
  - đọc log_count, log_active  (-r32)
  - upload toàn bộ log_buf ra file .bin  (-u <addr> <size> <file>)

Cách dùng:
    python scripts/dump_baseline.py                 # ra log.bin ở thư mục hiện tại
    python scripts/dump_baseline.py -o runs/run1.bin
Sau đó:
    python scripts/analyze_baseline.py runs/run1.bin --count <log_count in ra>

⚠️ Dump đúng bản build ĐANG CHẠY trên chip: nếu build lại sau khi nạp, .map
mới có thể lệch địa chỉ so với firmware trên chip.
"""
import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(ROOT, "amr_stm32f407", "Debug", "amr_stm32f407.map")  # board hiện tại
CLI = ("C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/"
       "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506/"
       "tools/bin/STM32_Programmer_CLI.exe")
STATE = {0: "IDLE (chưa nhận $VEL)", 1: "RUN (đang ghi!)", 2: "DONE (đã dừng)"}


def find_symbol(map_text, name):
    """Tìm mục '.bss.<name>' trong .map (GNU ld, -fdata-sections).
    Tên dài thì địa chỉ/kích thước nằm ở dòng kế tiếp -> dùng \\s+ bắt cả xuống dòng."""
    m = re.search(r"^\s*\.(?:bss|data)\." + re.escape(name) +
                  r"\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)", map_text, re.M)
    if not m:
        sys.exit(f"Không tìm thấy '{name}' trong .map -- đã build bản baseline-dwt chưa?")
    return int(m.group(1), 16), int(m.group(2), 16)


def read_u32(addr):
    out = subprocess.run([CLI, "-c", "port=SWD", "mode=HOTPLUG", "-r32", hex(addr), "4"],
                         capture_output=True, text=True).stdout
    m = re.search(r"0x%08X\s*:\s*([0-9A-Fa-f]{8})" % addr, out, re.I)
    if not m:
        print(out)
        sys.exit(f"Không đọc được {hex(addr)} -- ST-Link đã cắm? Chip có nguồn?")
    return int(m.group(1), 16)


def main():
    sys.stdout.reconfigure(encoding="utf-8")   # console Windows mặc định cp1252
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default="log.bin", help="file .bin đầu ra")
    ap.add_argument("--map", default=DEFAULT_MAP, help="file .map của bản build đang chạy")
    args = ap.parse_args()

    text = open(args.map, encoding="utf-8", errors="replace").read()
    buf_addr, buf_size = find_symbol(text, "log_buf")
    cnt_addr, _ = find_symbol(text, "log_count")
    act_addr, _ = find_symbol(text, "log_active")
    print(f"log_buf   @ {buf_addr:#010x}  ({buf_size} byte = {buf_size // 12} bản ghi)")
    print(f"log_count @ {cnt_addr:#010x}   log_active @ {act_addr:#010x}")

    active = read_u32(act_addr)
    count = read_u32(cnt_addr)
    print(f"log_active = {active} -> {STATE.get(active, '??')}")
    print(f"log_count  = {count}")
    if active == 1:
        print("⚠️ Log vẫn đang ghi -- dữ liệu có thể đổi trong lúc dump."
              " Nên chờ script gửi $VEL kết thúc hẳn rồi dump lại.")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    r = subprocess.run([CLI, "-c", "port=SWD", "mode=HOTPLUG",
                        "-u", hex(buf_addr), str(buf_size), os.path.abspath(args.out)],
                       capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(args.out):
        print(r.stdout, r.stderr)
        sys.exit("Upload thất bại.")
    print(f"Đã lưu {args.out} ({os.path.getsize(args.out)} byte)")
    print(f"Tiếp theo: python scripts/analyze_baseline.py {args.out} --count {count}")


if __name__ == "__main__":
    main()
