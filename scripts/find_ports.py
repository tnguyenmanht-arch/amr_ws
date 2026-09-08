#!/usr/bin/env python3
"""Tu nhan dien cong nao la STM32, cong nao la LiDAR.

KHONG dua vao ten /dev/ttyUSB0|1 (thu tu doi moi lan cam) va cung khong dua
vao so serial (ca hai deu la CP2102 bao "0001" giong het nhau).

Dau hieu nhan biet: STM32 TU DONG phun "$ODO,..." 100Hz ngay khi mo cong.
LiDAR thi khong — no cho duoc hoi bang giao thuc rieng.

⚠️ KHONG doi hoi doc thanh cong tren cong LiDAR: neu lan chay truoc bi giet
giua chung, LiDAR bi bo lai o trang thai dang quet va pyserial co the nem
loi "device reports readiness to read but returned no data". Vi vay ta chi
di TIM STM32; moi cong con lai mac dinh la LiDAR.
"""
import glob, sys, time, serial

ports = sorted(glob.glob('/dev/ttyUSB*'))
if len(ports) < 2:
    print(f"# LOI: chi thay {len(ports)} cong ttyUSB, can 2. Kiem tra day cam.",
          file=sys.stderr)
    sys.exit(1)

stm32 = None
for port in ports:
    try:
        s = serial.Serial(port, 115200, timeout=0.3)
        time.sleep(0.3)
        s.reset_input_buffer()
        t0, buf = time.time(), b''
        while time.time() - t0 < 1.2 and b'$ODO' not in buf:
            try:
                buf += s.read(s.in_waiting or 1)
            except serial.SerialException:
                break            # cong loi -> chac chan khong phai STM32
        s.close()
        if b'$ODO' in buf:
            stm32 = port
            break
    except Exception as e:
        print(f"# {port}: {e}", file=sys.stderr)

if stm32 is None:
    print("# LOI: khong cong nao phun $ODO -> STM32 chua chay hoac chua cam.",
          file=sys.stderr)
    sys.exit(1)

others = [p for p in ports if p != stm32]
print(f"STM32_PORT={stm32}\nLIDAR_PORT={others[0]}")
