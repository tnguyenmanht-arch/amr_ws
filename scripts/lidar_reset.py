#!/usr/bin/env python3
"""Reset RPLidar ve trang thai san sang, roi KIEM CHUNG no thuc su quet duoc.

Vi sao can: neu tien trinh truoc bi giet ngang, LiDAR bi bo lai o trang thai dang
phun du lieu -> lan sau bao "Can not start scan: 8000800x". Script nay:
  1. STOP + RESET, doc het banner khoi dong, doi cho im hang
  2. Hoi GET_HEALTH
  3. Thu SCAN that va dem so goi tin -> chi bao THANH CONG neu co du lieu that
Tra ve 0 neu LiDAR san sang, 1 neu khong (de shell script dung han, khong bat Nav2
roi moi phat hien hong).

Dung: lidar_reset.py /dev/ttyUSB1
"""
import sys, time, serial

def drain(s, quiet=0.4, cap=6.0):
    """Doc bo het du lieu ton dong cho toi khi cong im lang `quiet` giay."""
    t0 = last = time.time()
    while time.time() - t0 < cap:
        n = s.in_waiting
        if n:
            s.read(n); last = time.time()
        elif time.time() - last > quiet:
            return
        time.sleep(0.05)

def attempt(port):
    s = serial.Serial(port, 115200, timeout=0.4)
    try:
        s.dtr = False                                  # mo-to CHAY
        s.write(bytes([0xA5, 0x25])); s.flush()        # STOP
        time.sleep(0.3); drain(s)
        s.write(bytes([0xA5, 0x40])); s.flush()        # RESET
        time.sleep(2.5); drain(s)

        s.write(bytes([0xA5, 0x52])); s.flush()        # GET_HEALTH
        d = s.read(7)
        if len(d) < 7 or d[0] != 0xA5 or d[1] != 0x5A:
            return False, "khong tra loi GET_HEALTH"
        st = s.read(3)
        if not st or st[0] != 0:
            return False, f"health status = {st[0] if st else '?'} (0 moi la OK)"

        drain(s, quiet=0.2, cap=1.0)
        s.write(bytes([0xA5, 0x20])); s.flush()        # SCAN
        d = s.read(7)
        if len(d) < 7 or d[0] != 0xA5:
            return False, "khong bat dau quet duoc"
        t0, n = time.time(), 0
        while time.time() - t0 < 1.2:
            n += len(s.read(s.in_waiting or 1))
        s.write(bytes([0xA5, 0x25])); time.sleep(0.2)  # STOP lai, tra cong sach
        drain(s, quiet=0.2, cap=1.0)
        if n < 500:
            return False, f"quet duoc nhung qua it du lieu ({n} byte/1.2s)"
        return True, f"OK — {n} byte/1.2s (~{n//5} diem)"
    finally:
        s.close()

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
for i in range(1, 4):
    ok, msg = attempt(port)
    print(f"  lan {i}: {msg}", flush=True)
    if ok:
        sys.exit(0)
    time.sleep(1.5)
print("  !!! LiDAR khong san sang sau 3 lan thu.", file=sys.stderr)
print("  !!! Thu: rut/cam lai day USB cua LiDAR, hoac chay scripts/usb_reset.py bang sudo.",
      file=sys.stderr)
sys.exit(1)
