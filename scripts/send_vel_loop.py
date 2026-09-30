#!/usr/bin/env python3
"""Gửi $VEL lặp lại đều đặn qua CP2102 để đo baseline DWT (branch baseline-dwt).

⚠️ LÀM MOTOR QUAY. Nhấc bánh khỏi mặt đất trước khi chạy. Script hỏi xác
nhận trước khi gửi lệnh đầu tiên (bỏ qua bằng --yes).

Hành vi:
- Gửi "$VEL,<linear>,<angular>\\n" mỗi --period giây (mặc định 0.1 s, an toàn
  so với watchdog 300 ms của firmware) trong --duration giây.
- Script CHỈ gửi (không đọc/parse $ODO) nên vòng lặp không bị trễ như bài học
  script đơn luồng cũ; $ODO đến được xả bỏ để không đầy bộ đệm driver.
- Kết thúc (hết giờ hoặc Ctrl+C): gửi $VEL,0.00,0.00 vài lần rồi NGỪNG GỬI.
  300 ms sau watchdog firmware trip -> phiên log DWT tự dừng (log_active=DONE).
- Báo khoảng cách gửi lớn nhất thực tế: nếu > 300 ms thì watchdog đã trip
  giữa chừng và phiên log bị cắt sớm -> phải đo lại.

Cách dùng:
    python scripts/send_vel_loop.py --port COM9
    python scripts/send_vel_loop.py --port COM9 --linear 0.2 --duration 20
"""
import argparse
import sys
import time

import serial


def send(ser, lin, ang):
    ser.write(f"$VEL,{lin:.2f},{ang:.2f}\n".encode("ascii"))


def main():
    sys.stdout.reconfigure(encoding="utf-8")   # console Windows mặc định cp1252
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM9", help="cổng CP2102 (mặc định COM9)")
    ap.add_argument("--linear", type=float, default=0.2, help="m/s (mặc định 0.2)")
    ap.add_argument("--angular", type=float, default=0.0, help="rad/s (mặc định 0)")
    ap.add_argument("--period", type=float, default=0.1, help="chu kỳ gửi, s (mặc định 0.1)")
    ap.add_argument("--duration", type=float, default=20.0, help="thời lượng, s (mặc định 20)")
    ap.add_argument("--yes", action="store_true", help="bỏ qua câu hỏi xác nhận")
    args = ap.parse_args()

    if args.period >= 0.3:
        sys.exit("--period phải < 0.3 s (watchdog $VEL của firmware là 300 ms)")

    ser = serial.Serial(args.port, 115200, timeout=0)
    print(f"Mở {args.port} OK. Sẽ gửi $VEL,{args.linear:.2f},{args.angular:.2f}"
          f" mỗi {args.period*1000:.0f} ms trong {args.duration:.0f} s.", flush=True)

    if not args.yes:
        ans = input("Bánh đã NHẤC khỏi mặt đất? Gõ 'y' để motor bắt đầu quay: ")
        if ans.strip().lower() != "y":
            ser.close()
            sys.exit("Đã hủy, chưa gửi lệnh nào.")

    n_sent = 0
    max_gap = 0.0
    last = None
    t_begin = time.perf_counter()
    next_t = t_begin
    try:
        while True:
            now = time.perf_counter()
            if now - t_begin >= args.duration:
                break
            if now >= next_t:
                send(ser, args.linear, args.angular)
                n_sent += 1
                if last is not None:
                    max_gap = max(max_gap, now - last)
                last = now
                next_t += args.period
                if n_sent % 50 == 0:
                    print(f"  t={now - t_begin:5.1f}s  đã gửi {n_sent}"
                          f"  gap lớn nhất {max_gap*1000:.0f} ms", flush=True)
            ser.reset_input_buffer()          # xả $ODO, không cần đọc
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\nCtrl+C -- dừng.", flush=True)
    finally:
        for _ in range(5):                    # lệnh dừng rõ ràng
            send(ser, 0.0, 0.0)
            time.sleep(0.05)
        ser.close()

    print(f"Xong: gửi {n_sent} lệnh, khoảng cách gửi lớn nhất {max_gap*1000:.0f} ms.",
          flush=True)
    if max_gap > 0.3:
        print("⚠️ Có khoảng > 300 ms: watchdog đã trip giữa chừng -> phiên log bị"
              " cắt sớm. Nên đo lại.", flush=True)
    print("Chờ ~0.5 s cho watchdog trip (log tự dừng), rồi dump RAM.", flush=True)


if __name__ == "__main__":
    main()
