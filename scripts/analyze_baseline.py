#!/usr/bin/env python3
"""Phân tích log timing DWT dump từ RAM STM32F411 (branch baseline-dwt).

Đầu vào: file .bin = nội dung thô của mảng log_buf (xem dwt_log.h), mỗi bản
ghi 12 byte little-endian:  t_start u32 | t_end u32 | len u16 | id u8 | rsv u8

Cách dùng:
    python scripts/analyze_baseline.py log.bin
    python scripts/analyze_baseline.py log.bin --count 4123 --outdir out/

Nếu không truyền --count, script tự cắt ở bản ghi đầu tiên có id == 0
(ô RAM chưa ghi, .bss được xóa về 0 lúc khởi động).
"""
import argparse
import os
import struct
import sys
from collections import Counter

import numpy as np

# ---- Định dạng bản ghi: PHẢI khớp struct LogRecord trong dwt_log.h ----
REC_FMT = "<IIHBB"          # t_start, t_end, len, id, reserved
REC_SIZE = 12               # sizeof(LogRecord), firmware có _Static_assert == 12

# ---- Mã sự kiện: PHẢI khớp enum DWT_EventId trong dwt_log.h ----
EV_PID, EV_ODO, EV_IMUH, EV_DUMMY, EV_LOG_OVH = 1, 2, 3, 4, 5
EVENT_NAMES = {EV_PID: "PID", EV_ODO: "ODO", EV_IMUH: "IMUH",
               EV_DUMMY: "DUMMY", EV_LOG_OVH: "LOG_OVH"}

CPU_HZ = 168e6              # SYSCLK: F407 = 168 MHz (mặc định), F411 cũ = 100 MHz
                            # -> đổi bằng --cpu-hz, vì DWT đếm theo SYSCLK
PID_NOMINAL_US = 10_000.0   # PID_INTERVAL_MS = 10 ms
BAUD = 115200               # USART2, 8N1 = 10 bit/byte


def check_format():
    """Tự kiểm tra kích thước/padding của định dạng struct Python."""
    size = struct.calcsize(REC_FMT)
    if size != REC_SIZE:
        sys.exit(f"LỖI: struct '{REC_FMT}' = {size} byte, cần {REC_SIZE} -- sửa REC_FMT")
    # Dấu '<' = little-endian, KHÔNG căn lề -> đúng layout C khi các trường
    # đã sắp xếp sẵn như LogRecord (u32,u32,u16,u8,u8 -> offset 0,4,8,10,11).
    offsets = [0, 4, 8, 10, 11]
    probe = struct.pack(REC_FMT, 0x11111111, 0x22222222, 0x3333, 0x44, 0x55)
    got = [probe.index(b) for b in (b"\x11", b"\x22", b"\x33", b"\x44", b"\x55")]
    if got != offsets:
        sys.exit(f"LỖI: offset trường {got} != {offsets} (layout C)")


def load(path, count):
    raw = open(path, "rb").read()
    if len(raw) % REC_SIZE:
        print(f"CẢNH BÁO: kích thước file {len(raw)} không chia hết cho {REC_SIZE}"
              f" -- bỏ {len(raw) % REC_SIZE} byte cuối. Kiểm tra lại lệnh dump.")
        raw = raw[: len(raw) - len(raw) % REC_SIZE]
    recs = list(struct.iter_unpack(REC_FMT, raw))

    if count is not None:
        recs = recs[:count]
    else:
        # Cắt ở ô trống đầu tiên (id == 0)
        for i, r in enumerate(recs):
            if r[3] == 0:
                recs = recs[:i]
                break

    # Kiểm tra tính toàn vẹn: id lạ hoặc reserved != 0 => sai địa chỉ/định dạng dump
    bad = [i for i, r in enumerate(recs) if r[3] not in EVENT_NAMES or r[4] != 0]
    if bad:
        print(f"CẢNH BÁO: {len(bad)} bản ghi có id lạ/reserved != 0 (ví dụ index"
              f" {bad[:5]}) -- có thể dump sai địa chỉ hoặc sai --count.")
    return recs


def unwrap(ticks):
    """Chuyển chuỗi CYCCNT 32-bit (tràn sau ~42.9 s) thành trục thời gian liên tục.
    Dùng hiệu có dấu giữa 2 phần tử liên tiếp nên chịu được lệch thứ tự nhỏ."""
    out = np.empty(len(ticks), dtype=np.int64)
    acc = int(ticks[0])
    out[0] = acc
    for i in range(1, len(ticks)):
        d = (int(ticks[i]) - int(ticks[i - 1])) & 0xFFFFFFFF
        if d >= 0x80000000:
            d -= 0x100000000
        acc += d
        out[i] = acc
    return out


def us(ticks):
    return np.asarray(ticks, dtype=np.float64) / CPU_HZ * 1e6


def dur_ticks(recs):
    return np.array([(r[1] - r[0]) & 0xFFFFFFFF for r in recs], dtype=np.int64)


def stats_line(name, x, unit="µs"):
    if len(x) == 0:
        return f"  {name:<28s} (không có dữ liệu)"
    return (f"  {name:<28s} n={len(x):5d}  min={np.min(x):9.2f}  max={np.max(x):9.2f}"
            f"  mean={np.mean(x):9.2f}  std={np.std(x):8.2f} {unit}")


def main():
    global CPU_HZ   # --cpu-hz ghi đè hằng số mặc định
    sys.stdout.reconfigure(encoding="utf-8")   # console Windows mặc định cp1252
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bin", help="file dump RAM của log_buf")
    ap.add_argument("--count", type=int, default=None,
                    help="log_count đọc từ chip (nếu có); mặc định tự cắt ở id==0")
    ap.add_argument("--outdir", default=None, help="thư mục lưu PNG (mặc định cạnh file .bin)")
    ap.add_argument("--cpu-hz", type=float, default=CPU_HZ,
                    help="SYSCLK của chip đã đo (mặc định 168e6 = F407; F411 cũ dùng 100e6)")
    args = ap.parse_args()
    CPU_HZ = args.cpu_hz

    check_format()
    recs = load(args.bin, args.count)
    if len(recs) < 3:
        sys.exit(f"Quá ít bản ghi ({len(recs)}) -- log chưa được bật? (cần $VEL đầu tiên)")

    starts = unwrap([r[0] for r in recs])
    ends = starts + dur_ticks(recs)
    span_ticks = int(ends.max() - starts.min())
    span_s = span_ticks / CPU_HZ

    by_id = {k: [i for i, r in enumerate(recs) if r[3] == k] for k in EVENT_NAMES}
    print(f"File: {args.bin}")
    print(f"Tổng bản ghi: {len(recs)}  |  thời lượng phiên đo: {span_s:.3f} s")
    print("Số bản ghi theo sự kiện: " +
          ", ".join(f"{EVENT_NAMES[k]}={len(v)}" for k, v in by_id.items()))
    print()

    # ---- Overhead của chính việc ghi log ----
    ovh = us(dur_ticks([recs[i] for i in by_id[EV_LOG_OVH]]))
    ovh_cyc = dur_ticks([recs[i] for i in by_id[EV_LOG_OVH]])
    print("[Overhead đo]")
    print(stats_line("1 lần DWT_Log + DWT_Now", ovh))
    if len(ovh_cyc):
        print(f"  (theo chu kỳ CPU: min={ovh_cyc.min()}  max={ovh_cyc.max()}"
              f"  median={int(np.median(ovh_cyc))} cycles)")
    print()

    # ---- PID: chu kỳ + jitter + thời gian thực thi ----
    pid_idx = by_id[EV_PID]
    pid_start = starts[pid_idx]
    pid_period = us(np.diff(pid_start))
    pid_exec = us(dur_ticks([recs[i] for i in pid_idx]))
    jitter = pid_period - PID_NOMINAL_US
    print("[PID]")
    print(stats_line("chu kỳ PID", pid_period))
    print(stats_line("jitter (chu kỳ - 10 ms)", jitter))
    if len(jitter):
        print(f"  jitter |max| = {np.max(np.abs(jitter)):.2f} µs ;"
              f" p99 = {np.percentile(pid_period, 99):.2f} µs ;"
              f" chu kỳ > 11 ms: {np.sum(pid_period > 11_000)}/{len(pid_period)}")
    print(stats_line("thời gian thực thi PID", pid_exec))
    print()

    # ---- $ODO: thời gian gửi + phân bố độ dài khung ----
    odo_recs = [recs[i] for i in by_id[EV_ODO]]
    odo_tx = us(dur_ticks(odo_recs))
    odo_len = np.array([r[2] for r in odo_recs])
    print("[$ODO]")
    print(stats_line("thời gian snprintf+gửi", odo_tx))
    if len(odo_len):
        hist = Counter(odo_len.tolist())
        print("  phân bố độ dài khung (byte: số khung): " +
              ", ".join(f"{k}:{hist[k]}" for k in sorted(hist)))
        # So với lý thuyết đường truyền: 10 bit/byte ở 115200 baud
        per_byte_theory = 10 / BAUD * 1e6
        print(f"  lý thuyết đường dây: {per_byte_theory:.2f} µs/byte ->"
              f" {odo_len.min()*per_byte_theory:.0f}..{odo_len.max()*per_byte_theory:.0f} µs"
              f" cho {odo_len.min()}..{odo_len.max()} byte")
        excess = odo_tx - odo_len * per_byte_theory
        print(stats_line("phần dư so với lý thuyết", excess)
              + "   (snprintf + chờ TC byte cuối)")
    print()

    # ---- $IMUH ----
    imuh_recs = [recs[i] for i in by_id[EV_IMUH]]
    imuh_tx = us(dur_ticks(imuh_recs))
    print("[$IMUH]")
    print(stats_line("thời gian gửi", imuh_tx))
    print()

    # ---- Tải CPU cho truyền thông (TX polling chặn CPU suốt thời gian gửi) ----
    comm_ticks = dur_ticks(odo_recs).sum() + dur_ticks(imuh_recs).sum()
    pid_ticks = dur_ticks([recs[i] for i in pid_idx]).sum()
    print("[Tải CPU trên toàn phiên đo]")
    print(f"  truyền thông ($ODO + $IMUH): {100 * comm_ticks / span_ticks:6.2f} %")
    print(f"    trong đó $ODO:             {100 * dur_ticks(odo_recs).sum() / span_ticks:6.2f} %")
    print(f"  thân PID:                    {100 * pid_ticks / span_ticks:6.2f} %")
    print()

    # ---- Histogram ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Không có matplotlib -- bỏ qua vẽ biểu đồ (pip install matplotlib)")
        return

    outdir = args.outdir or os.path.dirname(os.path.abspath(args.bin))
    os.makedirs(outdir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(pid_period / 1000.0, bins=60)
    ax.axvline(PID_NOMINAL_US / 1000.0, linestyle="--", linewidth=1)
    ax.set_xlabel("Chu kỳ PID (ms)")
    ax.set_ylabel("Số lần")
    ax.set_title(f"Chu kỳ PID — superloop baseline (n={len(pid_period)})")
    fig.tight_layout()
    p1 = os.path.join(outdir, "pid_period_hist.png")
    fig.savefig(p1, dpi=150)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(odo_tx / 1000.0, bins=60)
    ax.set_xlabel("Thời gian snprintf + gửi \\$ODO (ms)")
    ax.set_ylabel("Số khung")
    ax.set_title(f"Thời gian gửi \\$ODO — TX polling 115200 (n={len(odo_tx)})")
    fig.tight_layout()
    p2 = os.path.join(outdir, "odo_tx_hist.png")
    fig.savefig(p2, dpi=150)
    print(f"Đã lưu: {p1}\n        {p2}")


if __name__ == "__main__":
    main()
