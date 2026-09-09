#!/usr/bin/env python3
"""Reset CUNG mot thiet bi USB — tuong duong rut ra cam lai, khong can dong vao day.

Vi sao can: RPLidar bi bo lai o trang thai DANG PHUN du lieu neu tien trinh truoc
bi giet ngang (Ctrl-C giua chung, timeout, kill -9). Lan ket noi sau doc phai rac
va bao "Can not start scan: 8000800x". Lenh STOP+RESET o muc giao thuc khong phai
luc nao cung go duoc; reset o muc USB thi go duoc chac chan.

Dung: usb_reset.py /dev/ttyUSB1
"""
import fcntl, os, sys, time

USBDEVFS_RESET = ord('U') << 8 | 20

def usb_node(tty):
    """Tu /dev/ttyUSBx tim ra /dev/bus/usb/BBB/DDD cua thiet bi USB cha."""
    p = os.path.realpath(f"/sys/class/tty/{os.path.basename(tty)}/device")
    # di nguoc len cho toi khi gap thu muc co busnum+devnum (= thiet bi USB, khong
    # phai giao dien hay cong noi tiep)
    for _ in range(4):
        if os.path.exists(os.path.join(p, "busnum")):
            b = int(open(os.path.join(p, "busnum")).read())
            d = int(open(os.path.join(p, "devnum")).read())
            return f"/dev/bus/usb/{b:03d}/{d:03d}"
        p = os.path.dirname(p)
    raise RuntimeError(f"khong tim duoc thiet bi USB cha cua {tty}")

def main():
    tty = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    node = usb_node(tty)
    fd = os.open(node, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    print(f"da reset USB {node} ({tty})")
    time.sleep(3)   # cho kernel gan lai /dev/ttyUSBx

main()
