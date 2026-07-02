#include "amr_hardware/serial_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>

namespace amr_hardware {

SerialDriver::~SerialDriver() { close(); }

bool SerialDriver::open(const std::string& port, int baud_rate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }

    speed_t speed = toBaudRate(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |=  CS8 | CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                     PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }
    return true;
}

void SerialDriver::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialDriver::sendCmdVel(float linear_x, float angular_z) {
    if (fd_ < 0) return false;

    char buf[48];
    int n = snprintf(buf, sizeof(buf), "$VEL,%.2f,%.2f\n", linear_x, angular_z);
    if (n <= 0) return false;

    ssize_t written = write(fd_, buf, static_cast<std::size_t>(n));
    return written == static_cast<ssize_t>(n);
}

bool SerialDriver::readOdom(OdomData& data) {
    if (fd_ < 0) return false;

    // Đọc HẾT byte đang có trong kernel buffer, không dừng ở dòng hợp lệ đầu
    // tiên: firmware gửi $ODO ở 100Hz nhưng node chỉ đọc mỗi timer tick
    // (publish_rate_hz, thường 20Hz) -> nếu dừng sớm, dữ liệu tồn đọng sẽ
    // dồn lại và ngày càng trễ so với thực tế. Luôn giữ bản ghi MỚI NHẤT.
    bool got_valid = false;
    uint8_t byte;
    while (read(fd_, &byte, 1) == 1) {
        if (byte == '\n') {
            line_buf_[line_len_] = '\0';

            long  enc_l = 0, enc_r = 0;
            float steer = 0.0f;
            int matched = sscanf(line_buf_, "$ODO,%ld,%ld,%f",
                                  &enc_l, &enc_r, &steer);
            line_len_ = 0;

            if (matched == 3) {
                data.left_ticks  = static_cast<int32_t>(enc_l);
                data.right_ticks = static_cast<int32_t>(enc_r);
                data.steer_deg   = steer;
                got_valid = true;
            }
            // Dòng không khớp format (nhiễu/dòng debug khác) -> bỏ qua, đọc tiếp
        } else if (byte == '\r') {
            // Bỏ qua CR (STM32 chỉ gửi '\n' nhưng phòng khi có CRLF)
        } else if (line_len_ < kLineBufSize - 1) {
            line_buf_[line_len_++] = static_cast<char>(byte);
        } else {
            line_len_ = 0;  // Tràn buffer -> dòng lỗi, reset
        }
    }
    return got_valid;
}

speed_t SerialDriver::toBaudRate(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

}  // namespace amr_hardware
