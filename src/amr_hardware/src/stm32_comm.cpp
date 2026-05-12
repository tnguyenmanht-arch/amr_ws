#include "amr_hardware/serial_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cerrno>

namespace amr_hardware {

SerialDriver::~SerialDriver() {
    close();
}

bool SerialDriver::open(const std::string& port, int baud_rate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t speed = toBaudRate(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;  // 1 giây timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void SerialDriver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialDriver::sendCommand(float linear_mm_s, float steering_deg) {
    if (fd_ < 0) return false;

    char buf[64];
    // Định dạng: "V<speed_mm_s> S<steering_deg>\n"
    // CẦN ĐỒNG BỘ với parser phía STM32
    int len = snprintf(buf, sizeof(buf), "V%.1f S%.1f\n", linear_mm_s, steering_deg);

    ssize_t written = write(fd_, buf, len);
    return written == len;
}

bool SerialDriver::readOdom(OdomData& data) {
    if (fd_ < 0) return false;

    char buf[64] = {};
    ssize_t n = read(fd_, buf, sizeof(buf) - 1);
    if (n <= 0) return false;

    // Định dạng nhận: "E<left_ticks> <right_ticks>\n"
    int32_t l, r;
    if (sscanf(buf, "E%d %d", &l, &r) == 2) {
        data.left_ticks  = l;
        data.right_ticks = r;
        return true;
    }
    return false;
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
