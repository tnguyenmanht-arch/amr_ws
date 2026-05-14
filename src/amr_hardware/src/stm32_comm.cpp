#include "amr_hardware/serial_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

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

    uint8_t frame[13];
    frame[0] = COMM_HDR1;
    frame[1] = COMM_HDR2;
    frame[2] = CMD_SET_VEL;
    frame[3] = SET_VEL_DATA_LEN;

    memcpy(&frame[4], &linear_x,  4);
    memcpy(&frame[8], &angular_z, 4);

    frame[12] = calcCRC(frame[2], frame[3], &frame[4], SET_VEL_DATA_LEN);

    ssize_t written = write(fd_, frame, sizeof(frame));
    return written == static_cast<ssize_t>(sizeof(frame));
}

bool SerialDriver::readOdom(OdomData& data) {
    if (fd_ < 0) return false;

    uint8_t byte;
    while (read(fd_, &byte, 1) == 1) {
        switch (rx_state_) {
            case RxState::WAIT_HDR1:
                if (byte == COMM_HDR1) rx_state_ = RxState::WAIT_HDR2;
                break;

            case RxState::WAIT_HDR2:
                rx_state_ = (byte == COMM_HDR2) ? RxState::WAIT_CMD
                                                 : RxState::WAIT_HDR1;
                break;

            case RxState::WAIT_CMD:
                rx_cmd_   = byte;
                rx_state_ = RxState::WAIT_LEN;
                break;

            case RxState::WAIT_LEN:
                rx_len_   = byte;
                rx_count_ = 0;
                rx_buf_.fill(0);
                if (rx_cmd_ == CMD_ODOM && rx_len_ == ODOM_DATA_LEN) {
                    rx_state_ = RxState::READ_DATA;
                } else {
                    rx_state_ = RxState::WAIT_HDR1;
                }
                break;

            case RxState::READ_DATA:
                rx_buf_[rx_count_++] = byte;
                if (rx_count_ >= rx_len_) {
                    rx_state_ = RxState::READ_CRC;
                }
                break;

            case RxState::READ_CRC: {
                uint8_t expected = calcCRC(rx_cmd_, rx_len_,
                                           rx_buf_.data(), rx_len_);
                rx_state_ = RxState::WAIT_HDR1;

                if (byte != expected) break;

                memcpy(&data.left_ticks,   &rx_buf_[0], 4);
                memcpy(&data.right_ticks,  &rx_buf_[4], 4);
                memcpy(&data.steering_pos, &rx_buf_[8], 2);
                return true;
            }
        }
    }
    return false;
}

uint8_t SerialDriver::calcCRC(uint8_t cmd, uint8_t len,
                               const uint8_t* data, uint8_t data_len) {
    uint8_t crc = cmd ^ len;
    for (uint8_t i = 0; i < data_len; i++) {
        crc ^= data[i];
    }
    return crc;
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
