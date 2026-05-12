#pragma once

#include <string>
#include <cstdint>
#include <termios.h>

namespace amr_hardware {

// Giao thức UART với STM32: mỗi frame là chuỗi ASCII đơn giản
// TX (Jetson → STM32): "V<linear_mm_s> S<steering_deg>\n"
// RX (STM32 → Jetson): "E<left_ticks> <right_ticks>\n"
//
// Chuỗi điều khiển phần cứng phía STM32:
//   Motor:  STM32 → I2C → Hiwonder 4-Ch Encoder Motor Driver → JGB37-520
//   Servo:  STM32 USART → TTL → Hiwonder TTL Bus Servo Debugging Board → HTS-20H
//
// CẦN ĐIỀU CHỈNH giao thức nếu dùng micro-ROS thay vì custom UART

struct OdomData {
    int32_t left_ticks;
    int32_t right_ticks;
};

class SerialDriver {
public:
    SerialDriver() = default;
    ~SerialDriver();

    // Mở port serial — port: "/dev/ttyUSB0", baud: 115200
    bool open(const std::string& port, int baud_rate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Gửi lệnh điều khiển sang STM32
    bool sendCommand(float linear_mm_s, float steering_deg);

    // Đọc dữ liệu encoder từ STM32
    bool readOdom(OdomData& data);

private:
    int fd_ = -1;
    speed_t toBaudRate(int baud);
};

}  // namespace amr_hardware
