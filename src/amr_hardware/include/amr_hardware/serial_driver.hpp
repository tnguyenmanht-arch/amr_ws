#pragma once

#include <string>
#include <cstdint>
#include <cstddef>
#include <termios.h>

namespace amr_hardware {

// Protocol ASCII line-based khớp firmware STM32 (jetson_comm.c):
//   Jetson -> STM32:  "$VEL,<linear>,<angular>\n"
//   STM32  -> Jetson: "$ODO,<enc_l>,<enc_r>,<steer_deg>\n"

struct OdomData {
    int32_t left_ticks;
    int32_t right_ticks;
    float   steer_deg;   // góc lái thực tế STM32 gửi về (độ, có dấu)
};

class SerialDriver {
public:
    SerialDriver() = default;
    ~SerialDriver();

    bool open(const std::string& port, int baud_rate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    bool sendCmdVel(float linear_x, float angular_z);
    bool readOdom(OdomData& data);

private:
    int fd_ = -1;
    speed_t toBaudRate(int baud);

    // Gom byte tới khi gặp '\n' thành 1 dòng ASCII hoàn chỉnh
    static constexpr std::size_t kLineBufSize = 64;
    char        line_buf_[kLineBufSize]{};
    std::size_t line_len_ = 0;
};

}  // namespace amr_hardware
