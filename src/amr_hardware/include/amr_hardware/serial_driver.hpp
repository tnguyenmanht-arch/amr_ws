#pragma once

#include <string>
#include <cstdint>
#include <termios.h>
#include <array>

namespace amr_hardware {

static constexpr uint8_t  COMM_HDR1         = 0xAAU;
static constexpr uint8_t  COMM_HDR2         = 0x55U;
static constexpr uint8_t  CMD_SET_VEL       = 0x01U;
static constexpr uint8_t  CMD_ODOM          = 0x02U;
static constexpr uint8_t  SET_VEL_DATA_LEN  = 8U;
static constexpr uint8_t  ODOM_DATA_LEN     = 10U;

struct OdomData {
    int32_t  left_ticks;
    int32_t  right_ticks;
    uint16_t steering_pos;
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

    enum class RxState : uint8_t {
        WAIT_HDR1, WAIT_HDR2, WAIT_CMD, WAIT_LEN, READ_DATA, READ_CRC
    };
    RxState rx_state_ = RxState::WAIT_HDR1;
    uint8_t rx_cmd_   = 0;
    uint8_t rx_len_   = 0;
    uint8_t rx_count_ = 0;
    std::array<uint8_t, 16> rx_buf_{};

    static uint8_t calcCRC(uint8_t cmd, uint8_t len,
                            const uint8_t* data, uint8_t data_len);
};

}  // namespace amr_hardware
