#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include "message.h"
#include "types.h"
#include "utils/serial_port.h"
#include <cstddef>
#include <vector>

namespace RC {
class RoboClaw {
public:
    typedef enum {
        M1 = 1,
        M2 = 2
    } Channel;

    RoboClaw();
    virtual ~RoboClaw();

    void init(std::string port_name, unsigned int baudrate, uint8_t address = 0x80, Channel channel = M1);

    inline uint8_t address() { return _address; }
    inline Channel chan() { return _channel; }

    void forward(uint8_t value);
    void backward(uint8_t value);
    int32_t read_encoder_position();
    int32_t read_encoder_speed();
    void set_velocity(int32_t value);
    std::string read_firmware_version();
    void set_encoder_position(int32_t value);
    double read_main_battery_voltage();
    double read_current();
    void set_velocity_pid(velocity_pid_params_t params);
    velocity_pid_params_t read_velocity_pid();
    void set_position_pid(position_pid_params_t params);
    position_pid_params_t read_position_pid();
    void move_to(uint32_t accel, uint32_t speed, uint32_t decel, int32_t pos);

private:
    std::vector<std::byte> send(const Message& msg);
    inline uint8_t get_fn_code(uint8_t if_m1, uint8_t if_m2)
    {
        return (_channel == Channel::M1 ? if_m1 : if_m2);
    }

    std::shared_ptr<SerialPort> _serial_port;
    uint8_t _address;
    Channel _channel;
};
}

#endif // ROBOCLAW_H
