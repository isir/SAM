#ifndef ROBOCLAWCLIENT_H
#define ROBOCLAWCLIENT_H

#include "message.h"
#include "types.h"
#include "utils/serial_port.h"
#include <QList>
#include <QObject>
#include <QThread>

namespace RC {
class RoboClaw : public QObject {
    Q_OBJECT
public:
    typedef enum {
        M1 = 1,
        M2 = 2
    } Channel;

    RoboClaw();
    virtual ~RoboClaw();

    void init(QString port_name, unsigned int baudrate, quint8 address = 0x80, Channel channel = M1);

    inline quint8 address() { return _address; }
    inline Channel chan() { return _channel; }

    void forward(quint8 value);
    void backward(quint8 value);
    qint32 read_encoder_position();
    qint32 read_encoder_speed();
    void set_velocity(qint32 value);
    QString read_firmware_version();
    void set_encoder_position(qint32 value);
    double read_main_battery_voltage();
    double read_current();
    void set_velocity_pid(velocity_pid_params_t params);
    velocity_pid_params_t read_velocity_pid();
    void set_position_pid(position_pid_params_t params);
    position_pid_params_t read_position_pid();
    void move_to(quint32 accel, quint32 speed, quint32 decel, qint32 pos);

private:
    QByteArray send(const Message& msg);

    std::shared_ptr<SerialPort> _serial_port;
    quint8 _address;
    Channel _channel;
};
}

#endif // ROBOCLAWCLIENT_H
