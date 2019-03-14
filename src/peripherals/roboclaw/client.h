#ifndef ROBOCLAWCLIENT_H
#define ROBOCLAWCLIENT_H

#include <QObject>
#include <QList>
#include <QThread>
#include "message.h"
#include "types.h"

namespace RoboClaw {
class Client : public QObject
{
    Q_OBJECT
public:
    typedef enum {
        M1 = 1,
        M2 = 2
    } Channel;

    Client(quint8 address = 0x80, Channel channel = M1);
    virtual ~Client();

    void connect_to_server(QString port_name, int baudrate, Qt::ConnectionType connection = Qt::AutoConnection);

    inline quint8 address() { return _address; }
    inline Channel chan() { return _channel; }

    void set_address(quint8 address, Channel channel);
    void on_answer_received(QByteArray data);

    void forward(quint8 value);
    void backward(quint8 value);
    qint32 read_encoder_position();
    qint32 read_encoder_speed();
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
    QByteArray send(const Message& msg, bool wait_for_answer = false);

    QList<QThread*> _callers;
    QString _server_port_name;
    int _server_baudrate;
    quint8 _address;
    Channel _channel;

signals:
    void answer_received_internal(QByteArray data);
    void send_msg(Client* client, Message msg);
};
}

#endif // ROBOCLAWCLIENT_H
