#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QByteArray>
#include <QMutex>
#include <QThread>
#include <QTime>
#include <string>
#include <termios.h>

class SerialPort {
public:
    SerialPort(QString port_name, unsigned int baudrate);
    SerialPort();

    ~SerialPort();

    void open(QString port_name, unsigned int baudrate);
    void open();

    void close();

    QString port_name() { return _port_name; }
    unsigned int baudrate() { return _baudrate; }

    void take_ownership();
    bool try_take_ownership();
    void release_ownership();

    QThread* owner() { return _owner; }

    QByteArray read(int n);
    QByteArray read_all();

    void write(QByteArray data);
    void write(const char* data, int n);

private:
    bool check_ownership();

    int _fd;
    QString _port_name;
    unsigned int _baudrate;

    QMutex _mutex;
    QThread* _owner;
    QTime _time;
    int _timeout;

    static const int _internal_buffer_size = 256;
    char _buffer[_internal_buffer_size];
};

#endif // SERIALPORT_H
