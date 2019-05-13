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
    SerialPort();
    ~SerialPort();

    void open(QString port_name, unsigned int baudrate);
    void close();

    bool take_ownership();
    void release_ownership();

    QThread* owner() { return _owner; }

    QByteArray read(int n);
    QByteArray readAll();
    void write(QByteArray data);
    void write(const char* data, int n);

private:
    bool check_ownership();

    int _fd;

    QMutex _mutex;
    QThread* _owner;
    QTime _time;
    int _timeout;

    char _buffer[256];
};

#endif // SERIALPORT_H
