#include "serialport.h"
#include <QDebug>
#include <fcntl.h>
#include <stdexcept>
#include <string.h>
#include <unistd.h>

SerialPort::SerialPort()
    : _fd(-1)
    , _owner(nullptr)
    , _timeout(-1)
{
}

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::open(QString port_name, unsigned int baudrate)
{
    _fd = ::open(port_name.toStdString().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        throw std::runtime_error(port_name.toStdString() + ": " + strerror(errno));
    }
    struct termios tio;
    tcgetattr(_fd, &tio);
    tio.c_cflag = baudrate | CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tcflush(_fd, TCIFLUSH);
    tcsetattr(_fd, TCSANOW, &tio);

    _time.start();
}
void SerialPort::close()
{
    if (_fd > 0) {
        ::close(_fd);
        _fd = -1;
    }
}

bool SerialPort::take_ownership()
{
    bool ret = _mutex.tryLock();
    _owner = QThread::currentThread();
    return ret;
}

void SerialPort::release_ownership()
{
    _owner = nullptr;
    _mutex.unlock();
}

QByteArray SerialPort::read(int n)
{
    QByteArray res;
    if (!check_ownership()) {
        qWarning() << QThread::currentThread() << "is not the current owner of this SerialPort -" << _owner;
        return res;
    }

    _time.restart();
    int read_cnt = 0;
    do {
        read_cnt = ::read(_fd, _buffer, n - res.size());
        res.append(_buffer, read_cnt);
        if (_timeout > 0 && _time.elapsed() >= _timeout) {
            throw std::runtime_error("SerialPort timed out");
        }
    } while (res.size() < n);
    return res;
}

QByteArray SerialPort::readAll()
{
    QByteArray res;
    if (!check_ownership()) {
        qWarning() << QThread::currentThread() << "is not the current owner of this SerialPort -" << _owner;
        return res;
    }
    int cnt = ::read(_fd, _buffer, 256);
    res.append(_buffer, cnt);
    return res;
}

void SerialPort::write(QByteArray data)
{
    write(data.data(), data.size());
}

void SerialPort::write(const char* data, int n)
{
    if (!check_ownership()) {
        qWarning() << QThread::currentThread() << "is not the current owner of this SerialPort -" << _owner;
        return;
    }
    ::write(_fd, data, n);
}

bool SerialPort::check_ownership()
{
    return _owner == nullptr || QThread::currentThread() == _owner;
}
