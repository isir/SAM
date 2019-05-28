#include "serial_port.h"
#include <QDebug>
#include <QMutexLocker>
#include <fcntl.h>
#include <unistd.h>

SerialPort::SerialPort(QString port_name, unsigned int baudrate)
    : _fd(-1)
    , _port_name(port_name)
    , _baudrate(baudrate)
    , _owner(nullptr)
    , _timeout(-1)
{
}

SerialPort::SerialPort()
    : SerialPort("", 0)
{
}

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::open(QString port_name, unsigned int baudrate)
{
    QMutexLocker lock(&_mutex);

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

void SerialPort::open()
{
    if (!_port_name.isEmpty() && _baudrate > 0 && _fd < 0) {
        open(_port_name, _baudrate);
    }
}

void SerialPort::close()
{
    QMutexLocker lock(&_mutex);

    if (_fd > 0) {
        ::close(_fd);
        _fd = -1;
    }
    _owner = nullptr;
}

void SerialPort::take_ownership()
{
    QMutexLocker lock(&_mutex);

    _owner = QThread::currentThread();
}

bool SerialPort::try_take_ownership()
{
    bool ret = _mutex.tryLock();

    if (ret) {
        _owner = QThread::currentThread();
        _mutex.unlock();
    }
    return ret;
}

void SerialPort::release_ownership()
{
    QMutexLocker lock(&_mutex);

    if (check_ownership()) {
        _owner = nullptr;
    }
}

QByteArray SerialPort::read(int n)
{
    QMutexLocker lock(&_mutex);

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

QByteArray SerialPort::read_all()
{
    QMutexLocker lock(&_mutex);

    if (!check_ownership()) {
        qWarning() << QThread::currentThread() << "is not the current owner of this SerialPort -" << _owner;
        return QByteArray();
    }
    int cnt = ::read(_fd, _buffer, _internal_buffer_size);
    if (cnt > 0) {
        return QByteArray(_buffer, cnt);
    } else {
        return QByteArray();
    }
}

void SerialPort::write(QByteArray data)
{
    qDebug() << "SerialPort::write" << data.toHex() << "(" << data.size() << ")";
    write(data.data(), data.size());
}

void SerialPort::write(const char* data, int n)
{
    QMutexLocker lock(&_mutex);

    if (!check_ownership()) {
        qWarning() << QThread::currentThread() << "is not the current owner of this SerialPort -" << _owner;
        return;
    }
    int w = ::write(_fd, data, n);
    if (w < n) {
        throw std::runtime_error(port_name().toStdString() + "::write: " + strerror(errno));
    }
}

bool SerialPort::check_ownership()
{
    return _owner == nullptr || QThread::currentThread() == _owner;
}
