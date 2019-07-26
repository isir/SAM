#include "serial_port.h"
#include <chrono>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

SerialPort::SerialPort(std::string port_name, unsigned int baudrate)
    : _fd(-1)
    , _port_name(port_name)
    , _baudrate(baudrate)
    , _owner(std::thread::id())
    , _timeout_ms(0)
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

void SerialPort::open(std::string port_name, unsigned int baudrate)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        throw std::runtime_error(port_name + ": " + strerror(errno));
    }
    struct termios tio;
    tcgetattr(_fd, &tio);
    tio.c_cflag = baudrate | CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tcflush(_fd, TCIFLUSH);
    tcsetattr(_fd, TCSANOW, &tio);
}

void SerialPort::open()
{
    if (_port_name.size() > 0 && _baudrate > 0 && _fd < 0) {
        open(_port_name, _baudrate);
    }
}

void SerialPort::close()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (_fd > 0) {
        ::close(_fd);
        _fd = -1;
    }
    _owner = std::thread::id();
}

void SerialPort::take_ownership()
{
    std::lock_guard<std::mutex> lock(_mutex);

    _owner = std::this_thread::get_id();
}

bool SerialPort::try_take_ownership()
{
    bool ret = _mutex.try_lock();

    if (ret) {
        _owner = std::this_thread::get_id();
        _mutex.unlock();
    }
    return ret;
}

void SerialPort::release_ownership()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (check_ownership()) {
        _owner = std::thread::id();
    }
}

std::vector<std::byte> SerialPort::read(size_t n)
{
    std::lock_guard<std::mutex> lock(_mutex);

    std::vector<std::byte> res;
    if (!check_ownership()) {
        throw std::runtime_error("SerialPort::read called from a thread that is not the current owner of this SerialPort");
    }

    const size_t buf_sz = 256;
    std::byte buf[buf_sz];

    auto start = std::chrono::steady_clock::now();
    do {
        ssize_t nr = ::read(_fd, &buf[0], res.size() - n);
        res.insert(res.end(), &buf[0], &buf[nr]);
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        if (_timeout_ms > 0 && elapsed_ms >= _timeout_ms) {
            throw std::runtime_error("SerialPort timed out");
        }
    } while (res.size() < n);
    return res;
}

std::vector<std::byte> SerialPort::read_all()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!check_ownership()) {
        throw std::runtime_error("SerialPort::read_all called from a thread that is not the current owner of this SerialPort");
    }

    const size_t buf_sz = 256;
    std::byte buf[buf_sz];

    int cnt = ::read(_fd, &buf, buf_sz);
    if (cnt > 0) {
        return std::vector<std::byte>(&buf[0], &buf[cnt]);
    } else {
        return std::vector<std::byte>();
    }
}

void SerialPort::write(std::vector<std::byte> data)
{
    write(reinterpret_cast<const char*>(data.data()), data.size());
}

void SerialPort::write(const char* data, size_t n)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!check_ownership()) {
        throw std::runtime_error("SerialPort::write called from a thread that is not the current owner of this SerialPort");
    }
    ssize_t w = ::write(_fd, data, n);
    if (w < 0 || static_cast<size_t>(w) < n) {
        throw std::runtime_error(port_name() + "::write: " + strerror(errno));
    }
}

bool SerialPort::check_ownership()
{
    return _owner == std::thread::id() || std::this_thread::get_id() == _owner;
}
