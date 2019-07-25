#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <cstddef>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>

class SerialPort {
public:
    SerialPort(std::string port_name, unsigned int baudrate);
    SerialPort();

    ~SerialPort();

    void open(std::string port_name, unsigned int baudrate);
    void open();

    void close();

    std::string port_name() { return _port_name; }
    unsigned int baudrate() { return _baudrate; }

    void take_ownership();
    bool try_take_ownership();
    void release_ownership();

    std::thread::id owner() { return _owner; }

    std::vector<std::byte> read(size_t n);
    std::vector<std::byte> read_all();

    void write(std::vector<std::byte> data);
    void write(const char* data, size_t n);

private:
    bool check_ownership();

    int _fd;
    std::string _port_name;
    unsigned int _baudrate;

    std::mutex _mutex;
    std::thread::id _owner;
    unsigned int _timeout_ms;
};

#endif // SERIALPORT_H
