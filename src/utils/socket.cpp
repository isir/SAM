#include "socket.h"
#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <string>

Socket::Socket()
    : _sock_fd(socket(AF_INET, SOCK_DGRAM, 0))
    , _new_fd(0)
{
    if (_sock_fd < 0) {
        throw std::runtime_error("Failed to create socket");
    }
}

bool Socket::bind(std::string address, int port)
{
    struct sockaddr_in s;
    s.sin_addr.s_addr = htonl(inet_addr(address.c_str()));
    s.sin_family = AF_INET;
    s.sin_port = htons(static_cast<in_port_t>(port));
    errno = 0;
    int r = ::bind(_sock_fd, reinterpret_cast<struct sockaddr*>(&s), sizeof(s));
    std::cout << "bind=" << errno << std::endl;
    return r == 0;
}

bool Socket::available()
{
    return recv(_sock_fd, _buf, _buf_sz, MSG_DONTWAIT | MSG_PEEK) > 0;
}

std::vector<std::byte> Socket::receive() //simple receive function
{
    ssize_t n = ::recv(_sock_fd, _buf, _buf_sz, MSG_DONTWAIT);
    if (n > 0) {
        return std::vector<std::byte>(_buf, _buf + n);
    } else {
        return std::vector<std::byte>();
    }
}

std::vector<std::byte> Socket::receive(struct sockaddr& addr) //receive message and get the IP address from the sender
{
    socklen_t fromlen;
    //struct sockaddr addr;
    fromlen = sizeof addr;
    ssize_t n = ::recvfrom(_sock_fd, _buf, _buf_sz, MSG_DONTWAIT, &addr, &fromlen);
    if (n > 0) {
        return std::vector<std::byte>(_buf, _buf + n);
    } else {
        return std::vector<std::byte>();
    }
}

bool Socket::send(const char* message, size_t message_length, const struct sockaddr dest_addr)
{
    ssize_t r = ::sendto(_sock_fd, message, message_length, MSG_CONFIRM, &dest_addr, sizeof dest_addr);
    return r == 0;
}


bool Socket::close()
{
    ::shutdown(_sock_fd, 0);
    return ::close(_sock_fd)==0;
}
