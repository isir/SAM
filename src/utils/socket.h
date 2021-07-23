#ifndef SOCKET_H
#define SOCKET_H

#include <string>
#include <vector>

class Socket {
public:
    Socket();

    bool bind(std::string address, int port);

    bool available();
    std::vector<std::byte> receive();
    std::vector<std::byte> receive(struct sockaddr& addr);

    bool send(const char* message, size_t message_length, const struct sockaddr dest_addr);

    bool close();

private:
    int _sock_fd;
    int _new_fd;

    static const std::size_t _buf_sz = 1024;
    std::byte _buf[_buf_sz];
};

#endif // SOCKET_H
