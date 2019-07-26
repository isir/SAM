#ifndef ROBOCLAWMESSAGE_H
#define ROBOCLAWMESSAGE_H

#include "answer.h"
#include <memory>

namespace RC {
class Message {
    friend class Client;

public:
    explicit Message();
    explicit Message(uint8_t address, uint8_t command, std::shared_ptr<Answer::BaseAnswer> ans, std::vector<std::byte> payload = std::vector<std::byte>(), bool append_crc = true);
    Message(const Message& other)
    {
        _data = other._data;
        _answer = other._answer;
    }

    ~Message() {}

    static uint16_t crc16(std::vector<std::byte> packet);

    std::vector<std::byte> data() const { return _data; }
    std::shared_ptr<Answer::BaseAnswer> answer() const { return _answer; }

    std::string to_string() const;

private:
    std::vector<std::byte> _data;
    std::shared_ptr<Answer::BaseAnswer> _answer;
};
}

#endif // ROBOCLAWMESSAGE_H
