#include "message.h"
#include <iomanip>
#include <sstream>

namespace RC {
Message::Message()
{
}

Message::Message(uint8_t address, uint8_t command, std::shared_ptr<Answer::BaseAnswer> ans, std::vector<std::byte> payload, bool append_crc)
    : _answer(ans)
{
    _data.push_back(std::byte { address });
    _data.push_back(std::byte { command });
    _data.insert(_data.end(), payload.begin(), payload.end());
    if (append_crc) {
        uint16_t crc = crc16(_data);
        _data.push_back(static_cast<std::byte>(crc >> 8));
        _data.push_back(static_cast<std::byte>(crc & 0xff));
    }
}

std::string Message::to_string() const
{
    std::string ret = "[";
    if (_data.size() >= 2) {
        ret.append(std::to_string(static_cast<unsigned int>(_data[0])));
        ret.push_back(',');
        ret.append(std::to_string(static_cast<unsigned int>(_data[1])));
    }
    if (_data.size() >= 3) {
        ret.append(",0x");
        std::stringstream ss;
        ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex;
        for (auto it = _data.begin() + 2; it < _data.end(); ++it) {
            ss << static_cast<unsigned int>(*it);
        }
        ret.append(ss.str());
    }
    ret.append("] (");
    ret.append(std::to_string(_data.size()));
    ret.append(")");
    return ret;
}

uint16_t Message::crc16(std::vector<std::byte> packet)
{
    unsigned int crc = 0;
    for (std::byte b : packet) {
        unsigned int value = static_cast<unsigned int>(b);
        crc = crc ^ (value << 8);
        for (unsigned char bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}
}
