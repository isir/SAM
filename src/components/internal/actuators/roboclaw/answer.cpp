#include "answer.h"
#include "message.h"

namespace RC {
namespace Answer {
    BaseAnswer::~BaseAnswer()
    {
    }

    std::vector<std::byte> BaseAnswer::format(std::vector<std::byte> v)
    {
        return v;
    }

    ExactMatch::ExactMatch(std::vector<std::byte> pattern)
        : _pattern(pattern)
    {
    }

    bool ExactMatch::try_match(std::vector<std::byte> v, std::vector<std::byte>)
    {
        return v == _pattern;
    }

    bool EndsWithCRC::try_match(std::vector<std::byte> v, std::vector<std::byte> command)
    {
        if (v.size() > 2 && command.size() >= 2) {
            v.insert(v.begin(), command.begin(), command.begin() + 2);
            uint16_t crc = Message::crc16(std::vector<std::byte>(v.begin(), v.end() - 2));
            uint16_t hi, lo;
            hi = static_cast<uint16_t>(*(v.end() - 2));
            lo = static_cast<uint16_t>(*(v.end() - 1));
            uint16_t msg_crc = static_cast<uint16_t>(hi << 8) | lo;
            return crc == msg_crc;
        }
        return false;
    }

    std::vector<std::byte> EndsWithCRC::format(std::vector<std::byte> v)
    {
        return std::vector<std::byte>(v.begin(), v.end() - 2);
    }
}
}
