#ifndef SAFE_STREAM_H
#define SAFE_STREAM_H

#include "logger.h"

namespace Log {
class SafeStream : public std::ostringstream {
public:
    SafeStream(Logger::MessageType t, std::string str = "");
    ~SafeStream();

    template <Logger::MessageType t>
    static SafeStream make(std::string str = "")
    {
        return SafeStream(t, str);
    }

private:
    Logger::MessageType _t;
    static std::mutex _mutex;
};
}

#define info Log::SafeStream::make<Log::Logger::INFO>
#define debug Log::SafeStream::make<Log::Logger::DEBUG>
#define warning Log::SafeStream::make<Log::Logger::WARNING>
#define critical Log::SafeStream::make<Log::Logger::CRITICAL>
#define fatal Log::SafeStream::make<Log::Logger::FATAL>

#endif // SAFE_STREAM_H
