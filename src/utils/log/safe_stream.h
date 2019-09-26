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
};
}

#define info(s) Log::SafeStream::make<Log::Logger::INFO>(s)
#define debug(s) Log::SafeStream::make<Log::Logger::DEBUG>(s)
#define warning(s) Log::SafeStream::make<Log::Logger::WARNING>(s)
#define critical(s) Log::SafeStream::make<Log::Logger::CRITICAL>(s)
#define fatal(s) Log::SafeStream::make<Log::Logger::FATAL>(s)

#endif // SAFE_STREAM_H
