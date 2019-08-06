#include "safe_stream.h"

namespace Log {
std::mutex SafeStream::_mutex;

SafeStream::SafeStream(Logger::MessageType t, std::string str)
    : std::ostringstream(str)
    , _t(t)
{
}

SafeStream::~SafeStream()
{
    std::lock_guard<std::mutex> lock(_mutex);
    Logger::instance().enqueue(_t, str());
}
}
