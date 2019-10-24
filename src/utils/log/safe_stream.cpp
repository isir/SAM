#include "safe_stream.h"

namespace Log {

SafeStream::SafeStream(Logger::MessageType t, std::string str)
    : std::ostringstream(str)
    , _t(t)
{
}

SafeStream::SafeStream(const SafeStream&& ss)
    : SafeStream(ss._t, ss.str())
{
}

SafeStream::~SafeStream()
{
    Logger::instance().enqueue(_t, str());
}
}
