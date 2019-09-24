#include "logger.h"
#include <fstream>

namespace Log {
Logger::Logger()
    : Worker("logger", Worker::Continuous)
    , _log_to_file(true)
    , _log_to_mqtt(true)
{
    do_work();
}

Logger::~Logger()
{
}

Logger& Logger::instance()
{
    static Logger l;
    return l;
}

void Logger::work()
{
    static std::ofstream os("/var/log/sam.log", std::ios::trunc | std::ios::out);

    std::queue<std::pair<MessageType, std::string>> queue_local;

    _queue_mutex.lock();
    queue_local.swap(_queue);
    _queue_mutex.unlock();

    while (!queue_local.empty()) {
        std::pair<MessageType, std::string> p = queue_local.front();
        std::string type_str = from_type(p.first);
        std::string type_str_upper(type_str);
        for (auto c : type_str) {
            c = static_cast<char>(toupper(c));
        }

        if (_log_to_file) {
            os << "[" << type_str_upper << "] " << p.second << std::endl;
        }

        if (_log_to_mqtt) {
            std::string topic_name = "sam/log/" + type_str;
            _mqtt.publish(topic_name, p.second);
        }

        queue_local.pop();
    }
    if (_log_to_file) {
        os.flush();
    }
}

void Logger::enqueue(MessageType t, std::string s)
{
    std::lock_guard lock(_queue_mutex);
    _queue.push(std::make_pair(t, s));
}

std::string Logger::from_type(MessageType t)
{
    switch (t) {
    case INFO:
        return "info";
    case DEBUG:
        return "debug";
    case WARNING:
        return "warning";
    case CRITICAL:
        return "critical";
    case FATAL:
        return "fatal";
    }
    return "";
}
}
