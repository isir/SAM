#ifndef LOGGER_H
#define LOGGER_H

#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <map>
#include <memory>
#include <queue>
#include <sstream>

namespace Log {
class Logger : public ThreadedLoop, public MqttUser {
public:
    enum MessageType { INFO,
        DEBUG,
        WARNING,
        CRITICAL,
        FATAL };

    static Logger& instance();

    void enqueue(MessageType t, std::string s);

private:
    explicit Logger();
    ~Logger() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::string from_type(MessageType t);

    std::queue<std::pair<MessageType, std::string>> _queue;

    bool _log_to_file;
    bool _log_to_mqtt;
};
}

#endif // LOGGER_H
