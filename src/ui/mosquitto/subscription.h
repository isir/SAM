#ifndef MOSQUITTO_SUBSCRIPTION_H
#define MOSQUITTO_SUBSCRIPTION_H

#include <string>
#include <map>
#include <mutex>
#include <functional>

namespace Mosquitto {

class Subscription {
    friend class ConnectHelper;

public:
    Subscription(std::string pattern);

    inline std::string pattern()
    {
        return _pattern;
    }

    inline bool add_callback(void* object, std::function<void(std::string)> callback)
    {
        std::lock_guard<std::mutex> lock(_mtx);
        return _callbacks.insert({ object, callback }).second;
    }

    void remove_callbacks(void* object);

private:
    void handle_message_received(std::string payload);

    std::string _pattern;
    std::map<void*, std::function<void(std::string)>> _callbacks;
    std::mutex _mtx;
};

}

#endif // MOSQUITTO_SUBSCRIPTION_H
