#include "subscription.h"

namespace Mosquitto {

Subscription::Subscription(std::string pattern)
    : _pattern(pattern)
{
}

void Subscription::remove_callbacks(void* object)
{
    std::lock_guard<std::mutex> lock(_mtx);
    _callbacks.erase(object);
}

void Subscription::handle_message_received(std::string payload)
{
    std::lock_guard<std::mutex> lock(_mtx);
    for (auto cb : _callbacks) {
        cb.second(payload);
    }
}

}
