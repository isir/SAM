#include "connect_helper.h"

namespace Mosquitto {

static void message_callback(struct mosquitto*, void* obj, const mosquitto_message* msg)
{
    ConnectHelper* c = reinterpret_cast<ConnectHelper*>(obj);
    c->handle_message_received(msg);
}

ConnectHelper::ConnectHelper()
    : _token(nullptr)
{
    _token = mosquitto_new(nullptr, true, reinterpret_cast<void*>(this));
    if (_token == nullptr) {
        throw std::runtime_error("Failed to allocate resources: " + std::to_string(errno));
    }
    mosquitto_message_callback_set(_token, message_callback);

    bool res = mosquitto_connect(_token, "127.0.0.1", 1883, 1) == MOSQ_ERR_SUCCESS;
    if (!res) {
        throw std::runtime_error("Failed to connect: " + std::to_string(errno));
    }
    mosquitto_loop_start(_token);
    _thread_condition = true;

    _thread = std::thread([this] {
        pthread_setname_np(pthread_self(), "mosquitto_helper");
        while (_thread_condition) {
            std::unique_lock<std::mutex> lock(_queue_mutex);
            _cv.wait(lock, [this] { return (_queue.size() > 0) || !_thread_condition; });

            if (_queue.empty()) {
                continue;
            }

            mosquitto_message msg = _queue.front();
            _queue.pop();
            lock.unlock();

            std::lock_guard lg(_sub_mutex);
            bool result;
            for (auto s : _subscriptions) {
                mosquitto_topic_matches_sub(s->pattern().data(), msg.topic, &result);
                if (result) {
                    s->handle_message_received(std::string(reinterpret_cast<char*>(msg.payload), static_cast<unsigned int>(msg.payloadlen)));
                }
            }
        }
    });
}

ConnectHelper::~ConnectHelper()
{
    if (_token) {
        mosquitto_disconnect(_token);
        mosquitto_loop_stop(_token, false);
        mosquitto_destroy(_token);
    }
    _thread_condition = false;
    _cv.notify_one();
    if (_thread.joinable()) {
        _thread.join();
    }
}

void ConnectHelper::handle_message_received(const mosquitto_message* msg)
{
    std::lock_guard lock(_queue_mutex);
    mosquitto_message dst;
    mosquitto_message_copy(&dst, msg);
    _queue.push(dst);
    _cv.notify_one();
}

}
