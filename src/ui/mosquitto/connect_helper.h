#ifndef CONNECT_HELPER_H
#define CONNECT_HELPER_H

#include <thread>
#include <condition_variable>
#include <atomic>
#include <mutex>
#include <vector>
#include <queue>
#include "mosquitto.h"
#include "subscription.h"

namespace Mosquitto {

class ConnectHelper {
public:
    friend class Client;

    ConnectHelper();
    ~ConnectHelper();

    void handle_message_received(const mosquitto_message* msg);

private:
    struct mosquitto* _token;

    std::thread _thread;
    std::atomic<bool> _thread_condition;
    std::condition_variable _cv;

    std::queue<mosquitto_message> _queue;
    std::mutex _queue_mutex;
    std::vector<std::shared_ptr<Subscription>> _subscriptions;
    std::mutex _sub_mutex;
};

}

#endif // CONNECT_HELPER_H
