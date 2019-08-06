#ifndef MOSQUITTO_CLIENT_H
#define MOSQUITTO_CLIENT_H

#include "library_version.h"
#include "library_wrapper.h"
#include "connect_helper.h"

namespace Mosquitto {

class Client {
public:
    enum QOS {
        QoS0 = 0,
        QoS1 = 1,
        QoS2 = 2
    };

    enum SyncMode {
        SYNC,
        ASYNC
    };

    Client();
    ~Client();

    static LibraryVersion lib_version();

    void set_will(std::string topic, std::string payload, QOS qos = QoS0, bool retain = true);
    void clear_will();

    void set_login_info(std::string username, std::string password);

    void process();

    std::shared_ptr<Subscription> subscribe(std::string pattern, QOS qos = QoS0);
    void publish(std::string topic, const void* payload, unsigned int payload_len, QOS qos = QoS0, bool retain = false);
    inline void publish(std::string topic, std::string payload, QOS qos = QoS0, bool retain = false)
    {
        publish(topic, payload.c_str(), payload.length(), qos, retain);
    }

private:
    static LibraryWrapper _lib_wrapper;
    static ConnectHelper _con_helper;
};

} // namespace Mosquitto

#endif // MOSQUITTO_CLIENT_H
