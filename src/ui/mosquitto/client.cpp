#include "client.h"
#include <stdexcept>

namespace Mosquitto {

LibraryWrapper Client::_lib_wrapper;
ConnectHelper Client::_con_helper;

Client::Client()
{
}

Client::~Client()
{
}

LibraryVersion Client::lib_version()
{
    int major, minor, revision;
    mosquitto_lib_version(&major, &minor, &revision);
    return LibraryVersion { major, minor, revision };
}

void Client::set_will(std::string topic, std::string payload, QOS qos, bool retain)
{
    mosquitto_will_set(_con_helper._token, topic.data(), static_cast<int>(payload.length()), reinterpret_cast<void*>(const_cast<char*>(payload.data())), qos, retain);
}

void Client::clear_will()
{
    mosquitto_will_clear(_con_helper._token);
}

void Client::set_login_info(std::string username, std::string password)
{
    mosquitto_username_pw_set(_con_helper._token, username.data(), password.data());
}

std::shared_ptr<Subscription> Client::subscribe(std::string pattern, QOS qos)
{
    std::lock_guard lock(_con_helper._sub_mutex);
    std::shared_ptr<Subscription> ptr;
    if (mosquitto_sub_topic_check(pattern.data()) == MOSQ_ERR_SUCCESS) {
        mosquitto_subscribe(_con_helper._token, nullptr, pattern.data(), qos);
        for (auto s : _con_helper._subscriptions) {
            if (s->pattern() == pattern) {
                return s;
            }
        }
        ptr = std::make_shared<Subscription>(pattern);
        _con_helper._subscriptions.push_back(ptr);
    }
    return ptr;
}

void Client::publish(std::string topic, const void* payload, unsigned int payload_len, QOS qos, bool retain)
{
    mosquitto_publish(_con_helper._token, nullptr, topic.c_str(), static_cast<int>(payload_len), payload, qos, retain);
}
}
