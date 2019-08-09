#ifndef PARAM_H
#define PARAM_H

#include "ui/mqtt_user.h"
#include "utils/named_object.h"
#include <atomic>
#include <mutex>
#include <sstream>

class BaseParam : public MqttUser, public NamedObject {
public:
    BaseParam(std::string name, NamedObject* parent = nullptr);
    virtual ~BaseParam() override = 0;

    bool changed();

    static BaseParam* from_topic_name(std::string topic_name);

protected:
    void _assign_raw(std::string v)
    {
        std::lock_guard<std::mutex> lock(_storage_mutex);
        if (v != _storage) {
            _storage = v;
            _value_changed = true;
        }
    }

    template <typename T>
    void _assign(T v)
    {
        T old_value = _to<T>();

        std::stringstream stream;
        stream << v;

        {
            _assign_raw(stream.str());
        }

        if (v != old_value || _first_assignment) {
            _value_changed = true;
            _first_assignment = false;
            _mqtt.publish(_topic_name, _storage, Mosquittopp::Client::QoS1);
        }
    }

    template <typename T>
    T _to()
    {
        T ret = T(0);

        {
            std::lock_guard<std::mutex> lock(_storage_mutex);
            std::stringstream stream(_storage);
            stream >> ret;
        }
        _value_changed = false;

        return ret;
    }

    std::string _storage;
    std::mutex _storage_mutex;

private:
    bool _first_assignment;

    static std::vector<BaseParam*> _param_list;
    static std::mutex _param_list_mutex;

    static const std::string _topic_prefix;

    std::string _topic_name;
    std::atomic<bool> _value_changed;
};

template <typename T>
class Param : public BaseParam {
public:
    Param(std::string name, NamedObject* parent)
        : BaseParam(name, parent)
    {
    }

    Param(std::string name, NamedObject* parent, T default_value)
        : BaseParam(name, parent)
    {
        assign(default_value);
    }

    ~Param() override
    {
    }

    inline void operator=(T v)
    {
        _assign<T>(v);
    }

    inline void assign(T v)
    {
        _assign<T>(v);
    }

    inline T to()
    {
        return _to<T>();
    }

    operator T()
    {
        return _to<T>();
    }

    inline T operator()()
    {
        return _to<T>();
    }
};

#endif // PARAM_H
