#ifndef PARAM_H
#define PARAM_H

#include "utils/interfaces/mqtt_user.h"
#include "utils/named_object.h"
#include <atomic>
#include <mutex>
#include <sstream>

class BaseParam : public MqttUser, public NamedObject {
public:
    enum Mode {
        Read = 1,
        Write = 2,
        ReadOnly = Read,
        WriteOnly = Write,
        ReadWrite = Read | Write
    };

    BaseParam(std::string name, Mode mode = ReadWrite, NamedObject* parent = nullptr);
    virtual ~BaseParam() override = 0;

    bool changed();

    static BaseParam* from_topic_name(std::string topic_name);

protected:
    void _assign_raw(std::string v);

    template <typename T>
    void _assign(T v)
    {
        if (_mode & Write) {
            std::stringstream stream;
            stream << v;

            _assign_raw(stream.str());

            if (_value_changed || _first_assignment) {
                _first_assignment = false;
                _mqtt.publish(_topic_name, _storage, Mosquittopp::Client::QoS1, true);
            }
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
    Mode _mode;
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
    Param(std::string name, Mode mode, NamedObject* parent)
        : BaseParam(name, mode, parent)
    {
    }

    Param(std::string name, Mode mode, NamedObject* parent, T default_value)
        : BaseParam(name, mode, parent)
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
