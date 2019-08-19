#include "param.h"

std::vector<BaseParam*> BaseParam::_param_list;
std::mutex BaseParam::_param_list_mutex;
const std::string BaseParam::_topic_prefix = "param/";

BaseParam::BaseParam(std::string name, NamedObject* parent)
    : NamedObject(name, parent)
    , _storage(" ")
    , _first_assignment(true)
    , _value_changed(true)
{
    _topic_name = _topic_prefix + full_name();

    {
        std::string param_list_string;
        std::lock_guard<std::mutex> lock(_param_list_mutex);
        for (BaseParam* p : _param_list) {
            if (p) {
                param_list_string += p->_topic_name + ",";
                if (p->_topic_name == _topic_name) {
                    throw std::runtime_error("A Parameter with the same name (" + _topic_name + ") already exists");
                }
            }
        }
        _param_list.push_back(this);
        param_list_string += _topic_name;
        _mqtt.publish(_topic_prefix + "/list", param_list_string);
    }

    auto cb = [this](Mosquittopp::Message msg) {
        _assign_raw(msg.payload());
    };

    _mqtt.subscribe(_topic_name, Mosquittopp::Client::QoS1)->add_callback(this, cb);
}

BaseParam::~BaseParam()
{
    _mqtt.subscribe(_topic_name)->remove_callbacks(this);
    std::lock_guard<std::mutex> lock(_param_list_mutex);
    for (auto it = _param_list.begin(); it < _param_list.end(); ++it) {
        if (*it == this) {
            _param_list.erase(it);
        }
    }
}

bool BaseParam::changed()
{
    return _value_changed;
}

BaseParam* BaseParam::from_topic_name(std::string topic_name)
{
    std::lock_guard<std::mutex> lock(_param_list_mutex);
    for (BaseParam* p : _param_list) {
        if (p->_topic_name == topic_name) {
            return p;
        }
    }
    return nullptr;
}

void BaseParam::_assign_raw(std::string v)
{
    std::lock_guard<std::mutex> lock(_storage_mutex);
    if (v != _storage) {
        _storage = v;
        _value_changed = true;
    }
}
