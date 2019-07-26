#include "param.h"

QList<BaseParam*> BaseParam::_param_list;
const QString BaseParam::_topic_suffix = "param/";

BaseParam::BaseParam(QString name, NamedObject* parent)
    : MqttUser(name, AUTOCONNECT, parent)
    , _storage(" ")
    , _value_changed(true)
{
    _topic_name = "";
    foreach (QString n, _parents) {
        _topic_name += n + "/";
    }
    _topic_name += _topic_suffix;
    _topic_name += _name;

    foreach (BaseParam* p, _param_list) {
        if (p->_topic_name == _topic_name) {
            throw std::runtime_error("A Parameter with the same name already exists");
        }
    }

    QObject::connect(&_mqtt, &QMqttClient::connected, this, &BaseParam::mqtt_connected_callback);

    _param_list.append(this);
}

BaseParam::~BaseParam()
{
}

bool BaseParam::changed()
{
    QMutexLocker lock(&_mutex);
    return _value_changed;
}

BaseParam* BaseParam::from_topic_name(QString topic_name)
{
    foreach (BaseParam* p, _param_list) {
        if (p->_topic_name == topic_name) {
            return p;
        }
    }
    return nullptr;
}

void BaseParam::message_received_callback(const QMqttMessage& msg)
{
    _assign_raw(msg.payload());
}

void BaseParam::mqtt_connected_callback()
{
    QMqttSubscription* sub = _mqtt.subscribe(_topic_name);
    QObject::disconnect(sub, &QMqttSubscription::messageReceived, this, &BaseParam::message_received_callback);
    QObject::connect(sub, &QMqttSubscription::messageReceived, this, &BaseParam::message_received_callback);

    QMutexLocker lock(&_mutex);
    _mqtt.publish(_topic_name, _storage, 1, true);
}
