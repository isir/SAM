#ifndef PARAM_H
#define PARAM_H

#include "ui/mqtt_user.h"
#include "utils/named_object.h"
#include <QMutex>
#include <QObject>
#include <QTextStream>

class BaseParam : public QObject, public MqttUser {
    Q_OBJECT
public:
    BaseParam(QString name, NamedObject* parent = nullptr);
    virtual ~BaseParam();

    bool changed();

    static BaseParam* from_topic_name(QString topic_name);

protected:
    void _assign_raw(const QByteArray& v)
    {
        QMutexLocker lock(&_mutex);
        if (v != _storage) {
            _storage = v;
            _value_changed = true;
            emit value_changed();
        }
    }

    template <typename T>
    void _assign(T v)
    {
        T old_value = _to<T>();

        QMutexLocker lock(&_mutex);

        _storage.clear();
        QTextStream stream(&_storage, QIODevice::ReadWrite);
        stream << v;

        if (v != old_value) {
            _value_changed = true;
            if (_mqtt.state() == QMqttClient::Connected)
                _mqtt.publish(_topic_name, _storage, 1, true);
            emit value_changed();
        }
    }

    template <typename T>
    T _to()
    {
        T ret;

        QMutexLocker lock(&_mutex);

        if (!_storage.isEmpty()) {
            QTextStream stream(_storage, QIODevice::ReadOnly);
            stream >> ret;
            _value_changed = false;
        }

        return ret;
    }

    QByteArray _storage;
    QMutex _mutex;

private:
    static QList<BaseParam*> _param_list;

    QString _topic_name;
    static const QString _topic_suffix;
    bool _value_changed;

signals:
    void value_changed();

private slots:
    void message_received_callback(const QMqttMessage& msg);
    void mqtt_connected_callback();
};

template <typename T>
class Param : public BaseParam {
public:
    Param(QString name, NamedObject* parent)
        : BaseParam(name, parent)
    {
    }
    Param(QString name, NamedObject* parent, T default_value)
        : BaseParam(name, parent)
    {
        assign(default_value);
    }
    ~Param() = default;

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
