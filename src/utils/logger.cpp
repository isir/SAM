#include "logger.h"
#include <iostream>

Logger::Logger(QObject* parent)
    : QObject(parent)
    , MqttUser("SAM_Logger")
    , _info_file("/var/log/sam_info")
    , _err_file("/var/log/sam_err")
{
    _info_file.open(QIODevice::ReadWrite);
    _err_file.open(QIODevice::ReadWrite);
    QObject::connect(this, &Logger::message, this, &Logger::handle_message, Qt::QueuedConnection);
    QObject::connect(&_mqtt, &QMqttClient::connected, this, &Logger::mqtt_connected_callback);
}

void Logger::async_handle_message(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QByteArray msg_type;
    QByteArray msg_data = msg.toLocal8Bit() + " (" + QByteArray(context.file ? context.file : "") + ":" + QByteArray::number(context.line) + ")\r\n";

    switch (type) {
    case QtDebugMsg:
        msg_type = "Debug";
        break;
    case QtInfoMsg:
        msg_type = "Info";
        break;
    case QtWarningMsg:
        msg_type = "Warning";
        break;
    case QtCriticalMsg:
        msg_type = "Critical";
        break;
    case QtFatalMsg:
        msg_type = "Fatal";
        break;
    }
    emit message(msg_type, msg_data);
}

void Logger::handle_message(QByteArray type, const QByteArray& msg)
{
    QFile* log_file = &_info_file;

    if (type == "Critical" || type == "Fatal") {
        log_file = &_err_file;
    }

    QByteArray line = type + ": " + msg;
    log_file->write(line);
    log_file->flush();

    QString mqtt_topic_name = QString("sam/log/") + type.toLower();
    switch (_mqtt.state()) {
    case QMqttClient::Disconnected: {
        connect_to_mqtt_server();
    }
    /* fall through */
    case QMqttClient::Connecting: {
        QPair<QString, QByteArray> mqtt_msg(mqtt_topic_name, line);
        _mqtt_queue.append(mqtt_msg);
        if (_mqtt_queue.size() > 100) {
            _mqtt_queue.pop_front();
        }
        break;
    }
    case QMqttClient::Connected: {
        _mqtt.publish(mqtt_topic_name, line);
        break;
    }
    }
}

void Logger::dequeue_msgs()
{
    while (_mqtt_queue.size() > 0) {
        QPair<QString, QByteArray> mqtt_msg = _mqtt_queue.takeFirst();
        _mqtt.publish(mqtt_msg.first, mqtt_msg.second);
    }
}

void Logger::mqtt_connected_callback()
{
    dequeue_msgs();
}
