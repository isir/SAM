#include "logger.h"

Logger::Logger(QObject* parent)
    : QObject(parent)
    , _info_file("/var/log/sam_info")
    , _err_file("/var/log/sam_err")
    , _mqtt(MqttClient::instance())
{
    _info_file.open(QIODevice::ReadWrite);
    _err_file.open(QIODevice::ReadWrite);
    QObject::connect(this, &Logger::message, this, &Logger::handle_message, Qt::QueuedConnection);
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
    QString mqtt_topic_name = type.toLower();
    QFile* log_file = &_info_file;

    if (type == "Critical" || type == "Fatal") {
        log_file = &_err_file;
    }

    QByteArray line = type + ": " + msg;
    log_file->write(line);
    log_file->flush();
    if (_mqtt.state() == QMqttClient::Connected) {
        _mqtt.publish(mqtt_topic_name, line);
    }
}
