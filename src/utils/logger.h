#ifndef LOGGER_H
#define LOGGER_H

#include "ui/mqtt_user.h"
#include <QFile>
#include <QObject>
#include <memory>

class Logger : public QObject, public MqttUser {
    Q_OBJECT
public:
    explicit Logger(QObject* parent = nullptr);
    void async_handle_message(QtMsgType type, const QMessageLogContext& context, const QString& msg);

public slots:
    void handle_message(QByteArray type, const QByteArray& msg);

private:
    void dequeue_msgs();

    QList<QPair<QString, QByteArray>> _mqtt_queue;
    QFile _info_file;
    QFile _err_file;

private slots:
    void mqtt_connected_callback();

signals:
    void message(QByteArray type, const QByteArray& msg);
};

#endif // LOGGER_H
