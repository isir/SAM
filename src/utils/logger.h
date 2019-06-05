#ifndef LOGGER_H
#define LOGGER_H

#include <QFile>
#include <QObject>
#include <memory>

class Logger : public QObject {
    Q_OBJECT
public:
    explicit Logger(QObject* parent = nullptr);
    void async_handle_message(QtMsgType type, const QMessageLogContext& context, const QString& msg);

public slots:
    void handle_message(QByteArray type, const QByteArray& msg);

private:
    QFile _info_file;
    QFile _err_file;

signals:
    void message(QByteArray type, const QByteArray& msg);
};

#endif // LOGGER_H
