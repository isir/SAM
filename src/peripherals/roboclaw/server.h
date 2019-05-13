#ifndef ROBOCLAWSERVER_H
#define ROBOCLAWSERVER_H

#include "client.h"
#include "message.h"
#include <QList>
#include <QObject>
#include <QPair>
#include <QSerialPort>

namespace RoboClaw {
class Server : public QObject {
    Q_OBJECT
public:
    Server(QString port_name, int baudrate);

    QString port_name();
    int baudrate();
    void register_client(Client* client, Qt::ConnectionType connection = Qt::AutoConnection) const;

public slots:
    void write_msg(Client* client, Message msg);

private:
    QSerialPort _sp;
    QByteArray _rcv_buffer;
    QList<QPair<Client*, Message>> _pending_messages;

private slots:
    void on_receive();
    void on_client_timeout();

signals:
    void answer_received(QByteArray data);
};
}

#endif // ROBOCLAWSERVER_H
