#ifndef ROBOCLAWSERVER_H
#define ROBOCLAWSERVER_H

#include <QObject>
#include <QSerialPort>
#include <QList>
#include <QPair>
#include "message.h"
#include "client.h"

namespace RoboClaw {
class Server : public QObject
{
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
    QList<QPair<Client*,Message> > _pending_messages;

private slots:
    void on_receive();

signals:
    void answer_received(QByteArray data);
};
}

#endif // ROBOCLAWSERVER_H
