#include "server.h"

RoboClaw::Server::Server(QString port_name, int baudrate) : QObject()
{
    QObject::connect(&_sp,&QSerialPort::readyRead,this,&Server::on_receive);

    _sp.setPortName(port_name);
    _sp.setBaudRate(baudrate);
    _sp.open(QIODevice::ReadWrite);
}

QString RoboClaw::Server::port_name() {
    return _sp.portName();
}

int RoboClaw::Server::baudrate() {
    return _sp.baudRate();
}

void RoboClaw::Server::register_client(Client *client, Qt::ConnectionType connection) const {
    QObject::disconnect(client,&Client::send_msg,this,&Server::write_msg);
    QObject::connect(client,&Client::send_msg,this,&Server::write_msg,static_cast<Qt::ConnectionType>(connection | Qt::UniqueConnection));
}

void RoboClaw::Server::write_msg(Client* client, Message msg) {
    _pending_messages.append(QPair<Client*,Message>(client,msg));
    _sp.write(msg._data);
}

void RoboClaw::Server::on_receive() {
    if(_pending_messages.size() == 0)
        return;

    if(_sp.bytesAvailable() > 0)
        _rcv_buffer.append(_sp.readAll());

    Client *c = _pending_messages.first().first;
    Message &msg = _pending_messages.first().second;;
    QRegExp rx(msg._regexp);

    int pos = rx.indexIn(QString::fromLatin1(_rcv_buffer.data(),_rcv_buffer.length()));
    if(pos > -1) {
        if(rx.captureCount() > 0) {
            c->on_answer_received(_rcv_buffer.mid(rx.pos(1),rx.cap(1).length()));
        }

        _rcv_buffer.remove(0,rx.cap(0).length());
        _pending_messages.removeFirst();

        if(_pending_messages.size() > 0)
            on_receive();
    }
}
