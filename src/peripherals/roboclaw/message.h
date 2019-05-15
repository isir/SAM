#ifndef ROBOCLAWMESSAGE_H
#define ROBOCLAWMESSAGE_H

#include <QByteArray>
#include <QMetaType>
#include <QString>

namespace RC {
class Message {
    friend class Client;

public:
    explicit Message();
    explicit Message(uint8_t address, uint8_t command, QByteArray payload = QByteArray(), QString regexp = QString(), bool append_crc = true);
    Message(const Message& other)
    {
        _data = other._data;
        _answer = other._answer;
        _regexp = other._regexp;
    }

    ~Message() {}

    QByteArray data() const { return _data; }
    QString regexp() const { return _regexp; }

    QString toString() const;

private:
    uint16_t crc16(QByteArray packet);

    QByteArray _data;
    QByteArray _answer;
    QString _regexp;
};
}

Q_DECLARE_METATYPE(RC::Message)

#endif // ROBOCLAWMESSAGE_H
