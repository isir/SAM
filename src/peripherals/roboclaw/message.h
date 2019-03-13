#ifndef ROBOCLAWMESSAGE_H
#define ROBOCLAWMESSAGE_H

#include <QString>
#include <QByteArray>
#include <QMetaType>

namespace RoboClaw {
class Message {
    friend class Server;

public:
    explicit Message();
    explicit Message(uint8_t address, uint8_t command, QByteArray payload = QByteArray(), QString regexp = QString(), bool append_crc = true);

    QString toString() const;

private:
    uint16_t crc16(QByteArray packet);

    QByteArray _data;
    QByteArray _answer;
    QString _regexp;
};
}

Q_DECLARE_METATYPE(RoboClaw::Message)

#endif // ROBOCLAWMESSAGE_H
