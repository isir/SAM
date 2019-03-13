#include "message.h"

RoboClaw::Message::Message() {

}

RoboClaw::Message::Message(uint8_t address, uint8_t command, QByteArray payload, QString regexp, bool append_crc) : _regexp(regexp)
{
    _data.push_back(address);
    _data.push_back(command);
    _data.push_back(payload);
    if(append_crc) {
        uint16_t crc = crc16(_data);
        _data.push_back(crc>>8);
        _data.push_back(crc&0xff);
    }
}

QString RoboClaw::Message::toString() const {
    QString ret('[');
    if(_data.size() >= 2) {
        ret.append(QString::number(_data[0])).append(',');
        ret.append(QString::number(_data[1]));
    }
    if(_data.size() >= 3) {
        ret.append(",0x").append(_data.mid(2).toHex());
    }
    ret.append("] (").append(QString::number(_data.length())).append(")");
    return ret;
}

uint16_t RoboClaw::Message::crc16(QByteArray packet) {
    uint16_t crc = 0;
    for (int byte = 0; byte < packet.size(); byte++) {
        crc = crc ^ (static_cast<uint16_t>(packet[byte]) << 8);
        for (unsigned char bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

