#include "touch_bionics_hand.h"

#include <QDebug>
#include <QThread>

TouchBionicsHand::TouchBionicsHand(std::shared_ptr<QMqttClient> mqtt)
    : _settings("TouchBionics")
    , _menu(mqtt)
{
    _sp.setPortName(_settings.value("port_name", "/dev/touchbionics").toString());
    _sp.setBaudRate(115200);
    if (_sp.open(QIODevice::ReadWrite)) {
        qDebug() << "### TOUCHBIONICS : Hand port opened";
    } else {
        throw std::runtime_error("/dev/" + _sp.portName().toStdString() + ": " + _sp.errorString().toStdString());
    }

    _speed = _settings.value("speed", 3).toInt();
    if (_speed < 0) {
        _speed = 0;
    } else if (_speed > 9) {
        _speed = 9;
    }

    _menu.set_title(QString("TouchBionics Hand "));
    _menu.set_code(QString("tb"));

    _menu.addItem(ConsoleMenuItem("Hand posture", "h", [this](QString) { this->setPosture(HAND_POSTURE); }));
    _menu.addItem(ConsoleMenuItem("Close hand", "ch", [this](QString) { this->move(HAND_CLOSING); }));
    _menu.addItem(ConsoleMenuItem("Open hand", "oh", [this](QString) { this->move(HAND_OPENING); }));
    _menu.addItem(ConsoleMenuItem("Pinch posture", "p", [this](QString) { this->setPosture(PINCH_POSTURE); }));
    _menu.addItem(ConsoleMenuItem("Close pinch", "cp", [this](QString) { this->move(PINCH_CLOSING); }));
    _menu.addItem(ConsoleMenuItem("Open pinch ", "op", [this](QString) { this->move(PINCH_OPENING); }));
    _menu.addItem(ConsoleMenuItem("Triple Pinch posture", "tp", [this](QString) { this->setPosture(TRIPLE_PINCH_POSTURE); }));
    _menu.addItem(ConsoleMenuItem("Close triple pinch", "ctp", [this](QString) { this->move(TRIPLE_PINCH_CLOSING); }));
    _menu.addItem(ConsoleMenuItem("Open triple pinch ", "otp", [this](QString) { this->move(TRIPLE_PINCH_OPENING); }));
    _menu.addItem(ConsoleMenuItem("Open forefinger ", "off", [this](QString) { this->move(FOREFINGER_OPENING); }));
}

TouchBionicsHand::~TouchBionicsHand()
{
}

void TouchBionicsHand::init_sequence()
{
    setPosture(TouchBionicsHand::HAND_POSTURE);
    QThread::sleep(1);

    setSpeed(5);
    move(TouchBionicsHand::HAND_CLOSING);
    QThread::msleep(500);

    move(TouchBionicsHand::HAND_OPENING);
    QThread::sleep(1);
    move(TouchBionicsHand::THUMB_INT_CLOSING);
}

void TouchBionicsHand::setPosture(POSTURE posture)
{
    QByteArray cmd = "QG" + QByteArray::number(posture) + "\r";
    _sp.write(cmd);

    switch (posture) {
    case HAND_POSTURE:
        qDebug() << "TouchBionics: Setting Hand Posture";
        break;
    case PINCH_POSTURE:
        qDebug() << "TouchBionics: Setting Pinch Posture";
        break;
    case TRIPLE_PINCH_POSTURE:
        qDebug() << "TouchBionics: Setting Triple Pinch Posture";
        break;
    case GLOVE_POSTURE:
        qDebug() << "TouchBionics: Setting Glove Posture";
        break;
    default:
        qWarning() << "TouchBionics: Unknown Posture";
        break;
    }

    QThread::msleep(10);
    move(HAND_OPENING_ALL);
}

void TouchBionicsHand::move(int action)
{

    // If changing direction, send 0 before
    if (((_last_action + action) % 2 == 1) && _last_action != 0) {
        if (_last_action % 2 == 0)
            _sp.write("+0+0+0+0+0+0\r", 13);
        else
            _sp.write("-0-0-0-0-0-0\r", 13);
    }

    if (_last_action != action) {
        _count = 0;
        switch (action) {
        case THUMB_CLOSING: // thumb flexion + rotation
            sprintf(_cmd, "-%d-0-0-0-0-%d\r", _speed, _speed);
            break;
        case THUMB_OPENING:
            sprintf(_cmd, "+%d+0+0+0+0+%d\r", _speed, _speed);
            break;
        case THUMB_EXT_CLOSING: // thumb flexion
            sprintf(_cmd, "-%d-0-0-0-0-0\r", _speed);
            break;
        case THUMB_EXT_OPENING:
            sprintf(_cmd, "+%d+0+0+0+0+0\r", _speed);
            break;
        case THUMB_INT_CLOSING: // thumb rotation
            sprintf(_cmd, "-0-0-0-0-0-%d\r", _speed);
            break;
        case THUMB_INT_OPENING:
            sprintf(_cmd, "+0+0+0+0+0+%d\r", _speed);
            break;
        case FOREFINGER_CLOSING: // Forefinger flexion
            sprintf(_cmd, "-0-%d-0-0-0-0\r", _speed);
            break;
        case FOREFINGER_OPENING:
            sprintf(_cmd, "+0+%d+0+0+0+0\r", _speed);
            break;
        case MIDDLEFINGER_CLOSING: // Middle finger flexion
            sprintf(_cmd, "-0-0-%d-0-0-0\r", _speed);
            break;
        case MIDDLEFINGER_OPENING:
            sprintf(_cmd, "+0+0+%d+0+0+0\r", _speed);
            break;
        case RINGFINGER_CLOSING: // Annulaire
            sprintf(_cmd, "-0-0-0-%d-0-0\r", _speed);
            break;
        case RINGFINGER_OPENING:
            sprintf(_cmd, "+0+0+0+%d+0+0\r", _speed);
            break;
        case LITTLEFINGER_CLOSING: // Auriculaire
            sprintf(_cmd, "-0-0-0-0-%d-0\r", _speed);
            break;
        case LITTLEFINGER_OPENING:
            sprintf(_cmd, "+0+0+0+0+%d+0\r", _speed);
            break;
        case PINCH_CLOSING: // Pinch but thumb rotation
            sprintf(_cmd, "-%d-%d-0-0-0-0\r", _speed, _speed);
            break;
        case PINCH_OPENING:
            sprintf(_cmd, "+%d+%d+0+0+0+0\r", _speed, _speed);
            break;
        case TRIPLE_PINCH_CLOSING: // Pinch but thumb rotation
            sprintf(_cmd, "-%d-%d-%d-0-0-0\r", _speed, _speed, _speed);
            break;
        case TRIPLE_PINCH_OPENING:
            sprintf(_cmd, "+%d+%d+%d+0+0+0\r", _speed, _speed, _speed);
            break;
        case HAND_CLOSING: // Hand but thumb rotation
            sprintf(_cmd, "-%d-%d-%d-%d-%d-0\r", _speed, _speed, _speed, _speed, _speed);
            break;
        case HAND_OPENING:
            sprintf(_cmd, "+%d+%d+%d+%d+%d+0\r", _speed, _speed, _speed, _speed, _speed);
            break;
        case PINCH_CLOSING_ALL: // Pinch
            sprintf(_cmd, "-%d-%d-0-0-0-%d\r", _speed, _speed, _speed);
            break;
        case PINCH_OPENING_ALL:
            sprintf(_cmd, "+%d+%d+0+0+0+%d\r", _speed, _speed, _speed);
            break;
        case HAND_CLOSING_ALL: // Hand
            sprintf(_cmd, "-%d-%d-%d-%d-%d-%d\r", _speed, _speed, _speed, _speed, _speed, _speed);
            break;
        case HAND_OPENING_ALL:
            sprintf(_cmd, "+%d+%d+%d+%d+%d+%d\r", _speed, _speed, _speed, _speed, _speed, _speed);
            break;
        case ALL_BUT_PINCH_CLOSING: // three last fingers
            sprintf(_cmd, "-0-0-%d-%d-%d-0\r", _speed, _speed, _speed);
            break;
        case ALL_BUT_PINCH_OPENING:
            sprintf(_cmd, "+0+0+%d+%d+%d+0\r", _speed, _speed, _speed);
            break;
        default:
            action = 0;
            sprintf(_cmd, "+0+0+0+0+0+0\r");
            break;
        }
        _sp.write(_cmd, 13);
    } else {
        if (_count < _NB_OF_CMD_TO_RESEND) {
            _count++;
            _sp.write(_cmd, 13);
        }
    }

    _last_action = action;
}

ConsoleMenu& TouchBionicsHand::menu()
{
    return _menu;
}
