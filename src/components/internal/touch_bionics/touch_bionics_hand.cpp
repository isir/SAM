#include "touch_bionics_hand.h"

#include "utils/log/log.h"

TouchBionicsHand::TouchBionicsHand()
{
    _sp.open("/dev/touchbionics", B115200);
    _speed = 3;

    _menu->set_description("TouchBionics Hand ");
    _menu->set_code("tb");

    _menu->add_item("Hand posture", "h", [this](std::string) { this->setPosture(HAND_POSTURE); });
    _menu->add_item("Close hand", "ch", [this](std::string) { this->move(HAND_CLOSING); });
    _menu->add_item("Open hand", "oh", [this](std::string) { this->move(HAND_OPENING); });
    _menu->add_item("Pinch posture", "p", [this](std::string) { this->setPosture(PINCH_POSTURE); });
    _menu->add_item("Close pinch", "cp", [this](std::string) { this->move(PINCH_CLOSING); });
    _menu->add_item("Open pinch ", "op", [this](std::string) { this->move(PINCH_OPENING); });
    _menu->add_item("Triple Pinch posture", "tp", [this](std::string) { this->setPosture(TRIPLE_PINCH_POSTURE); });
    _menu->add_item("Close triple pinch", "ctp", [this](std::string) { this->move(TRIPLE_PINCH_CLOSING); });
    _menu->add_item("Open triple pinch ", "otp", [this](std::string) { this->move(TRIPLE_PINCH_OPENING); });
    _menu->add_item("Open forefinger ", "off", [this](std::string) { this->move(FOREFINGER_OPENING); });
}

TouchBionicsHand::~TouchBionicsHand()
{
}

void TouchBionicsHand::init_sequence()
{
    setPosture(TouchBionicsHand::HAND_POSTURE);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    set_speed(5);
    move(TouchBionicsHand::HAND_CLOSING);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    move(TouchBionicsHand::STOP);
}

void TouchBionicsHand::setPosture(POSTURE posture)
{
    std::string cmd = "QG";
    if (static_cast<int>(posture) < 10) {
        cmd += '0';
    }
    cmd += std::to_string(posture) + '\r';
    _sp.write(cmd);

    switch (posture) {
    case HAND_POSTURE:
        debug() << "TouchBionics: Setting Hand Posture";
        break;
    case PINCH_POSTURE:
        debug() << "TouchBionics: Setting Pinch Posture";
        break;
    case TRIPLE_PINCH_POSTURE:
        debug() << "TouchBionics: Setting Triple Pinch Posture";
        break;
    case GLOVE_POSTURE:
        debug() << "TouchBionics: Setting Glove Posture";
        break;
    default:
        warning() << "TouchBionics: Unknown Posture";
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    move(HAND_OPENING_ALL);
}

void TouchBionicsHand::set_speed(int new_speed)
{
    if (new_speed < 0) {
        _speed = 0;
    } else if (new_speed > 9) {
        _speed = 9;
    } else {
        _speed = new_speed;
    }
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
