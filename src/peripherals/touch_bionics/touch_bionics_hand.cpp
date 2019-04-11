#include "touch_bionics_hand.h"

#include <QDebug>

/**
 * \brief TouchBionicsHand::TouchBionicsHand Constructor
 * \param filename
 */
TouchBionicsHand::TouchBionicsHand(char* filename)
{
    _f = -1;
    _f = open(filename, O_RDWR | O_NOCTTY | O_NDELAY);
    if (_f == -1)
        qCritical() << "TOUCHBIONICS ERROR : Unable to open port.";

    struct termios options;
    tcgetattr(_f, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(_f, TCIFLUSH);
    tcsetattr(_f, TCSANOW, &options);
    _speed = 3;
    qDebug() << "### TOUCHBIONICS : Hand port opened";

    _menu.set_title(QString("TouchBionics Hand "));
    _menu.set_code(QString("tb"));

    _menu.addItem(ConsoleMenuItem("Hand posture", "h", [this](QString) { this->setPosture(HAND_POSTURE); }));
    _menu.addItem(ConsoleMenuItem("Close hand", "hc", [this](QString) { this->move(HAND_CLOSING); }));
    _menu.addItem(ConsoleMenuItem("Open hand", "ho", [this](QString) { this->move(HAND_OPENING); }));
    _menu.addItem(ConsoleMenuItem("Pinch posture", "p", [this](QString) { this->setPosture(PINCH_POSTURE); }));
    _menu.addItem(ConsoleMenuItem("Close pinch", "cp", [this](QString) { this->move(PINCH_CLOSING); }));
    _menu.addItem(ConsoleMenuItem("Open pinch ", "op", [this](QString) { this->move(PINCH_OPENING); }));
}

/**
 * \brief TouchBionicsHand::~TouchBionicsHand Destructor
 */
TouchBionicsHand::~TouchBionicsHand()
{
    if (_f != -1) {
        close(_f);
    }
}

/**
 * \brief TouchBionicsHand::setPosture Changes the hand's mode of the TouchBionics.
 * Look at the available postures in their amazing doc if you need others.
 * \param posture
 */
void TouchBionicsHand::setPosture(POSTURE posture)
{
    sprintf(_cmd, "QG%2d\r", posture);
    write(_f, _cmd, 5);

    switch (posture) {
    case HAND_POSTURE:
        qDebug("### TOUCHBIONICS : Setting Hand Posture");
        break;
    case PINCH_POSTURE:
        qDebug("### TOUCHBIONICS : Setting Pinch Posture");
        break;
    case TRIPLE_PINCH_POSTURE:
        qDebug("### TOUCHBIONICS : Setting triple Pinchh Posture");
        break;
    case GLOVE_POSTURE:
        qDebug("### TOUCHBIONICS : Setting Glove Posture");
        break;
    default:
        qDebug("### TOUCHBIONICS WARNING : Uncoded posture");
        break;
    }

    usleep(10 * 1000);
    move(HAND_OPENING_ALL);
}

/**
 * \brief TouchBionicsHand::move Performs an action on touchbionics hand.
 * \param action
 */
void TouchBionicsHand::move(int action)
{

    // If changing direction, send 0 before
    if (((_last_action + action) % 2 == 1) && _last_action != 0) {
        if (_last_action % 2 == 0)
            write(_f, "+0+0+0+0+0+0\r", 13);
        else
            write(_f, "-0-0-0-0-0-0\r", 13);
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
            write(_f, _cmd, 13);
            break;
        }
        write(_f, _cmd, 13);
    } else {
        if (_count < _NB_OF_CMD_TO_RESEND) {
            _count++;
            write(_f, _cmd, 13);
        }
    }

    _last_action = action;
}

ConsoleMenu& TouchBionicsHand::menu()
{
    return _menu;
}
