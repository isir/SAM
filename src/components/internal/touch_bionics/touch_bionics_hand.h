#ifndef TOUCHBIONICSHAND_H
#define TOUCHBIONICSHAND_H

#include "utils/interfaces/menu_user.h"
#include "utils/serial_port.h"
#include <memory>

/**
 * @brief The TouchBionicsHand class allows to control the TouchBionics hands.
 */
class TouchBionicsHand : public MenuUser {
public:
    enum ACTION {
        STOP,
        THUMB_CLOSING,
        THUMB_OPENING,
        FOREFINGER_CLOSING,
        FOREFINGER_OPENING,
        MIDDLEFINGER_CLOSING,
        MIDDLEFINGER_OPENING,
        RINGFINGER_CLOSING,
        RINGFINGER_OPENING,
        LITTLEFINGER_CLOSING,
        LITTLEFINGER_OPENING,
        PINCH_CLOSING,
        PINCH_OPENING,
        HAND_CLOSING,
        HAND_OPENING,
        THUMB_EXT_CLOSING,
        THUMB_EXT_OPENING,
        THUMB_INT_CLOSING,
        THUMB_INT_OPENING,
        PINCH_CLOSING_ALL,
        PINCH_OPENING_ALL,
        HAND_CLOSING_ALL,
        HAND_OPENING_ALL,
        ALL_BUT_PINCH_CLOSING,
        ALL_BUT_PINCH_OPENING,
        TRIPLE_PINCH_CLOSING,
        TRIPLE_PINCH_OPENING,
    };

    enum POSTURE {
        HAND_POSTURE,
        PINCH_POSTURE,
        TRIPLE_PINCH_POSTURE,
        ONLY_THUMB_POSTURE,
        GLOVE_POSTURE = 24,
    };

    TouchBionicsHand();
    ~TouchBionicsHand();

    void init_sequence();
    void move(int action);
    void setPosture(POSTURE posture);

    int speed() { return _speed; }
    void set_speed(int new_speed);

    void take_ownership() { _sp.take_ownership(); }
    void release_ownership() { _sp.release_ownership(); }

private:
    int _speed;
    char _cmd[32];
    int _last_action;
    int _count;
    const int _NB_OF_CMD_TO_RESEND = 3;

    SerialPort _sp;
};

#endif // TOUCHBIONICSHAND_H
