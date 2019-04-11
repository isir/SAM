#include "remotecomputercontrol.h"

RemoteComputerControl::RemoteComputerControl()
    : BasicController(.01)
    , _buzzer(29)
    , _pronosup(PronoSupination::instance())
    , _osmer(OsmerElbow::instance())
    , _hand(TouchBionicsHand::instance())
{
    _menu.set_title("Remote control with keyboard");
    _menu.set_code("key");
    _menu.addItem(_pronosup.menu());
    _menu.addItem(_osmer.menu());
    _menu.addItem(_hand.menu());
}

RemoteComputerControl::~RemoteComputerControl()
{
}

bool RemoteComputerControl::setup()
{
    _buzzer.makeNoise(BuzzerConfig::SHORT_BUZZ);

    _hand.setPosture(TouchBionicsHand::HAND_POSTURE);
    QThread::sleep(1);

    _hand.setSpeed(5);
    _hand.move(TouchBionicsHand::HAND_CLOSING);
    QThread::msleep(500);

    _hand.move(TouchBionicsHand::HAND_OPENING);
    QThread::sleep(1);
    _hand.move(TouchBionicsHand::THUMB_INT_CLOSING);

    _osmer.calibration();
    _pronosup.set_encoder_position(0);

    _hand.move(TouchBionicsHand::HAND_CLOSING_ALL);
    QThread::msleep(500);
    _hand.move(TouchBionicsHand::HAND_OPENING_ALL);
    QThread::msleep(500);
    return true;
}

void RemoteComputerControl::loop(double, double)
{
}

void RemoteComputerControl::cleanup()
{
    _osmer.forward(0);
    _pronosup.forward(0);
    _hand.move(TouchBionicsHand::HAND_CLOSING_ALL);
}
