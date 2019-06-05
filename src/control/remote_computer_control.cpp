#include "remote_computer_control.h"

RemoteComputerControl::RemoteComputerControl(SAM::Components robot)
    : BasicController(.01)
    , _robot(robot)
{
    _menu.set_title("Remote control with keyboard");
    _menu.set_code("key");
    _menu.addItem(_robot.wrist_pronosup->menu());
    _menu.addItem(_robot.elbow->menu());
    _menu.addItem(_robot.hand->menu());
}

RemoteComputerControl::~RemoteComputerControl()
{
    stop();
}

bool RemoteComputerControl::setup()
{
    _robot.buzzer->makeNoise(BuzzerConfig::SHORT_BUZZ);

    _robot.hand->setPosture(TouchBionicsHand::HAND_POSTURE);
    QThread::sleep(1);

    _robot.hand->setSpeed(5);
    _robot.hand->move(TouchBionicsHand::HAND_CLOSING);
    QThread::msleep(500);

    _robot.hand->move(TouchBionicsHand::HAND_OPENING);
    QThread::sleep(1);
    _robot.hand->move(TouchBionicsHand::THUMB_INT_CLOSING);

    _robot.elbow->calibrate();
    _robot.wrist_pronosup->set_encoder_position(0);

    _robot.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
    QThread::msleep(500);
    _robot.hand->move(TouchBionicsHand::HAND_OPENING_ALL);
    QThread::msleep(500);
    return true;
}

void RemoteComputerControl::loop(double, double)
{
}

void RemoteComputerControl::cleanup()
{
    _robot.elbow->forward(0);
    _robot.wrist_pronosup->forward(0);
    _robot.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
}
