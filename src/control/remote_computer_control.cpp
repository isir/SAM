#include "remote_computer_control.h"

RemoteComputerControl::RemoteComputerControl(std::shared_ptr<SAM::Components> robot)
    : BasicController(.01)
    , _robot(robot)
{
    _menu->set_description("Remote control with keyboard");
    _menu->set_code("key");
    _menu->add_item(_robot->joints.wrist_pronosup->menu());
    _menu->add_item(_robot->joints.elbow->menu());
    _menu->add_item(_robot->joints.hand->menu());
}

RemoteComputerControl::~RemoteComputerControl()
{
    stop();
}

bool RemoteComputerControl::setup()
{
    _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::SHORT_BUZZ);

    _robot->joints.hand->setPosture(TouchBionicsHand::HAND_POSTURE);
    QThread::sleep(1);

    _robot->joints.hand->setSpeed(5);
    _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING);
    QThread::msleep(500);

    _robot->joints.hand->move(TouchBionicsHand::HAND_OPENING);
    QThread::sleep(1);
    _robot->joints.hand->move(TouchBionicsHand::THUMB_INT_CLOSING);

    _robot->joints.elbow->calibrate();
    _robot->joints.wrist_pronosup->set_encoder_position(0);

    _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
    QThread::msleep(500);
    _robot->joints.hand->move(TouchBionicsHand::HAND_OPENING_ALL);
    QThread::msleep(500);
    return true;
}

void RemoteComputerControl::loop(double, double)
{
}

void RemoteComputerControl::cleanup()
{
    _robot->joints.elbow->forward(0);
    _robot->joints.wrist_pronosup->forward(0);
    _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
}
