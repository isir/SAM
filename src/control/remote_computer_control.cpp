#include "remote_computer_control.h"
#include "utils/check_ptr.h"

RemoteComputerControl::RemoteComputerControl(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("remote_computer_control", .01)
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand)) {
        throw std::runtime_error("Remote Computer Control is missing components");
    }

    _menu->set_description("Remote control with keyboard");
    _menu->set_code("key");
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.hand->menu());
}

RemoteComputerControl::~RemoteComputerControl()
{
    stop_and_join();
}

bool RemoteComputerControl::setup()
{
    _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);

    _robot->joints.hand->setPosture(TouchBionicsHand::HAND_POSTURE);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    _robot->joints.hand->set_speed(5);

    _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    _robot->joints.hand->move(TouchBionicsHand::HAND_OPENING);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    _robot->joints.hand->move(TouchBionicsHand::THUMB_INT_CLOSING);

    _robot->joints.elbow_flexion->calibrate();
    _robot->joints.wrist_pronation->set_encoder_position(0);

    _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    _robot->joints.hand->move(TouchBionicsHand::HAND_OPENING_ALL);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

void RemoteComputerControl::loop(double, clock::time_point)
{
}

void RemoteComputerControl::cleanup()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
}
