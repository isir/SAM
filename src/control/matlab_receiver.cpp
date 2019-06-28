#include "matlab_receiver.h"
#include "utils/check_ptr.h"
#include <QNetworkDatagram>

MatlabReceiver::MatlabReceiver(std::shared_ptr<SAM::Components> robot, QObject* parent)
    : QObject(parent)
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand)) {
        throw std::runtime_error("Matlab Receiver is missing components");
    }

    _menu->set_description("Matlab receiver");
    _menu->set_code("mr");

    _settings.beginGroup("MatlabReceiver");
    int port = _settings.value("port", 45456).toInt();
    if (!_socket.bind(QHostAddress::AnyIPv4, port)) { //}, QUdpSocket::ShareAddress)) {
        qCritical() << "Failed to bind on port" << port;
    }
    QObject::connect(&_socket, &QUdpSocket::readyRead, this, &MatlabReceiver::socket_callback);

    _menu->add_item("Start", "start", [this](QString) { this->start(); });
    _menu->add_item("Stop", "stop", [this](QString) { this->stop(); });
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.hand->menu());
}

MatlabReceiver::~MatlabReceiver()
{
    stop();
}

void MatlabReceiver::start()
{
    _robot->joints.hand->take_ownership();
    QObject::disconnect(this, &MatlabReceiver::command_received, this, &MatlabReceiver::handle_command);
    QObject::connect(this, &MatlabReceiver::command_received, this, &MatlabReceiver::handle_command);

    _robot->joints.elbow_flexion->calibrate();
    _robot->joints.hand->init_sequence();

    qInfo() << "MatlabReceiver: Starting";
}

void MatlabReceiver::stop()
{
    QObject::disconnect(this, &MatlabReceiver::command_received, this, &MatlabReceiver::handle_command);
    _robot->joints.hand->release_ownership();
    _robot->joints.wrist_pronation->forward(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
}

void MatlabReceiver::socket_callback()
{
    while (_socket.hasPendingDatagrams()) {
        QNetworkDatagram datagram = _socket.receiveDatagram();
        if (datagram.data().size() >= 4) {
            emit command_received(static_cast<Command>(datagram.data().at(3)));
        }
    }
}

void MatlabReceiver::handle_command(Command c)
{
    qDebug() << static_cast<Command>(c);
    bool is_hand_command = true, is_wrist_command = false, is_elbow_command = false;
    switch (c) {
    case HAS_NOT_RECEIVE_FIRST_DATA:
        is_hand_command = false;
        break;
    case NONE:
        is_hand_command = false;
        break;
    case REINITIALIZE_HAND_POSITION:
        _robot->joints.hand->setPosture(TouchBionicsHand::HAND_POSTURE);
        break;
    case THUMB_UP:
        _robot->joints.hand->move(TouchBionicsHand::THUMB_OPENING);
        break;
    case THUMB_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::THUMB_CLOSING);
        break;
    case THUMB_FLEXION_UP:
        _robot->joints.hand->move(TouchBionicsHand::THUMB_OPENING);
        break;
    case THUMB_FLEXION_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::THUMB_CLOSING);
        break;
    case FOREFINGER_UP:
        _robot->joints.hand->move(TouchBionicsHand::FOREFINGER_OPENING);
        break;
    case FOREFINGER_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::FOREFINGER_CLOSING);
        break;
    case MIDDLE_FINGER_UP:
        _robot->joints.hand->move(TouchBionicsHand::MIDDLEFINGER_OPENING);
        break;
    case MIDDLE_FINGER_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::MIDDLEFINGER_CLOSING);
        break;
    case RING_FINGER_UP:
        _robot->joints.hand->move(TouchBionicsHand::RINGFINGER_OPENING);
        break;
    case RING_FINGER_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::RINGFINGER_CLOSING);
        break;
    case LITTLEFINGER_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::LITTLEFINGER_CLOSING);
        break;
    case LITTLEFINGER_UP:
        _robot->joints.hand->move(TouchBionicsHand::LITTLEFINGER_OPENING);
        break;
    case HAND_UP:
        _robot->joints.hand->move(TouchBionicsHand::HAND_OPENING);
        break;
    case HAND_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING);
        break;
    case PINCH_UP:
        _robot->joints.hand->move(TouchBionicsHand::PINCH_OPENING);
        break;
    case PINCH_DOWN:
        _robot->joints.hand->move(TouchBionicsHand::PINCH_CLOSING);
        break;
    case WRIST_UP:
        is_wrist_command = true;
        is_hand_command = false;
        _robot->joints.wrist_pronation->forward(60);
        break;
    case WRIST_DOWN:
        is_wrist_command = true;
        is_hand_command = false;
        _robot->joints.wrist_pronation->backward(60);
        break;
    case ELBOW_UP:
        is_elbow_command = true;
        is_hand_command = false;
        _robot->joints.elbow_flexion->set_velocity_safe(-30);
        break;
    case ELBOW_DOWN:
        is_elbow_command = true;
        is_hand_command = false;
        _robot->joints.elbow_flexion->set_velocity_safe(30);
        break;
    default:
        is_hand_command = false;
        break;
    }

    if (!is_hand_command) {
        _robot->joints.hand->move(TouchBionicsHand::STOP);
    }
    if (!is_wrist_command) {
        _robot->joints.wrist_pronation->forward(0);
    }
    if (!is_elbow_command) {
        _robot->joints.elbow_flexion->set_velocity_safe(0);
    }
}
