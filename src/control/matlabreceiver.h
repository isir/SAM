#ifndef MATLABRECEIVER_H
#define MATLABRECEIVER_H

#include "ui/consolemenu.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QObject>
#include <QUdpSocket>

class MatlabReceiver : public QObject {
    Q_OBJECT
public:
    enum Command {
        NONE = 0,
        THUMB_DOWN = 1,
        THUMB_UP = 2,
        FOREFINGER_DOWN = 3,
        FOREFINGER_UP = 4,
        MIDDLE_FINGER_DOWN = 5,
        MIDDLE_FINGER_UP = 6,
        RING_FINGER_DOWN = 7,
        RING_FINGER_UP = 8,
        LITTLEFINGER_DOWN = 9,
        LITTLEFINGER_UP = 10,
        PINCH_DOWN = 11,
        PINCH_UP = 12,
        HAND_DOWN = 13,
        HAND_UP = 14,
        WRIST_DOWN = 17,
        WRIST_UP = 18,
        ELBOW_UP = 21,
        ELBOW_DOWN = 22,
        THUMB_FLEXION_DOWN = 56,
        THUMB_FLEXION_UP = 55,
        HAS_NOT_RECEIVE_FIRST_DATA = 77,
        REINITIALIZE_HAND_POSITION = 42
    };

    explicit MatlabReceiver(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt, QObject* parent = nullptr);
    ~MatlabReceiver();

    ConsoleMenu& menu() { return _menu; }

private:
    QUdpSocket _socket;
    Settings _settings;
    SAM::Components _robot;
    ConsoleMenu _menu;

private slots:
    void start();
    void stop();

    void socket_callback();
    void handle_command(Command c);

signals:
    void command_received(Command c);
};

#endif // MATLABRECEIVER_H
