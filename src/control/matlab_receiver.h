#ifndef MATLAB_RECEIVER_H
#define MATLAB_RECEIVER_H

#include "control/threaded_loop.h"
#include "utils/sam.h"
#include "utils/socket.h"

class MatlabReceiver : public ThreadedLoop {
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

    explicit MatlabReceiver(std::shared_ptr<SAM::Components> robot);
    ~MatlabReceiver() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    void handle_command(Command c);

    Socket _socket;
    std::shared_ptr<SAM::Components> _robot;
};

#endif // MATLABRECEIVER_H
