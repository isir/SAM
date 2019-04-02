#include "demo.h"
#include "algorithms/myocontrol.h"
#include <QDebug>

#define FULL_MYO 0
#define IMU_ELBOW 1
#define FULL_MYO_FINGERS 2

Demo::Demo()
    : BasicController(.01)
    , _buzzer(29)
    , _pronosup(PronoSupination::instance())
    , _osmer(OsmerElbow::instance())
    , _hand("/dev/tb_hand")
{
}

bool Demo::setup()
{
    if (digitalRead(4)) {
        _buzzer.makeNoise(BuzzerConfig::SHORT_BUZZ);
        return false;
    }
    _buzzer.makeNoise(BuzzerConfig::DOUBLE_BUZZ);
    _myoband.start();

    _hand.setPosture(TouchBionicsHand::HAND_POSTURE);
    QThread::sleep(1);

    _hand.setSpeed(5);
    _hand.move(TouchBionicsHand::HAND_CLOSING);
    QThread::msleep(500);

    _hand.move(TouchBionicsHand::HAND_OPENING);
    QThread::sleep(1);
    _hand.move(TouchBionicsHand::THUMB_INT_CLOSING);

    _osmer.calibration();

    _hand.move(TouchBionicsHand::HAND_CLOSING_ALL);
    QThread::msleep(500);
    _hand.move(TouchBionicsHand::HAND_OPENING_ALL);
    QThread::msleep(500);

    if (!_myoband.connected()) {
        return false;
    }
    return true;
}

void Demo::loop(double, double)
{
    static MyoControl myocontrol;

    static int control_mode = 0, counter_auto_control = 0, move_elbow_counter = 0, elbow_mode = 0, old_elbow_mode = 0;
    static const int max_acc_change_mode = 100000000;

    static const int elbow_speed_up = -35;
    static const int elbow_speed_down = 35;
    static const int elbow_speed_time = 60;
    static double elbow_speed_gain = 0;
    static double elbow_speed_decel = 0;
    static const int wrist_speed = 50;

    static const int threshold_emg1_demo_nat = 550;
    static const int threshold_emg2_demo_nat = 550;
    static const int threshold_emg1_coco_demo_nat = 550;
    static const int threshold_emg2_coco_demo_nat = 550;

    static const MyoControl::MODE demoCocoSequence1[] = { MyoControl::MYO_MODE_WRIST, MyoControl::MYO_MODE_HAND, MyoControl::MYO_MODE_ELBOW };
    static const MyoControl::MODE demoCocoSequence2[] = { MyoControl::MYO_MODE_WRIST, MyoControl::MYO_MODE_HAND };
    static const MyoControl::MODE demoCocoSequence3[] = { MyoControl::MYO_MODE_WRIST, MyoControl::MYO_MODE_HAND, MyoControl::MYO_MODE_THUMB, MyoControl::MYO_MODE_FOREFINGER, MyoControl::MYO_MODE_MIDDLEFINGER, MyoControl::MYO_MODE_RINGFINGER, MyoControl::MYO_MODE_LITTLEFINGER, MyoControl::MYO_MODE_ELBOW };

    double emg[2], imu[10];

    for (int i = 0; i < 10; i++) {
        imu[i] = _myoband.getIMUs()[i];
    }

    if ((imu[0] * imu[0] + imu[1] * imu[1] + imu[2] * imu[2]) > max_acc_change_mode && counter_auto_control == 0) {
        emg[0] = 0;
        emg[1] = 0;
        control_mode = (control_mode + 1) % 3;
        _buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ, 10000);
        _osmer.set_velocity(0);
        counter_auto_control = 100;

        if (control_mode == FULL_MYO)
            myocontrol.initBubbleCocontractionControl(demoCocoSequence1, 3, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 150, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 150, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
        else if (control_mode == IMU_ELBOW)
            myocontrol.initBubbleCocontractionControl(demoCocoSequence2, 2, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 150, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 150, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
        else if (control_mode == FULL_MYO_FINGERS)
            myocontrol.initBubbleCocontractionControl(demoCocoSequence3, 8, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 150, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 150, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
    } else {
        emg[0] = _myoband.getEMGs()[3];
        emg[1] = _myoband.getEMGs()[7];
    }

    if (myocontrol.hasChangedMode()) {
        _buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ, 10000);
        if (myocontrol.getOldMode() == MyoControl::MYO_MODE_HAND || myocontrol.getOldMode() == MyoControl::MYO_MODE_HAND_CLOSING || myocontrol.getOldMode() == MyoControl::MYO_MODE_HAND_OPENING) {
            _hand.move(0);
        }
        if (myocontrol.getOldMode() == MyoControl::MYO_MODE_WRIST || myocontrol.getOldMode() == MyoControl::MYO_MODE_WRIST_FORWARDING || myocontrol.getOldMode() == MyoControl::MYO_MODE_WRIST_BACKWARDING) {
            _pronosup.forward(0);
        }
        if (myocontrol.getOldMode() == MyoControl::MYO_MODE_ELBOW) {
            _osmer.set_velocity(0);
        }
    }

    switch (myocontrol.getJointAction(emg[0], emg[1])) {
    case MyoControl::ELBOW_DOWN:
        _osmer.set_velocity(elbow_speed_down);
        elbow_mode = 1;
        break;
    case MyoControl::ELBOW_UP:
        _osmer.set_velocity(elbow_speed_up);
        elbow_mode = -1;
        break;
    case MyoControl::ELBOW_STOP:
        if (elbow_mode == 1) {
            elbow_speed_gain = -1;
            elbow_speed_decel = elbow_speed_gain / elbow_speed_time;
        } else if (elbow_mode == -1) {
            elbow_speed_gain = 1;
            elbow_speed_decel = elbow_speed_gain / elbow_speed_time;
        }
        elbow_mode = 0;
        _osmer.set_velocity(elbow_speed_gain * elbow_speed_up);
        if (elbow_speed_gain != 0. && qAbs(elbow_speed_gain) >= qAbs(elbow_speed_decel)) {
            elbow_speed_gain -= elbow_speed_decel;
        } else {
            elbow_speed_gain = 0.;
        }
        break;
    case MyoControl::WRIST_BACKWARD:
        _pronosup.backward(wrist_speed);
        break;
    case MyoControl::WRIST_FORWARD:
        _pronosup.forward(wrist_speed);
        break;
    case MyoControl::WRIST_STOP:
        _pronosup.forward(0);
        break;
    case MyoControl::HAND_STOP:
        _hand.move(0);
        break;
    case MyoControl::THUMB_STOP:
        _hand.move(0);
        break;
    case MyoControl::FOREFINGER_STOP:
        _hand.move(0);
        break;
    case MyoControl::RINGFINGER_STOP:
        _hand.move(0);
        break;
    case MyoControl::MIDDLEFINGER_STOP:
        _hand.move(0);
        break;
    case MyoControl::LITTLEFINGER_STOP:
        _hand.move(0);
        break;
    case MyoControl::HAND_OPEN:
        _hand.move(TouchBionicsHand::HAND_OPENING_ALL);
        break;
    case MyoControl::HAND_CLOSE:
        _hand.move(TouchBionicsHand::HAND_CLOSING_ALL);
        break;
    case MyoControl::THUMB_OPEN:
        _hand.move(TouchBionicsHand::THUMB_OPENING);
        break;
    case MyoControl::THUMB_CLOSE:
        _hand.move(TouchBionicsHand::THUMB_CLOSING);
        break;
    case MyoControl::FOREFINGER_OPEN:
        _hand.move(TouchBionicsHand::FOREFINGER_OPENING);
        break;
    case MyoControl::FOREFINGER_CLOSE:
        _hand.move(TouchBionicsHand::FOREFINGER_CLOSING);
        break;
    case MyoControl::MIDDLEFINGER_OPEN:
        _hand.move(TouchBionicsHand::MIDDLEFINGER_OPENING);
        break;
    case MyoControl::MIDDLEFINGER_CLOSE:
        _hand.move(TouchBionicsHand::MIDDLEFINGER_CLOSING);
        break;
    case MyoControl::RINGFINGER_OPEN:
        _hand.move(TouchBionicsHand::RINGFINGER_OPENING);
        break;
    case MyoControl::RINGFINGER_CLOSE:
        _hand.move(TouchBionicsHand::RINGFINGER_CLOSING);
        break;
    case MyoControl::LITTLEFINGER_OPEN:
        _hand.move(TouchBionicsHand::LITTLEFINGER_OPENING);
        break;
    case MyoControl::LITTLEFINGER_CLOSE:
        _hand.move(TouchBionicsHand::LITTLEFINGER_CLOSING);
        break;
    default:
        qWarning() << "MYOCONTROL ERROR : Unknown joint to activate...";
    }

    // Elbow control if not full myo
    if (counter_auto_control > 0) {
        counter_auto_control--;
    } else {
        if (control_mode == IMU_ELBOW) {
            old_elbow_mode = elbow_mode;
            if (imu[1] > 1000 || imu[1] < -1200) {
                if (move_elbow_counter > 10) { // remove acc jump (due to cocontraction for example...)
                    if (imu[1] > 1000) {
                        // elbow down : -1
                        elbow_mode = -1;
                        _osmer.set_velocity(elbow_speed_down);
                    } else if (imu[1] < -1200) {
                        // elbow up : 1
                        elbow_mode = 1;
                        _osmer.set_velocity(elbow_speed_up);
                    }
                } else {
                    move_elbow_counter++;
                }
            } else {
                move_elbow_counter = 0;
                // elbow stop : 0
                if (old_elbow_mode == -1) {
                    elbow_speed_gain = -1;
                    elbow_speed_decel = elbow_speed_gain / elbow_speed_time;
                } else if (old_elbow_mode == 1) {
                    elbow_speed_gain = 1;
                    elbow_speed_decel = elbow_speed_gain / elbow_speed_time;
                }
                elbow_mode = 0;

                _osmer.set_velocity(elbow_speed_gain * elbow_speed_up);
                if (elbow_speed_gain != 0. && qAbs(elbow_speed_gain) >= abs(elbow_speed_decel)) {
                    elbow_speed_gain -= elbow_speed_decel;
                } else {
                    elbow_speed_gain = 0.;
                }
            }
        }
    }
}

void Demo::cleanup()
{
}
