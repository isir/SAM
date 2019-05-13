#include "demo.h"
#include "algorithms/myocontrol.h"
#include "peripherals/ledstrip.h"
#include <QDebug>
#include <QTime>

#define FULL_MYO 0
#define IMU_ELBOW 1
#define FULL_MYO_FINGERS 2

Demo::Demo(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt)
    : BasicController(mqtt, .01)
    , _robot(robot)
{
    _menu.set_title("Demo");
    _menu.set_code("demo");
    _menu.addItem(_robot.wrist_pronosup->menu());
    _menu.addItem(_robot.elbow->menu());
    _menu.addItem(_robot.hand->menu());

    _settings.beginGroup("Demo");
    _pin_up = _settings.value("pin_up", 24).toInt();
    _pin_down = _settings.value("pin_down", 22).toInt();
    _settings.endGroup();

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

Demo::~Demo()
{
    stop();
}

bool Demo::setup()
{
    pinMode(28, INPUT);
    pullUpDnControl(28, PUD_UP);
    if (digitalRead(28)) {
        _robot.buzzer->makeNoise(BuzzerConfig::SHORT_BUZZ);
        return false;
    }
    _robot.buzzer->makeNoise(BuzzerConfig::DOUBLE_BUZZ);

    if (!_robot.hand->take_ownership()) {
        throw std::runtime_error("Demo failed to take ownership of the hand");
    }
    _robot.hand->init_sequence();
    _robot.elbow->calibrate();

    _robot.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
    QThread::msleep(500);
    _robot.hand->move(TouchBionicsHand::HAND_OPENING_ALL);
    QThread::msleep(500);

    return true;
}

void Demo::loop(double, double)
{
    static MyoControl myocontrol;

    static int control_mode = 0;
    static int counter_auto_control = 0, move_elbow_counter = 0;
    static const int max_acc_change_mode = 15;

    static const int elbow_speed_up = -35;
    static const int elbow_speed_down = 35;
    static const double wrist_speed = 20;

    static const int threshold_emg1_demo_nat = 15;
    static const int threshold_emg2_demo_nat = 15;
    static const int threshold_emg1_coco_demo_nat = 15;
    static const int threshold_emg2_coco_demo_nat = 15;

    static const MyoControl::MODE demoCocoSequence1[] = { MyoControl::MYO_MODE_WRIST, MyoControl::MYO_MODE_HAND, MyoControl::MYO_MODE_ELBOW };
    static const MyoControl::MODE demoCocoSequence2[] = { MyoControl::MYO_MODE_WRIST, MyoControl::MYO_MODE_HAND };
    static const MyoControl::MODE demoCocoSequence3[] = { MyoControl::MYO_MODE_WRIST, MyoControl::MYO_MODE_HAND, MyoControl::MYO_MODE_THUMB, MyoControl::MYO_MODE_FOREFINGER, MyoControl::MYO_MODE_MIDDLEFINGER, MyoControl::MYO_MODE_RINGFINGER, MyoControl::MYO_MODE_LITTLEFINGER, MyoControl::MYO_MODE_ELBOW };

    static LedStrip::color current_color = LedStrip::none;

    volatile double emg[2];

    static bool first = true;
    if (first) {
        control_mode = FULL_MYO;
        current_color = LedStrip::green;
        myocontrol.setControlType(MyoControl::BUBBLE_COCO_CONTROL);
        myocontrol.initBubbleCocontractionControl(demoCocoSequence1, 3, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 7, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 7, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
        first = false;
    }

    Eigen::Vector3d acc;

    emg[0] = 0;
    emg[1] = 0;

    if (_robot.myoband) {
        acc = _robot.myoband->get_acc();

        if (_robot.myoband->connected()) {
            if ((acc.squaredNorm() > max_acc_change_mode) && counter_auto_control == 0) {
                control_mode = (control_mode + 1) % 3;
                _robot.buzzer->makeNoise(BuzzerConfig::TRIPLE_BUZZ);
                _robot.elbow->set_velocity_safe(0);
                counter_auto_control = 100;

                if (control_mode == FULL_MYO) {
                    qInfo() << "Full myo";
                    current_color = LedStrip::green;
                    myocontrol.initBubbleCocontractionControl(demoCocoSequence1, 3, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 7, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 7, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
                } else if (control_mode == IMU_ELBOW) {
                    qInfo() << "IMU Elbow";
                    current_color = LedStrip::red;
                    myocontrol.initBubbleCocontractionControl(demoCocoSequence2, 2, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 7, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 7, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
                } else if (control_mode == FULL_MYO_FINGERS) {
                    qInfo() << "Full myo + fingers";
                    current_color = LedStrip::blue;
                    myocontrol.initBubbleCocontractionControl(demoCocoSequence3, 8, 15, 5, 5, 5, threshold_emg1_demo_nat, threshold_emg1_demo_nat - 7, threshold_emg2_demo_nat, threshold_emg2_demo_nat - 7, threshold_emg1_coco_demo_nat, threshold_emg2_coco_demo_nat);
                }
            } else {
                QVector<qint32> rms = _robot.myoband->get_emgs_rms();
                if (rms.size() < 8)
                    return;
                emg[0] = rms[3];
                emg[1] = rms[7];
            }
        }
    }

    if (digitalRead(_pin_down) == 0) {
        emg[0] = 80;
    }
    if (digitalRead(_pin_up) == 0) {
        emg[1] = 80;
    }

    if (myocontrol.hasChangedMode()) {
        _robot.buzzer->makeNoise(BuzzerConfig::STANDARD_BUZZ);
        if (myocontrol.getOldMode() == MyoControl::MYO_MODE_HAND || myocontrol.getOldMode() == MyoControl::MYO_MODE_HAND_CLOSING || myocontrol.getOldMode() == MyoControl::MYO_MODE_HAND_OPENING) {
            _robot.hand->move(0);
        }
        if (myocontrol.getOldMode() == MyoControl::MYO_MODE_WRIST || myocontrol.getOldMode() == MyoControl::MYO_MODE_WRIST_FORWARDING || myocontrol.getOldMode() == MyoControl::MYO_MODE_WRIST_BACKWARDING) {
            _robot.wrist_pronosup->forward(0);
        }
        if (myocontrol.getOldMode() == MyoControl::MYO_MODE_ELBOW) {
            _robot.elbow->set_velocity_safe(0);
        }
    }

    MyoControl::JOINT_ACTION action = myocontrol.getJointAction(emg[0], emg[1]);

    QVector<LedStrip::color> colors;
    colors.fill(current_color, 10);
    switch (myocontrol.get_current_index()) {
    case 0:
        colors[4] = LedStrip::color(80, 30, 0, 1);
        break;
    case 1:
        colors[4] = LedStrip::color(0, 50, 50, 1);
        break;
    case 2:
        colors[4] = LedStrip::color(50, 0, 50, 1);
        break;
    default:
        break;
    }

    switch (action) {
    case MyoControl::ELBOW_DOWN:
        _robot.elbow->set_velocity_safe(elbow_speed_down);
        colors[2] = LedStrip::white;
        colors[3] = LedStrip::white;
        break;
    case MyoControl::ELBOW_UP:
        _robot.elbow->set_velocity_safe(elbow_speed_up);
        colors[5] = LedStrip::white;
        colors[6] = LedStrip::white;
        break;
    case MyoControl::ELBOW_STOP:
        _robot.elbow->set_velocity_safe(0);
        break;
    case MyoControl::WRIST_BACKWARD:
        _robot.wrist_pronosup->set_velocity(-wrist_speed);
        colors[2] = LedStrip::white;
        colors[3] = LedStrip::white;
        break;
    case MyoControl::WRIST_FORWARD:
        _robot.wrist_pronosup->set_velocity(wrist_speed);
        colors[5] = LedStrip::white;
        colors[6] = LedStrip::white;
        break;
    case MyoControl::WRIST_STOP:
        _robot.wrist_pronosup->forward(0);
        break;
    case MyoControl::HAND_STOP:
        _robot.hand->move(0);
        break;
    case MyoControl::THUMB_STOP:
        _robot.hand->move(0);
        break;
    case MyoControl::FOREFINGER_STOP:
        _robot.hand->move(0);
        break;
    case MyoControl::RINGFINGER_STOP:
        _robot.hand->move(0);
        break;
    case MyoControl::MIDDLEFINGER_STOP:
        _robot.hand->move(0);
        break;
    case MyoControl::LITTLEFINGER_STOP:
        _robot.hand->move(0);
        break;
    case MyoControl::HAND_OPEN:
        _robot.hand->move(TouchBionicsHand::HAND_OPENING_ALL);
        colors[2] = LedStrip::white;
        colors[3] = LedStrip::white;
        break;
    case MyoControl::HAND_CLOSE:
        _robot.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
        colors[5] = LedStrip::white;
        colors[6] = LedStrip::white;
        break;
    case MyoControl::THUMB_OPEN:
        _robot.hand->move(TouchBionicsHand::THUMB_OPENING);
        break;
    case MyoControl::THUMB_CLOSE:
        _robot.hand->move(TouchBionicsHand::THUMB_CLOSING);
        break;
    case MyoControl::FOREFINGER_OPEN:
        _robot.hand->move(TouchBionicsHand::FOREFINGER_OPENING);
        break;
    case MyoControl::FOREFINGER_CLOSE:
        _robot.hand->move(TouchBionicsHand::FOREFINGER_CLOSING);
        break;
    case MyoControl::MIDDLEFINGER_OPEN:
        _robot.hand->move(TouchBionicsHand::MIDDLEFINGER_OPENING);
        break;
    case MyoControl::MIDDLEFINGER_CLOSE:
        _robot.hand->move(TouchBionicsHand::MIDDLEFINGER_CLOSING);
        break;
    case MyoControl::RINGFINGER_OPEN:
        _robot.hand->move(TouchBionicsHand::RINGFINGER_OPENING);
        break;
    case MyoControl::RINGFINGER_CLOSE:
        _robot.hand->move(TouchBionicsHand::RINGFINGER_CLOSING);
        break;
    case MyoControl::LITTLEFINGER_OPEN:
        _robot.hand->move(TouchBionicsHand::LITTLEFINGER_OPENING);
        break;
    case MyoControl::LITTLEFINGER_CLOSE:
        _robot.hand->move(TouchBionicsHand::LITTLEFINGER_CLOSING);
        break;
    default:
        qWarning() << "MYOCONTROL ERROR : Unknown joint to activate...";
        break;
    }

    _robot.leds->set(colors);

    // Elbow control if not full myo
    if (counter_auto_control > 0) {
        counter_auto_control--;
    } else {
        if (control_mode == IMU_ELBOW) {
            if (acc[1] > 0.2 || acc[1] < -0.2) {
                if (move_elbow_counter > 10) { // remove acc jump (due to cocontraction for example...)
                    if (acc[1] > 0.2) {
                        _robot.elbow->set_velocity_safe(elbow_speed_down);
                    } else if (acc[1] < -0.2) {
                        _robot.elbow->set_velocity_safe(elbow_speed_up);
                    }
                } else {
                    move_elbow_counter++;
                }
            } else {
                _robot.elbow->set_velocity_safe(0);
            }
        }
    }

    if (digitalRead(28)) {
        stop();
    }
}

void Demo::cleanup()
{
    if (_robot.myoband) {
        _robot.myoband->stop();
    }
    _robot.wrist_pronosup->forward(0);
    _robot.elbow->move_to(0, 20);
    _robot.leds->set(LedStrip::white, 10);
    _robot.hand->release_ownership();
}
