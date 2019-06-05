#include "demo.h"
#include "algorithms/myocontrol.h"
#include "peripherals/ledstrip.h"
#include <QDebug>
#include <QTime>

#define FULL_MYO 0
#define IMU_ELBOW 1
#define FULL_MYO_FINGERS 2

Demo::Demo(SAM::Components robot)
    : BasicController(.01)
    , _robot(robot)
{
    _menu.set_title("Demo");
    _menu.set_code("demo");
    if (_robot.shoulder)
        _menu.addItem(_robot.shoulder->menu());
    if (_robot.elbow)
        _menu.addItem(_robot.elbow->menu());
    if (_robot.wrist_pronosup)
        _menu.addItem(_robot.wrist_pronosup->menu());
    if (_robot.hand)
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
    _robot.hand->take_ownership();
    _robot.hand->init_sequence();
    _robot.elbow->calibrate();
    if (_robot.wrist_flex) {
        _robot.wrist_flex->calibrate();
    }
    _robot.wrist_pronosup->calibrate();

    return true;
}

void Demo::loop(double, double)
{
    static std::unique_ptr<MyoControl::Classifier> myocontrol;

    static int control_mode = 0;
    static int counter_auto_control = 0, move_elbow_counter = 0;
    static const int max_acc_change_mode = 15;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_cocontraction = 5;
    static const unsigned int counts_before_bubble = 5;
    static const unsigned int counts_after_bubble = 5;

    static const MyoControl::EMGThresholds thresholds(15, 8, 15, 15, 8, 15);

    MyoControl::Action elbow(
        "Elbow", [this]() { _robot.elbow->set_velocity_safe(-35); }, [this]() { _robot.elbow->set_velocity_safe(35); }, [this]() { _robot.elbow->set_velocity_safe(0); });
    MyoControl::Action wrist_pronosup(
        "Wrist rotation", [this]() { _robot.wrist_pronosup->set_velocity_safe(40); }, [this]() { _robot.wrist_pronosup->set_velocity_safe(-40); }, [this]() { _robot.wrist_pronosup->set_velocity_safe(0); });
    MyoControl::Action wrist_flex(
        "Wrist flexion", [this]() { _robot.wrist_flex->set_velocity_safe(20); }, [this]() { _robot.wrist_flex->set_velocity_safe(-20); }, [this]() { _robot.wrist_flex->set_velocity_safe(0); });
    MyoControl::Action shoulder(
        "Shoulder", [this]() { _robot.shoulder->set_velocity_safe(35); }, [this]() { _robot.shoulder->set_velocity_safe(-35); }, [this]() { _robot.shoulder->set_velocity_safe(0); });
    MyoControl::Action hand(
        "Hand", [this]() { _robot.hand->move(TouchBionicsHand::HAND_OPENING_ALL); }, [this]() { _robot.hand->move(TouchBionicsHand::HAND_CLOSING_ALL); }, [this]() { _robot.hand->move(TouchBionicsHand::STOP); });

    std::vector<MyoControl::Action> s1 { hand, wrist_pronosup, elbow };
    if (_robot.wrist_flex)
        s1.insert(s1.begin() + 2, wrist_flex);
    if (_robot.shoulder)
        s1.push_back(shoulder);
    std::vector<MyoControl::Action> s2 { hand, wrist_pronosup };

    static LedStrip::color current_color = LedStrip::none;

    volatile double emg[2];

    static bool first = true;
    if (first) {
        control_mode = FULL_MYO;
        current_color = LedStrip::green;
        myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
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
                    myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
                } else if (control_mode == IMU_ELBOW) {
                    qInfo() << "IMU Elbow";
                    current_color = LedStrip::red;
                    myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s2, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
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

    myocontrol->process(emg[0], emg[1]);

    if (myocontrol->has_changed_mode()) {
        _robot.buzzer->makeNoise(BuzzerConfig::STANDARD_BUZZ);
    }

    QVector<LedStrip::color> colors;
    colors.fill(current_color, 10);
    switch (myocontrol->current_index()) {
    case 0:
        colors[4] = LedStrip::color(80, 30, 0, 1);
        break;
    case 1:
        colors[4] = LedStrip::color(0, 50, 50, 1);
        break;
    case 2:
        colors[4] = LedStrip::color(50, 0, 50, 1);
        break;
    case 3:
        colors[4] = LedStrip::white;
        break;
    default:
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
                        _robot.elbow->set_velocity_safe(35);
                    } else if (acc[1] < -0.2) {
                        _robot.elbow->set_velocity_safe(-35);
                    }
                } else {
                    move_elbow_counter++;
                }
            } else {
                _robot.elbow->set_velocity_safe(0);
            }
        }
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
