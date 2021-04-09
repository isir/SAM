#include "controle_bretelles.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include <iostream>

ControleBretelles::ControleBretelles(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("matlab_receiver", 0.01)
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand_quantum)) {
        throw std::runtime_error("Demo is missing components");
    }

    _menu->set_description("Controle bretelles");
    _menu->set_code("cb");

    if (_robot->joints.shoulder_medial_rotation)
        _menu->add_item(_robot->joints.shoulder_medial_rotation->menu());
    if (_robot->joints.wrist_flexion) {
        _menu->add_item(_robot->joints.wrist_flexion->menu());
    }
    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());

    int port = 45457;
    if (!_socket.bind("0.0.0.0", port)) {
        critical() << "Failed to bind on port" << port;
    }
}

ControleBretelles::~ControleBretelles()
{
    stop_and_join();
}

bool ControleBretelles::setup()
{
    _robot->joints.elbow_flexion->calibrate();
    if (_robot->joints.wrist_flexion) {
        _robot->joints.wrist_flexion->calibrate();
    }
    _robot->joints.wrist_pronation->calibrate();

    return true;
}

void ControleBretelles::loop(double, clock::time_point)
{
    while (_socket.available()) {
        std::vector<std::byte> buf = _socket.receive();
        unpack_data(buf);
        std::cout << _avg << "\t" << _avd << "\t" << _arg << "\t" << _ard << "\t" << _avc << "\t" << _arc << "\t" <<_label << "\t" << _gain << std::endl;
    }

    static std::unique_ptr<MyoControl::Classifier> myocontrol;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_cocontraction = 5;
    static const unsigned int counts_before_bubble = 5;
    static const unsigned int counts_after_bubble = 5;

    static const MyoControl::EMGThresholds thresholds(15, 8, 15, 15, 8, 15);

    auto robot = _robot;
    MyoControl::Action elbow(
        "Elbow", [robot]() { robot->joints.elbow_flexion->set_velocity_safe(-35); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(35); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(0); });
    MyoControl::Action wrist_pronosup(
        "Wrist rotation", [robot]() { robot->joints.wrist_pronation->set_velocity_safe(40); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(-40); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });
    MyoControl::Action wrist_flex(
        "Wrist flexion", [robot]() { robot->joints.wrist_flexion->set_velocity_safe(20); }, [robot]() { robot->joints.wrist_flexion->set_velocity_safe(-20); }, [robot]() { robot->joints.wrist_flexion->set_velocity_safe(0); });
    MyoControl::Action shoulder(
        "Shoulder", [robot]() { robot->joints.shoulder_medial_rotation->set_velocity_safe(35); }, [robot]() { robot->joints.shoulder_medial_rotation->set_velocity_safe(-35); }, [robot]() { robot->joints.shoulder_medial_rotation->set_velocity_safe(0); });
    MyoControl::Action hand(
        "Hand", [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,2); }, [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); }, [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::STOP); });

    std::vector<MyoControl::Action> s1 {wrist_pronosup, elbow};
    if (_robot->joints.wrist_flexion)
        s1.insert(s1.begin() + 1, wrist_flex);
    if (_robot->joints.shoulder_medial_rotation)
        s1.push_back(shoulder);

    static LedStrip::color current_color = LedStrip::none;

    int emg[2];

    static bool first = true;
    if (first) {
        current_color = LedStrip::green;
        myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
        first = false;
    }

    emg[0] = 0;
    emg[1] = 0;

    if (_label==3) {
        emg[0] = 80;
    }
    if (_label==5) {
        emg[1] = 80;
    }
    if (_label==1) {
        emg[0] = 80;
        emg[1] = 80;
    }

    myocontrol->process(emg[0], emg[1]);

    if (myocontrol->has_changed_mode()) {
        _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
    }

    std::vector<LedStrip::color> colors(10, current_color);
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
    _robot->user_feedback.leds->set(colors);
}

void ControleBretelles::unpack_data(std::vector<std::byte> buffer) {
    char virgule = ',';
    unsigned int i = 0, j = 0, k=0;
    int a[4]={0}, avrgd[6]={0};
    while (j<6 && i<buffer.size()) {
        if (static_cast<char>(buffer[i])==virgule) {
            for (unsigned int l=0;l<k;l++) {
                avrgd[j]+=a[l]*static_cast<int>(std::pow(10,k-l-1));
            }
            j++;
            k = 0;
        } else {
            a[k]=static_cast<int>(buffer[i])-48;
            k++;
        }
        i++;
    }
    _avg = avrgd[0];
    _avd = avrgd[1];
    _arg = avrgd[2];
    _ard = avrgd[3];
    _avc = avrgd[4];
    _arc = avrgd[5];
    _label = static_cast<int>(buffer[i])-48;
    _gain = static_cast<int>(buffer[i+2])-48 + (static_cast<int>(buffer[i+4])-48)*0.1 + (static_cast<int>(buffer[i+5])-48)*0.01;
}

void ControleBretelles::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    if (_robot->joints.wrist_flexion) {
        _robot->joints.wrist_flexion->move_to(0, 10);
    }
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.hand_quantum->makeContraction(QuantumHand::STOP);

    _robot->user_feedback.leds->set(LedStrip::white, 11);
}
