#include "controle_bretelles.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include "wiringPi.h"
#include <iostream>
#include <filesystem>

ControleBretelles::ControleBretelles(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("controle_bretelles", 0.01)
    , _robot(robot)
    , _start_button(24)
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

    _menu->add_item("calib", "Calibrate braces values", [this](std::string) {calib_bretelles();});

    std::shared_ptr<MenuBackend> mode_submenu = std::make_shared<MenuBackend>("mode", "Choose control mode");
    mode_submenu->add_item("0", "Visualization mode", [this](std::string) { _mode = 0;});
    mode_submenu->add_item("1", "Demo discret epaules", [this](std::string) { _mode = 1;});
    mode_submenu->add_item("2", "Controle vitesse P 1DOF", [this](std::string) { _mode = 2;});
    mode_submenu->add_item("3", "Controle 2DOF", [this](std::string) { _mode = 3;});
    _menu->add_item(mode_submenu);

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
    if (_robot->joints.elbow_flexion->is_calibrated() == false && _mode!=0)
        _robot->joints.elbow_flexion->calibrate();
    if (_robot->joints.wrist_flexion) {
        _robot->joints.wrist_flexion->calibrate();
    }
    _robot->joints.wrist_pronation->calibrate();
    _robot->joints.wrist_pronation->set_encoder_position(0);


    _start = false;
    _cnt = 0;

    // Record data in txt file
    std::string filename("bretelles");
    std::string suffix;
    int cnt = 0;
    std::string extension(".txt");
    do {
        ++cnt;
        suffix = "_" + std::to_string(cnt);
    } while (std::filesystem::exists(filename + suffix + extension));
    _file = std::ofstream(filename + suffix + extension);
    if (!_file.good()) {
        critical() << "Failed to open" << (filename + suffix + extension);
        return false;
    }

    return true;
}

void ControleBretelles::loop(double, clock::time_point)
{
    static double pronoSupEncoder = 0;
    int keyStatus = _robot->sensors.touchsensor->readKeyStatus();
    static int prev_pin_status_value = 1;
    while (_socket.available()) {
        std::vector<std::byte> buf = _socket.receive();
        unpack_data(buf);
        std::cout << _avg << "\t" << _avd << "\t" << _arg << "\t" << _ard << "\t" << _cg << "\t" << _cd << "\t" <<_label << "\t" << _button << "\t" << prev_pin_status_value << "\t" << _gain << std::endl;
    }

    try {
        pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error when reading wrist encoder: " << e.what() << std::endl;
    }

    if (_mode != 0) {
        if (_start) {
            // if ((keyStatus==4 || _button == 0) && (prev_pin_status_value == 1)) { // button pressed to stop
            if (_button == 0 && (prev_pin_status_value == 1)) { // button pressed to stop
                _start = false;
                _robot->joints.wrist_pronation->forward(0);
            } else {
                switch ( _mode)
                {
                case 1:
                    demo_mode();
                    break;
                case 2:
                    controle_vitesse_p();
                    break;
                case 3:
                    controle_2DOF();
                    break;
                default:
                    std::cout << "Problem in mode selection" << std::endl;
                }
            }
        //} else if ((keyStatus==4 || _button == 0) && (prev_pin_status_value == 1)) { // button pressed to start
        } else if (_button == 0 && (prev_pin_status_value == 1)) { // button pressed to start
            _start = true;
            _cnt = 0;
        }
        //if (keyStatus==4 || _button == 0) {
        if (_button == 0) {
            prev_pin_status_value = 0;
        } else {
            prev_pin_status_value = 1;
        }
    }

    _file << _mode << "\t" << _start << "\t" << _avg << "\t" << _avd << "\t" << _arg << "\t" << _ard << "\t" << _cg << "\t" << _cd << "\t" <<_label << "\t" << _button << "\t" << _gain;
    _file << "\t" << _val_init[0] << "\t" << _val_init[1] << "\t" << _val_init[2]<< "\t" << _val_init[3] << "\t" << _val_init[4] << "\t" << _val_init[5] <<"\t" << pronoSupEncoder;
    _file << std::endl;
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
    _cg = avrgd[4];
    _cd = avrgd[5];
    _label = static_cast<int>(buffer[i])-48;
    _button = static_cast<int>(buffer[i+2])-48;
    _gain = static_cast<int>(buffer[i+4])-48 + (static_cast<int>(buffer[i+6])-48)*0.1 + (static_cast<int>(buffer[i+7])-48)*0.01;
}

void ControleBretelles::demo_mode() {
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
    if (_label==2) {
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

void ControleBretelles::controle_vitesse_p() {
    if (_cnt == 0) {
        calib_bretelles();
    } else {
        double speed = 0;
        double height = (_avg+_arg)/2;
        double height_init = (_val_init[0]+_val_init[2])/2;
        double error = height-height_init;
        if (error > 40)
            speed = round(error/5);
        else if (error < -40)
            speed = round(error/4);
        else
            speed = 0;

        _robot->joints.wrist_pronation->set_velocity_safe(speed);
    }
    _cnt++;
}

void ControleBretelles::controle_2DOF() {
    if (_cnt == 0) {
        calib_bretelles();
    } else {
        double speed_wrist = 0;
        double speed_elbow = 0;
        double cg_diff = _cg-_val_init[4];
        double avg_diff = _avg-_val_init[0];
        double arg_diff = _arg-_val_init[2];
        static unsigned int mode_changed_counter = 0;
        static unsigned int mode_change = 0;
        if (mode_changed_counter > 10) {
            mode_change = 0;
            mode_changed_counter = 0;
        }
        if (cg_diff<-50 && avg_diff>80 && mode_change != 2) { //retraction
            speed_wrist = 0;
            speed_elbow = round(cg_diff/8);
            mode_change = 1;
            mode_changed_counter = 0;
            std::cout << "retraction \t" << speed_elbow << std::endl;
        } else if (avg_diff<-80 && arg_diff<-80 && mode_change != 1) { //épaule basse
            speed_wrist = round(avg_diff/4);
            speed_elbow = 0;
            mode_change = 2;
            mode_changed_counter = 0;
            std::cout << "epaule basse \t" << speed_wrist << std::endl;
        } else if (cg_diff>50 && avg_diff<(cg_diff-50) && mode_change != 2) { //protraction
            speed_wrist = 0;
            speed_elbow = round(cg_diff/10);
            mode_change = 1;
            mode_changed_counter = 0;
            std::cout << "protraction \t" << speed_elbow << std::endl;
        } else if (cg_diff>20 && avg_diff>80 && arg_diff>50 && mode_change != 1) { //haussement épaule
            speed_wrist = round(avg_diff/5);
            speed_elbow = 0;
            mode_change = 2;
            mode_changed_counter = 0;
            std::cout << "epaule haute \t" << speed_wrist << std::endl;
        } else {
            mode_changed_counter++;
        }
        _robot->joints.wrist_pronation->set_velocity_safe(speed_wrist);
        _robot->joints.elbow_flexion->set_velocity_safe(speed_elbow);
    }
    _cnt++;
}

void ControleBretelles::controle_vitesse_pi() {
    static double sum_error = 0;
    if (_cnt == 0) {
        calib_bretelles();
        sum_error = 0;
    } else {
        double speed = 0;
        double height = (_avd+_ard)/2;
        double height_init = (_val_init[1]+_val_init[3])/2;
        double error = height_init-height;
        sum_error += error*0.01;
        if (error < -10)
            speed = round(-error/6-0*sum_error);
        else if (error > 10)
            speed = round(-error/4-0*sum_error);
        else
            speed = 0;

        _robot->joints.wrist_pronation->set_velocity_safe(speed);
        std::cout << height << "\t" << height_init << "\t" << error << "\t" << sum_error << "\t" << speed << std::endl;
    }
    _cnt++;
}

void ControleBretelles::calib_bretelles() {
    std::cout << "Please do not move" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int cnt = 0;
    for(int i=0;i<6;i++) {
        _val_init[i] = 0;
    }
    std::cout << "Starting to record data for calibration..." << std::endl;

    clock::time_point start_time = clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(clock::now() - start_time).count() < 2) {
        while (_socket.available()) {
            std::vector<std::byte> buf = _socket.receive();
            unpack_data(buf);
            _val_init[0] += _avg;
            _val_init[1] += _avd;
            _val_init[2] += _arg;
            _val_init[3] += _ard;
            _val_init[4] += _cg;
            _val_init[5] += _cd;
            cnt++;
        }
    }
    for(int i=0;i<6;i++) {
        _val_init[i] = round(_val_init[i]/cnt);
    }
    std::cout << _val_init[0] << "\t" << _val_init[1] << "\t" << _val_init[2] << "\t" << _val_init[3] << "\t" << _val_init[4] << "\t" << _val_init[5] << std::endl;
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
