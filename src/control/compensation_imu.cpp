#include "compensation_imu.h"
#include "algo/myocontrol.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

// indicate if optitrack is on
#define OPTITRACK 0

CompensationIMU::CompensationIMU(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("CompensationIMU", 0.0143)
    , _robot(robot)
    , _lambdaW("lambda wrist", BaseParam::ReadWrite, this, 5.)
    , _thresholdW("threshold wrist", BaseParam::ReadWrite, this, 5.)
    , _lambdaE("lambda elbow", BaseParam::ReadWrite, this, 0.)
    , _thresholdE("threshold elbow", BaseParam::ReadWrite, this, 5.)
    , _lt("_lt(cm)", BaseParam::ReadWrite, this, 40)
    , _lua("_lua(cm)", BaseParam::ReadWrite, this, 30)
    , _lfa("_lfa(cm)", BaseParam::ReadWrite, this, 35)
    , _lsh("_lsh(cm)", BaseParam::ReadWrite, this, -20)
{
    if (!check_ptr(_robot->sensors.yellow_imu)) {
        throw std::runtime_error("Compensation IMU Control is missing components");
    }

    if (!_receiver.bind("0.0.0.0", 45453)) {
        critical() << "CompensationOptitrack: Failed to bind receiver";
    }

    _menu->set_description("CompensationIMU");
    _menu->set_code("imu");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("pin", "Display Pin data", [this](std::string) { this->displayPin(); });
    _menu->add_item("calib", "Calibration", [this](std::string) { this->calibration(); });

    _menu->add_item(_robot->joints.wrist_pronation->menu());
    if (_robot->joints.elbow_flexion)
        _menu->add_item(_robot->joints.elbow_flexion->menu());
    if (_robot->joints.hand)
        _menu->add_item(_robot->joints.hand->menu());

    // threshold EMG
    _th_low[0] = 3000;
    _th_low[1] = 1500;

    _th_high[0] = 3000;
    _th_high[1] = 3000;
}

CompensationIMU::~CompensationIMU()
{
    if (_robot->joints.elbow_flexion)
        _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void CompensationIMU::tare_IMU()
{
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.red_imu)
        _robot->sensors.red_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    std::this_thread::sleep_for(std::chrono::seconds(6));
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void CompensationIMU::displayPin()
{
    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

void CompensationIMU::calibration()
{
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        _robot->joints.elbow_flexion->calibrate();
    }
    if (_robot->joints.elbow_flexion->is_calibrated())
        debug() << "Calibration elbow: ok \n";

    _robot->joints.elbow_flexion->move_to(90, 20);
}

bool CompensationIMU::setup()
{
    // ADC setup
    _param_file = std::ifstream("myo_thresholds");
    if (!_param_file.good()) {
        critical() << "Failed to open myo_thresholds file. Using defau_lt values instead.";
    } else {
        std::string number_string;
        for (uint16_t i = 0; i < _n_electrodes; i++) {
            std::getline(_param_file, number_string);
            _th_low[i] = std::stoi(number_string);
        }
        for (uint16_t i = 0; i < _n_electrodes; i++) {
            std::getline(_param_file, number_string);
            _th_high[i] = std::stoi(number_string);
        }
    }
    _param_file.close();

    // Prosthesis setup
    _robot->joints.wrist_pronation->set_encoder_position(0);

    if (saveData) {
        std::string filename("compensationIMU");
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
        _need_to_write_header = true;
    }
    _cnt = 0.;
    _start_time = clock::now();
    _emg[0] = 0;
    _emg[1] = 0;
    // generate random number for buzzer
    int randN = rand() % 3;
    buzzN = floor(1 / period() + randN / period());
    debug() << "buzzN : " << buzzN;
    return true;
}

void CompensationIMU::loop(double dt, clock::time_point time)
{
    int init_cnt = 10;
    //    debug() << "cnt : " << _cnt;
    double timeWithDe_lta = (time - _start_time).count();

#if OPTITRACK
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    debug() << "Rigid Bodies: " << data.nRigidBodies;
#endif
    double debugData[10];

    if (saveData) {
        if (_need_to_write_header) {
            _file << " time, pinUp, pinDown,";
            _file << " qWhite.w, qWhite.x, qWhite.y, qWhite.z, qRed.w, qRed.x, qRed.y, qRed.z,";
            _file << " qYellow.w, qYellow.x, qYellow.y, qYellow.z,";
            _file << " phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW, wristEncoder,";
            _file << " nbRigidBodies";

#if OPTITRACK
            for (int i = 0; i < data.nRigidBodies; i++) {
                _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
            }
#endif
            _file << "\r\n";
            _need_to_write_header = false;
        }
    }

    double qWhite[4], qRed[4], qYellow[4];
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->get_quat(qWhite);
    if (_robot->sensors.red_imu)
        _robot->sensors.red_imu->get_quat(qRed);
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->get_quat(qYellow);

    //    printf("qYellow: %lf; %lf; %lf; %lf\n", qYellow[0], qYellow[1], qYellow[2], qYellow[3]);
    //    printf("qWhite: %lf; %lf; %lf; %lf\n", qWhite[0], qWhite[1], qWhite[2], qWhite[3]);

    qWhite_record.w() = qWhite[0];
    qWhite_record.x() = qWhite[1];
    qWhite_record.y() = qWhite[2];
    qWhite_record.z() = qWhite[3];

    qYellow_record.w() = qYellow[0];
    qYellow_record.x() = qYellow[1];
    qYellow_record.y() = qYellow[2];
    qYellow_record.z() = qYellow[3];

    /// WRIST
    double wristAngleEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    double elbowAngleEncoder = 0.;
    double beta = 0.;
    int boolBuzz = 0;
    //    if ((_cnt + 250) % buzzN == 0) {
    //        _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
    //        boolBuzz = 1;
    //    }

    if (_robot->joints.elbow_flexion) {
        elbowAngleEncoder = _robot->joints.elbow_flexion->read_encoder_position();
        if (protoCyb) {
            beta = -_robot->joints.elbow_flexion->pos() * M_PI / 180.;
        } else {
            beta = _robot->joints.elbow_flexion->pos() * M_PI / 180.;
        }
        law_elbowAndWrist(init_cnt, debugData, beta);
    } else {
        law_wristOnly(init_cnt, debugData);
    }

    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;

    if (saveData) {
        _file << timeWithDe_lta << ' ' << pin_down_value << ' ' << pin_up_value << ' ' << boolBuzz << ' ' << _emg[0] << ' ' << _emg[1];
        _file << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3] << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3];
        _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
        _file << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << debugData[3] << debugData[4] << ' ' << debugData[5] << ' ' << debugData[6] << ' ' << debugData[7] << ' ' << debugData[8];
        _file << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << wristAngleEncoder;
        if (_robot->joints.elbow_flexion) {
            _file << ' ' << _lambdaE << ' ' << _thresholdE << ' ' << elbowAngleEncoder;
        }

#if OPTITRACK
        _file << ' ' << data.nRigidBodies;
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
            _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
            _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
        }
#endif
        _file << std::endl;
    }

    ++_cnt;
    //    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void CompensationIMU::law_elbowAndWrist(int init_cnt, double debugData[], double beta)
{
    /// HAND
    _emg[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    _emg[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    static std::unique_ptr<MyoControl::Classifier> handcontrol;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_btn = 2;
    static const unsigned int counts_before_bubble = 2;
    static const unsigned int counts_after_bubble = 2;

    static const MyoControl::EMGThresholds thresholds(5000, 1500, 0, 5000, 1500, 0);

    auto robot = _robot;

    MyoControl::Action co_contraction("Co contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::CO_CONTRACTION); });
    MyoControl::Action double_contraction("Double contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION); });
    MyoControl::Action triple_contraction("Triple contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION); });

    std::vector<MyoControl::Action> s1{ co_contraction, double_contraction, triple_contraction };

    static bool first = true;
    if (first) {
        handcontrol = std::make_unique<MyoControl::QuantumClassifier>(s1, thresholds, counts_after_mode_change, counts_btn, counts_before_bubble, counts_after_bubble);
        first = false;
    }

    //EMG1 and EMG2 = hand
    static bool btn = 0;
    if (!_robot->btn1) {
        btn = 1;
    } else {
        btn = 0;
    }
    handcontrol->process(_emg[0], _emg[1], btn);

    if (_cnt == 0) {
        _lawimuWE.initialization(_lt, _lua, _lfa, _lsh);
        printf("Elbow and Wrist\n");
    } else if (_cnt <= init_cnt) {
        _lawimuWE.recordInitialPosition(qYellow_record, qWhite_record, _cnt, init_cnt);

        _lawimuWE.moveToIdealFrames(qYellow_record, _cnt, init_cnt);
    } else {

        _lawimuWE.computeAxis(qYellow_record, qWhite_record);
        //// compute Rotation matrices
        _lawimuWE.rotationMatrices(qWhite_record);
        /// Compute end-effector coordinates with compensation  motions
        _lawimuWE.new_coordEE(_lt, _lua, _lfa, _lsh, beta);
        /// Compute elbow velocity
        _lawimuWE.computeBetaDot(_lua, _lfa, _lambdaE, _thresholdE * M_PI / 180, beta);
        _lawimuWE.display(_cnt);
        // Compute wrist ang. velocity
        _lawimuWE.computeWristVel(_lambdaW, _thresholdW * M_PI / 180);

        /// Send elbow velocity
        if (protoCyb) {
            _robot->joints.elbow_flexion->set_velocity_safe(-_lawimuWE.getBetaDot_deg());
        } else {
            _robot->joints.elbow_flexion->set_velocity_safe(_lawimuWE.getBetaDot_deg());
        }

        /// Send wrist velocity
        _robot->joints.wrist_pronation->set_velocity_safe(_lawimuWE.getWristVel());
    }
    // Write debug data file
    _lawimuWE.writeDebugData(debugData, beta);
}

void CompensationIMU::law_wristOnly(int init_cnt, double debugData[])
{
    if (_cnt == 0) {
        _lawimu.initialization();
        printf("Wrist Only \n");
    } else if (_cnt <= init_cnt) {
        _lawimu.initialPositions(qYellow_record, _cnt, init_cnt);
    } else {
        _lawimu.rotationMatrices(qYellow_record);
        _lawimu.controlLawWrist(_lambdaW, _thresholdW * M_PI / 180);

        _robot->joints.wrist_pronation->set_velocity_safe(_lawimu.returnWristVel_deg());
        //        _robot->joints.wrist_pronation->forward(40);

        if (_cnt % 50 == 0) {
            _lawimu.displayData();
        }
    }

    _lawimu.writeDebugData(debugData);

    /// read EMG
    // Set configuration bits
    //    _emg[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    //    _emg[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    //    debug() << "emg : " << _emg[0] << "; " << _emg[1];
    //    uint16_t config_global = ADS1015_REG_CONFIG_CQUE_NONE | // Disable the comparator (defau_lt val)
    //        ADS1015_REG_CONFIG_CLAT_NONLAT | // Non-latching (defau_lt val)
    //        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (defau_lt val)
    //        ADS1015_REG_CONFIG_CMODE_TRAD | // Traditional comparator (defau_lt val)
    //        ADS1015_REG_CONFIG_DR_3300SPS | // 3300 samples per second (ie 860sps pour un ADS1115)
    //        ADS1015_REG_CONFIG_MODE_SINGLE | // Single-shot mode (defau_lt)
    //        _robot->sensors.adc0->getGain() | //Set PGA/vo_ltage range
    //        ADS1015_REG_CONFIG_OS_SINGLE; // Set 'start single-conversion' bit

    //    uint16_t config_c0 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_0; //for channel 0
    //    uint16_t config_c1 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_1; //for channel 1
    //    uint16_t config_c2 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_2; //for channel 2
    //    uint16_t config_c3 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_3; //for channel 3
    //    //Write config register to the ADC
    //    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c1);
    //    // Wait for the conversion to complete
    //    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
    //        ;
    //    // Read the conversion resu_lts
    //    _emg[0] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    //    //Write config register to the ADC
    //    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c3);
    //    // Wait for the conversion to complete
    //    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
    //        ;
    //    // Read the conversion resu_lts
    //    _emg[1] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    //    for (uint16_t i = 0; i < 2; i++) {
    //        if (_emg[i] > 65500) {
    //            _emg[i] = 0;
    //        }
    //    }
}

void CompensationIMU::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    if (saveData)
        _file.close();
}
