#include "compensation_imu.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

CompensationIMU::CompensationIMU(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Compensation IMU", 0.0143)
    , _robot(robot)
    , _lambdaW("lambda wrist", BaseParam::ReadWrite, this, 5)
    , _thresholdW("threshold wrist", BaseParam::ReadWrite, this, 5.)
{
    if (!check_ptr(_robot->joints.wrist_pronation, _robot->sensors.yellow_imu)) {
        throw std::runtime_error("Compensation IMU Control is missing components");
    }

    if (!_receiver.bind("0.0.0.0", 45453)) {
        critical() << "CompensationOptitrack: Failed to bind receiver";
    }

    _menu->set_description("CompensationIMU");
    _menu->set_code("imu");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("pin", "Display Pin data", [this](std::string) { this->displayPin(); });

    _menu->add_item(_robot->joints.wrist_pronation->menu());
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

bool CompensationIMU::setup()
{
    // ADC setup
    _param_file = std::ifstream("myo_thresholds");
    if (!_param_file.good()) {
        critical() << "Failed to open myo_thresholds file. Using default values instead.";
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
    debug() << "cnt : " << _cnt;
    double timeWithDelta = (time - _start_time).count();

    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    debug() << "Rigid Bodies: " << data.nRigidBodies;

    double debugData[10];

    if (_need_to_write_header) {
        _file << " time, pinUp, pinDown,";
        _file << " qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,";
        _file << " qFA.w, qFA.x, qFA.y, qFA.z,";
        _file << " phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW, wristEncoder,";
        _file << " nbRigidBodies";
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
        }
        _file << "\r\n";
        _need_to_write_header = false;
    }

    /// HAND
    _emg[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    _emg[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    if (_emg[0] > _th_high[0] && _emg[1] < _th_low[1]) {
        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 1);
    } else if (_emg[1] > _th_high[1] && _emg[0] < _th_low[0]) {
        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 1);
    } else {
    }

    /// WRIST
    double wristAngleEncoder = _robot->joints.wrist_pronation->read_encoder_position();

    double qBras[4], qTronc[4], qFA[4];
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->get_quat(qBras);
    if (_robot->sensors.red_imu)
        _robot->sensors.red_imu->get_quat(qTronc);
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->get_quat(qFA);

    //    qDebug("qfa: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);

    Eigen::Quaterniond qFA_record;
    qFA_record.w() = qFA[0];
    qFA_record.x() = qFA[1];
    qFA_record.y() = qFA[2];
    qFA_record.z() = qFA[3];

    int boolBuzz = 0;
    //    if ((_cnt + 250) % buzzN == 0) {
    //        _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
    //        boolBuzz = 1;
    //    }

    if (_cnt == 0) {
        _lawimu.initialization();
    } else if (_cnt <= init_cnt) {
        _lawimu.initialPositions(qFA_record, _cnt, init_cnt);
    } else {
        _lawimu.rotationMatrices(qFA_record);
        _lawimu.controlLawWrist(_lambdaW, _thresholdW * M_PI / 180);

        _robot->joints.wrist_pronation->set_velocity_safe(_lawimu.returnWristVel_deg());
        //        _robot->joints.wrist_pronation->forward(40);

        if (_cnt % 50 == 0) {
            _lawimu.displayData();
            // qDebug("lambdaW: %d", _lambdaW);
            //            printf("lambdaW: %d\n", _lambdaW);
        }
    }

    _lawimu.writeDebugData(debugData);

    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;

    /// read EMG
    // Set configuration bits
    //    _emg[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    //    _emg[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    //    debug() << "emg : " << _emg[0] << "; " << _emg[1];
    //    uint16_t config_global = ADS1015_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
    //        ADS1015_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
    //        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
    //        ADS1015_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
    //        ADS1015_REG_CONFIG_DR_3300SPS | // 3300 samples per second (ie 860sps pour un ADS1115)
    //        ADS1015_REG_CONFIG_MODE_SINGLE | // Single-shot mode (default)
    //        _robot->sensors.adc0->getGain() | //Set PGA/voltage range
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
    //    // Read the conversion results
    //    _emg[0] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    //    //Write config register to the ADC
    //    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c3);
    //    // Wait for the conversion to complete
    //    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
    //        ;
    //    // Read the conversion results
    //    _emg[1] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    //    for (uint16_t i = 0; i < 2; i++) {
    //        if (_emg[i] > 65500) {
    //            _emg[i] = 0;
    //        }
    //    }

    _file << timeWithDelta << ' ' << pin_down_value << ' ' << pin_up_value << ' ' << boolBuzz << ' ' << _emg[0] << ' ' << _emg[1];
    _file << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    _file << ' ' << qFA[0] << ' ' << qFA[1] << ' ' << qFA[2] << ' ' << qFA[3];
    _file << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << debugData[3];
    _file << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << wristAngleEncoder;
    _file << ' ' << data.nRigidBodies;

    for (int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    _file << std::endl;

    ++_cnt;
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void CompensationIMU::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    _file.close();
}
