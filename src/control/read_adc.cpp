#include "read_adc.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include <filesystem>
#include <iostream>
#include "utils/check_ptr.h"

// indicate if optitrack is on
#define OPTITRACK 1

ReadADC::ReadADC(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Read ADC", .025)
    , _robot(robot)
{
    _menu->set_description("Read ADC");
    _menu->set_code("adc");
}

ReadADC::~ReadADC()
{
    stop_and_join();
}


void ReadADC::readAllADC() //Optimized function to read all 6 electrodes
{
    clock::time_point time1 = clock::now();

    // Set configuration bits
    uint16_t config_global = ADS1015_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
        ADS1015_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
        ADS1015_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
        ADS1015_REG_CONFIG_DR_3300SPS | // 3300 samples per second (ie 860sps pour un ADS1115)
        ADS1015_REG_CONFIG_MODE_SINGLE | // Single-shot mode (default)
        _robot->sensors.adc0->getGain() | //Set PGA/voltage range
        ADS1015_REG_CONFIG_OS_SINGLE; // Set 'start single-conversion' bit

    uint16_t config_c0 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_0; //for channel 0
    uint16_t config_c1 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_1; //for channel 1
    uint16_t config_c2 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_2; //for channel 2
    uint16_t config_c3 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_3; //for channel 3

    //Write config register to the ADC
    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);
    _robot->sensors.adc2->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);
    _robot->sensors.adc3->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);

    // Wait for the conversion to complete
    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
        ;

    // Read the conversion results
    _electrodes[0] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[2] = _robot->sensors.adc2->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[4] = _robot->sensors.adc3->readRegister(ADS1015_REG_POINTER_CONVERT);

    //Write config register to the ADC
    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c3);
    _robot->sensors.adc2->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c0);
    _robot->sensors.adc3->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c0);

    // Wait for the conversion to complete
    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
        ;

    // Read the conversion results
    _electrodes[1] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[3] = _robot->sensors.adc2->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[5] = _robot->sensors.adc3->readRegister(ADS1015_REG_POINTER_CONVERT);

//    for (uint16_t i = 0; i < _n_electrodes; i++) {
//        if (_electrodes[i] > 65500) {
//            _electrodes[i] = 0;
//        }
//    }
}

bool ReadADC::setup()
{
    _robot->user_feedback.leds->set(LedStrip::white, 11);

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _electrodes[i] = 0;
    }

    if (saveData) {
        std::string filename("myo");
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
    }

    _start_time = clock::now();
    return true;
}

void ReadADC::loop(double, clock::time_point time)
{
    double timeWithDelta = std::chrono::duration_cast<std::chrono::microseconds>(time - _start_time).count();

    readAllADC();

    //Publish EMG values with MQTT
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
        std::cout << _electrodes[i] << "\t";
    }
    std::cout << std::endl;

    // Write in .txt file
    if (saveData) {
    _file << timeWithDelta << "\t" << _electrodes[0] << "\t" << _electrodes[1] << "\t" << _electrodes[2] << "\t" << _electrodes[3] << "\t" << _electrodes[4] << "\t" << _electrodes[5];
    _file << std::endl;
    }
}

void ReadADC::cleanup()
{
    _file.close();
}
