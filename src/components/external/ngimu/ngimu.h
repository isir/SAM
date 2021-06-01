//NGIMU library based on the NgimuReceive library from Seb Madgwick

#ifndef NGIMU_H
#define NGIMU_H

#include "utils/serial_port.h"
#include "utils/threaded_loop.h"
#include <mutex>
#include "Osc99/Osc99.h"
#include <string>

#define NGIMU_BAUDRATE B115200

class NGIMU : public ThreadedLoop {
public:
    NGIMU(std::string filename, unsigned int baudrate = NGIMU_BAUDRATE);
    ~NGIMU() override;

    bool get_sensors(double* s);
    bool get_euler(double* e);
    bool get_quat(double* q);
    bool get_battery(double* b);
    bool get_humidity(double h);
    bool get_temperature(double* t);

    std::string get_serialnumber();
    bool is_serialnumber_available();

    bool send_command_message(const char* const commandAddress);
    bool send_command_message(const char* const commandAddress, const bool argument);
    bool send_command(const void* const oscContents);
    bool send_command_identify();
    bool send_command_algorithm_init();
    bool send_command_serial_number();

private:
    void loop(double dt, clock::time_point time) override;
    void init_imudata();

    OscError ProcessSensors(OscMessage oscMessage);
    OscError ProcessQuaternion(OscMessage oscMessage);
    OscError ProcessEuler(OscMessage oscMessage);
    OscError ProcessBattery(OscMessage oscMessage);
    OscError ProcessHumidity(OscMessage oscMessage);
    OscError ProcessTemperature(OscMessage oscMessage);
    OscError ProcessSerialNumber (OscMessage oscMessage);

    SerialPort _sp;
    OscSlipDecoder _oscSlipDecoder;
    OscMessage _oscMessage;
    OscTimeTag _oscTimeTag;

    char _serial_number[9];
    bool serialnumber_available;

    //sensors data
    double imudata_sensors[10];
    bool imudata_sensors_available;
    //euler data
    double imudata_euler[3];
    bool imudata_euler_available;
    // quat data
    double imudata_quat[4];
    bool imudata_quat_available;
    // battery data
    double imudata_batt[4];
    bool imudata_batt_available;
    // humidity data
    double imudata_humidity;
    bool imudata_humidity_available;
    // temperature data
    double imudata_temperature[2];
    bool imudata_temperature_available;

    bool received_error;

    //interprocess_semaphore  data_access_mutex;
    std::mutex _mutex;
};

#endif // NGIMU_H


