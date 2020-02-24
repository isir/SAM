//NGIMU library based on the NgimuReceive library from Seb Madgwick

#ifndef NGIMU_H
#define NGIMU_H

#include "utils/serial_port.h"
#include "utils/threaded_loop.h"
#include <mutex>
#include "ux/Osc99/Osc99.h"

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

    bool send_command();
    bool send_command_identify();

private:
    void loop(double dt, clock::time_point time) override;
    void init_imudata();

    OscError ProcessSensors(OscMessage current_oscMessage);
    OscError ProcessQuaternion(OscMessage current_oscMessage);
    OscError ProcessEuler(OscMessage current_oscMessage);
    OscError ProcessBattery(OscMessage current_oscMessage);
    OscError ProcessHumidity(OscMessage current_oscMessage);
    OscError ProcessTemperature(OscMessage current_oscMessage);

    SerialPort _sp;
    OscSlipDecoder _oscSlipDecoder;

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

    //interprocess_semaphore  data_access_mutex;
    std::mutex _mutex;
};

#endif // NGIMU_H


