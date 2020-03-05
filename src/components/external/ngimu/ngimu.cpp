#include "ngimu.h"
#include "utils/log/log.h"
#include <iostream>
#include <unistd.h>


NGIMU::NGIMU(std::string filename, unsigned int baudrate)
    : ThreadedLoop(filename.substr(5), 0.001)
{
    _sp.open(filename, baudrate);
    init_imudata();

    info() << " ### NGIMU : NGIMU created at address " << filename.substr(5);
    start();
}

NGIMU::~NGIMU()
{
    stop_and_join();
    _sp.close();
}

void NGIMU::init_imudata()
{
    //init all data
    for (int i = 0; i < 3; i++) {
        imudata_euler[i] = 0.0;
        imudata_quat[i] = 0.0;
        imudata_sensors[i] = 0.0;
    }
    imudata_quat[3] = 0.0;
    for (int i = 3; i < 10; i++) {
        imudata_sensors[i] = 0.0;
    }

    imudata_euler_available = false;
    imudata_quat_available = false;
    imudata_sensors_available = false;
}

void NGIMU::loop(double, clock::time_point)
{
    static OscMessage oscMessage;
    static OscTimeTag oscTimeTag;
    static char addressPattern[MAX_OSC_ADDRESS_PATTERN_LENGTH + 1];

    std::vector<std::byte> buf = _sp.read_all();
    if (buf.size() == 0) {
        return;
    }

    OscError oscError = OscErrorNone;
    for (unsigned int i = 0; i < buf.size(); i++) {
        oscError = _oscSlipDecoder.OscSlipProcessByte(static_cast<char>(buf[i]), &oscMessage, &oscTimeTag);
        if (oscError !=OscErrorNone) {
            debug() << OscErrorGetMessage(oscError);
            return;
        } else if (static_cast<char>(buf[i]) == static_cast<char>(0xC0) ) {
            oscMessage.OscMessageGetAddressPattern(addressPattern);
            if (OscAddressMatch(addressPattern, "/sensors")) {
                oscError = ProcessSensors(oscMessage);
            } else if (OscAddressMatch(addressPattern, "/quaternion")) {
                oscError = ProcessQuaternion(oscMessage);
            } else if (OscAddressMatch(addressPattern, "/euler")) {
                oscError = ProcessEuler(oscMessage);
            } else if (OscAddressMatch(addressPattern, "/battery")) {
                oscError = ProcessBattery(oscMessage);
            } else if (OscAddressMatch(addressPattern, "/humidity")) {
                oscError = ProcessHumidity(oscMessage);
            } else if (OscAddressMatch(addressPattern, "/temperature")) {
                oscError = ProcessTemperature(oscMessage);
            } else {
                debug() << "OSC address pattern not recognised: " << addressPattern;
            }
        }
    }
}

bool NGIMU::get_sensors(double* s)
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    bool result = imudata_sensors_available;

    //access data
    s[0] = imudata_sensors[0]; // gyroscope X axis in °/s
    s[1] = imudata_sensors[1]; // gyroscope Y axis in °/s
    s[2] = imudata_sensors[2]; // gyroscope Z axis in °/s
    s[3] = imudata_sensors[3]; // accelerometer X axis in g
    s[4] = imudata_sensors[4]; // accelerometer Y axis in g
    s[5] = imudata_sensors[5]; // accelerometer Z axis in g
    s[6] = imudata_sensors[6]; // magnetometer X axis in µT
    s[7] = imudata_sensors[7]; // magnetometer Y axis in µT
    s[8] = imudata_sensors[8]; // magnetometer Z axis in µT
    s[9] = imudata_sensors[9]; // barometer in hPa

    imudata_sensors_available = false;

    return result;
}

bool NGIMU::get_euler(double* e)
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    bool result = imudata_euler_available;

    //access data
    e[0] = imudata_euler[0]; // roll angle in degrees
    e[1] = imudata_euler[1]; // pitch angle in degrees
    e[2] = imudata_euler[2]; // yaw angle in degrees
    imudata_euler_available = false;

    return result;
}

bool NGIMU::get_quat(double* q)
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    bool result = imudata_quat_available;

    //access data
    q[0] = imudata_quat[0]; // w element
    q[1] = imudata_quat[1]; // x element
    q[2] = imudata_quat[2]; // y element
    q[3] = imudata_quat[3]; // z element
    imudata_quat_available = false;

    //std::cout << imudata_quat[0] << "\t" << imudata_quat[1] << "\t" << imudata_quat[2] << "\t" << imudata_quat[3] << std::endl;

    return result;
}

bool NGIMU::get_battery(double* b)
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    bool result = imudata_batt_available;

    //access data
    b[0] = imudata_batt[0]; // battery level in %
    b[1] = imudata_batt[1]; // time to empty battery in minutes
    b[2] = imudata_batt[2]; // battery voltage in V
    b[3] = imudata_batt[3]; // battery current in mA
    imudata_batt_available = false;

    return result;
}

bool NGIMU::get_humidity(double h)
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    bool result = imudata_humidity_available;

    h = imudata_humidity; // relative humidity in %
    imudata_humidity_available = false;

    return result;
}

bool NGIMU::get_temperature(double* t)
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    bool result = imudata_temperature_available;

    //access data
    t[0] = imudata_temperature[0]; // gyroscope/accelerometer temperature in °C
    t[1] = imudata_temperature[1]; // barometer temperature in °C
    imudata_temperature_available = false;

    return result;
}


/**
 * @brief Process "/sensors" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessSensors(OscMessage current_oscMessage) {

    std::lock_guard scoped_lock(_mutex);

    // Get gyroscope X axis
    OscError oscError;
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get gyroscope Y axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get gyroscope Z axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer X axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[3]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer Y axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[4]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer Z axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[5]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer X axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[6]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer Y axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[7]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer Z axis
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[8]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get barometer
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[9]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    imudata_sensors_available = true;
    return OscErrorNone;
}

/**
 * @brief Process "/quaternion" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessQuaternion(OscMessage current_oscMessage) {

    std::lock_guard scoped_lock(_mutex);

    // Get W element
    OscError oscError;
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get X element
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Y element
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Z element
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[3]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    for (int i=0; i<4; i++) {
        std::cout << imudata_quat[i] << "\t";
    }
    std::cout << std::endl;

    imudata_quat_available = true;
    return OscErrorNone;
}

/**
 * @brief Process "/euler" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessEuler(OscMessage current_oscMessage) {

    std::lock_guard scoped_lock(_mutex);

    // Get roll
    OscError oscError;
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_euler[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get pitch
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_euler[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yaw
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_euler[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    imudata_euler_available = true;
    return OscErrorNone;
}


/**
 * @brief Process "/battery" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessBattery(OscMessage current_oscMessage) {

    std::lock_guard scoped_lock(_mutex);

    // Get battery level in %
    OscError oscError;
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get time to empty battery in minutes
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get battery voltage in V
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get battery current in mA
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[3]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Could be possible to get charger state (string)

    imudata_batt_available = true;
    return OscErrorNone;
}

/**
 * @brief Process "/humidity" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessHumidity(OscMessage current_oscMessage) {

    std::lock_guard scoped_lock(_mutex);

    // Get relative humidity in %
    OscError oscError;
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_humidity);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    imudata_humidity_available = true;
    return OscErrorNone;
}

/**
 * @brief Process "/temperature" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessTemperature(OscMessage current_oscMessage) {

    std::lock_guard scoped_lock(_mutex);

    // Get gyroscope/accelerometer temperature in °C
    OscError oscError;
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_temperature[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get barometer temperature in °C
    oscError = current_oscMessage.OscMessageGetArgumentAsDouble(&imudata_temperature[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    imudata_temperature_available = true;
    return OscErrorNone;
}






bool NGIMU::send_command_identify() {
    OscMessage oscMessage;
    OscError oscError;
    oscError = oscMessage.OscMessageInitialise("/identify");
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    OscPacket oscPacket;
    oscError = oscPacket.OscPacketInitialiseFromContents(&oscMessage);
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    return send_command(oscPacket);
}


bool NGIMU::send_command(OscPacket oscPacket) {
    OscError oscError;
    char slipPacket[MAX_OSC_PACKET_SIZE];
    size_t slipPacketSize;
    OscSlipDecoder oscSlipDecoder;
    oscError = oscSlipDecoder.OscSlipEncodePacket(&oscPacket, &slipPacketSize, slipPacket, sizeof(slipPacket));
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    _sp.write(slipPacket, slipPacketSize);
    usleep(200 * 1000);

    return true;
}
