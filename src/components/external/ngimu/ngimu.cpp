#include "ngimu.h"
#include "utils/log/log.h"
#include <iostream>
#include <unistd.h>

NGIMU::NGIMU(std::string filename, unsigned int baudrate)
    : ThreadedLoop(filename.substr(5), 0.001)
    , received_error(false)
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
    imudata_humidity = 0.0;
    for (int i = 0; i < 2; i++) {
        imudata_temperature[i] = 0.0;
        imudata_euler[i] = 0.0;
        imudata_quat[i] = 0.0;
        imudata_sensors[i] = 0.0;
        imudata_batt[i] = 0.0;
    }
    imudata_euler[2] = 0.0;
    for (int i = 2; i < 4; i++) {
        imudata_quat[i] = 0.0;
        imudata_sensors[i] = 0.0;
        imudata_batt[i] = 0.0;
    }
    for (int i = 4; i < 10; i++) {
        imudata_sensors[i] = 0.0;
    }

    imudata_euler_available = false;
    imudata_quat_available = false;
    imudata_sensors_available = false;
    imudata_humidity_available = false;
    imudata_batt_available = false;
    serialnumber_available = false;
}

void NGIMU::loop(double, clock::time_point)
{
    char addressPattern[MAX_OSC_ADDRESS_PATTERN_LENGTH + 1];

    std::vector<std::byte> buf = _sp.read_all();
    if (buf.size() == 0) {
        return;
    }

//    for(unsigned int i=0; i< buf.size(); i++)
//        std::cout << static_cast<char>(buf[i]);
//    std::cout << std::endl;

    OscError oscError = OscErrorNone;
    for (unsigned int i = 0; i < buf.size(); i++) {
        oscError = _oscSlipDecoder.OscSlipProcessByte(static_cast<char>(buf[i]), &_oscMessage, &_oscTimeTag);
        if (oscError != OscErrorNone) {
            debug() << OscErrorGetMessage(oscError);
            return;
        } else if (static_cast<char>(buf[i]) == static_cast<char>(0xC0)) {
            _oscMessage.OscMessageGetAddressPattern(addressPattern);
            if (OscAddressMatch(addressPattern, "/sensors")) {
                oscError = ProcessSensors(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/quaternion")) {
                oscError = ProcessQuaternion(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/euler")) {
                oscError = ProcessEuler(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/battery")) {
                oscError = ProcessBattery(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/humidity")) {
                oscError = ProcessHumidity(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/temperature")) {
                oscError = ProcessTemperature(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/serialnumber")) {
                oscError = ProcessSerialNumber(_oscMessage);
            } else if (OscAddressMatch(addressPattern, "/error")) {
                debug() << "NGIMU receive an OSC message with address pattern /error";
                std::lock_guard scoped_lock(_mutex);
                received_error = true;
            } else {
                debug() << "OSC address pattern not recognised : " << addressPattern;
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
OscError NGIMU::ProcessSensors(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    // Get gyroscope X axis
    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get gyroscope Y axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get gyroscope Z axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer X axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[3]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer Y axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[4]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer Z axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[5]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer X axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[6]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer Y axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[7]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer Z axis
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[8]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get barometer
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_sensors[9]);
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
OscError NGIMU::ProcessQuaternion(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    // Get W element
    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get X element
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Y element
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Z element
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_quat[3]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

//    for (int i=0; i<4; i++) {
//        std::cout << imudata_quat[i] << "\t";
//    }
//    std::cout << std::endl;

    imudata_quat_available = true;
    return OscErrorNone;
}

/**
 * @brief Process "/euler" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessEuler(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    // Get roll
    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_euler[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get pitch
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_euler[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yaw
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_euler[2]);
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
OscError NGIMU::ProcessBattery(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    // Get battery level in %
    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get time to empty battery in minutes
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get battery voltage in V
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[2]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get battery current in mA
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_batt[3]);
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
OscError NGIMU::ProcessHumidity(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    // Get relative humidity in %
    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_humidity);
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
OscError NGIMU::ProcessTemperature(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    // Get gyroscope/accelerometer temperature in °C
    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_temperature[0]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get barometer temperature in °C
    oscError = oscMessage.OscMessageGetArgumentAsDouble(&imudata_temperature[1]);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    imudata_temperature_available = true;
    return OscErrorNone;
}


/**
 * @brief Process "/serialnumber" message.
 * @return Error code (0 if successful).
 */
OscError NGIMU::ProcessSerialNumber(OscMessage oscMessage)
{
    std::lock_guard scoped_lock(_mutex);

    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsString(_serial_number, sizeof(_serial_number));
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return oscError;
    } else {
        serialnumber_available = true;
        info() << "NGIMU serial number : " << _serial_number;
    }

    return OscErrorNone;
}


//Send an oscMessage without argument
bool NGIMU::send_command_message(const char* const commandAddress)
{
    OscMessage oscMessage;
    OscError oscError;

    oscError = oscMessage.OscMessageInitialise(commandAddress);
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }
    return send_command(&oscMessage);
}

//Send an oscMessage with a boolean argument
bool NGIMU::send_command_message(const char* const commandAddress, const bool argument)
{
    OscMessage oscMessage;
    OscError oscError;

    oscError = oscMessage.OscMessageInitialise(commandAddress);
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    oscError = oscMessage.OscMessageAddBool(argument);
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    return send_command(&oscMessage);
}

// Receive an oscContents (oscMessage or oscBundle) and send it via the serial port
bool NGIMU::send_command(const void* const oscContents)
{
    OscPacket oscPacket;
    OscError oscError;

    oscError = oscPacket.OscPacketInitialiseFromContents(oscContents);
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    char slipPacket[MAX_OSC_PACKET_SIZE];
    size_t slipPacketSize;
    OscSlipDecoder oscSlipDecoder;
    oscError = oscSlipDecoder.OscSlipEncodePacket(&oscPacket, &slipPacketSize, slipPacket, sizeof(slipPacket));
    if (oscError != OscErrorNone) {
        debug() << OscErrorGetMessage(oscError);
        return false;
    }

    unsigned int retries = 10; //Tries 10 times to send the message
    do {
        received_error = false;
        _sp.write(slipPacket, slipPacketSize);
        usleep(200 * 1000);
        retries--;
    } while (received_error == true && (retries > 0));

    received_error = false;

    //success?
    if (retries != 0) {
        return true;
    } else {
        return false;
    }
}

bool NGIMU::send_command_identify()
{
    return send_command_message("/identify");
}

bool NGIMU::send_command_algorithm_init()
{
    return send_command_message("/ahrs/initialise");
}

bool NGIMU::send_command_serial_number()
{
    return send_command_message("/serialnumber");
}

std::string NGIMU::get_serialnumber()
{
    std::lock_guard scoped_lock(_mutex); //will be freed on function exit
    std::string nb(_serial_number);
    return nb;
}

bool NGIMU::is_serialnumber_available()
{
    return serialnumber_available;
}
