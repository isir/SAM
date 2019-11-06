/*
* This file is part of libximu
*
* Copyright(c) sschulz <AT> techfak.uni-bielefeld.de
* http://opensource.cit-ec.de/projects/libximu
*
* This file may be licensed under the terms of the
* GNU Lesser General Public License Version 3 (the ``LGPL''),
* or (at your option) any later version.
*
* Software distributed under the License is distributed
* on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
* express or implied. See the LGPL for the specific language
* governing rights and limitations.
*
* You should have received a copy of the LGPL along with this
* program. If not, go to http://www.gnu.org/licenses/lgpl.html
* or write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
* The development of this software was supported by the
* Excellence Cluster EXC 277 Cognitive Interaction Technology.
* The Excellence Cluster EXC 277 is a grant of the Deutsche
* Forschungsgemeinschaft (DFG) in the context of the German
* Excellence Initiative.
*/
#ifndef XIMU_H
#define XIMU_H

#include "utils/serial_port.h"
#include "utils/threaded_loop.h"

#include <mutex>
#include <string>
#include <termios.h>

#define XIMU_BAUDRATE B115200

#define RX_PACKET_BUFFER_SIZE 1024
#define XIMU_READ_REGISTER_TRIES 5

#define XIMU_SUPPORTED_FIRMWARE_REV_MAJOR 9
#define XIMU_SUPPORTED_FIRMWARE_REV_MINOR 6

class XIMU : public ThreadedLoop {
public:
    XIMU(std::string filename, int level = XIMU_LOGLEVEL_NONE, unsigned int baudrate = XIMU_BAUDRATE);
    ~XIMU() override;

    int set_loglevel(int i);
    int detect_device();

    bool send_command_algorithm_init();
    bool send_command_algorithm_init_then_tare();
    bool send_command_algorithm_tare();
    bool send_command_algorithm_clear_tare();
    bool send_command(unsigned int command_code);

    bool get_device_detected();

    int get_register(unsigned int register_address, unsigned int* val);
    bool get_euler(double* e);
    bool get_quat(double* e);
    bool get_cal(double* gyro, double* accel, double* mag);
    bool get_time(struct tm* time);

    bool areQuatConsistent(double currentQuatW, double currentQuatX, double currentQuatY, double currentQuatZ);

    enum XIMU_LOGLEVEL {
        XIMU_LOGLEVEL_NONE,
        XIMU_LOGLEVEL_LOG,
        XIMU_LOGLEVEL_DEBUG
    };

    enum XIMU_PACKET_HEADER {
        PACKET_HEADER_ERROR,
        PACKET_HEADER_COMMAND,
        PACKET_HEADER_READREGISTER,
        PACKET_HEADER_WRITEREGISTER,
        PACKET_HEADER_READDATETIME,
        PACKET_HEADER_WRITEDATETIME,
        PACKET_HEADER_RAWBATTERYANDTHERMOMETERDATA,
        PACKET_HEADER_CALBATTERYANDTHERMOMETERDATA,
        PACKET_HEADER_RAWINERTIALANDMAGNETICDATA,
        PACKET_HEADER_CALINERTIALANDMAGNETICDATA,
        PACKET_HEADER_QUATERNIONDATA,
        PACKET_HEADER_DIGITALIODATA,
        PACKET_HEADER_RAWANALOGUEINPUTDATA,
        PACKET_HEADER_CALANALOGUEINPUTDATA,
        PACKET_HEADER_PWMOUTPUTDATA,
        PACKET_HEADER_RAWADXL345BUSDATA,
        PACKET_HEADER_CALADXL345BUSDATA
    };

    enum XIMU_ERROR_CODES {
        ERROR_CODE_NoError,
        ERROR_CODE_FactoryResetFailed,
        ERROR_CODE_LowBattery,
        ERROR_CODE_USBreceiveBufferOverrun,
        ERROR_CODE_USBtransmitBufferOverrun,
        ERROR_CODE_BluetoothReceiveBufferOverrun,
        ERROR_CODE_BluetoothTransmitBufferOverrun,
        ERROR_CODE_SDcardWriteBufferOverrun,
        ERROR_CODE_TooFewBytesInPacket,
        ERROR_CODE_TooManyBytesInPacket,
        ERROR_CODE_InvalidChecksum,
        ERROR_CODE_UnknownHeader,
        ERROR_CODE_InvalidNumBytesForPacketHeader,
        ERROR_CODE_InvalidRegisterAddress,
        ERROR_CODE_RegisterReadOnly,
        ERROR_CODE_InvalidRegisterValue,
        ERROR_CODE_InvalidCommand,
        ERROR_CODE_GyroscopeAxisNotAt200dps,
        ERROR_CODE_GyroscopeNotStationary,
        ERROR_CODE_AcceleroemterAxisNotAt1g,
        ERROR_CODE_MagnetometerSaturation,
        ERROR_CODE_IncorrectAuxillaryPortMode,
        ERROR_CODE_UARTreceiveBufferOverrun,
        ERROR_CODE_UARTtransmitBufferOverrun
    };

    enum XIMU_COMMAND_CODES {
        COMMAND_CODE_NullCommand,
        COMMAND_CODE_FactoryReset,
        COMMAND_CODE_Reset,
        COMMAND_CODE_Sleep,
        COMMAND_CODE_ResetSleepTimer,
        COMMAND_CODE_SampleGyroscopeAxisAt200dps,
        COMMAND_CODE_CalculateGyroscopeSensitivity,
        COMMAND_CODE_SampleGyroscopeBiasTemp1,
        COMMAND_CODE_SampleGyroscopeBiasTemp2,
        COMMAND_CODE_CalculateGyroscopeBiasParameters,
        COMMAND_CODE_SampleAccelerometerAxisAt1g,
        COMMAND_CODE_CalculateAccelerometerBiasAndSensitivity,
        COMMAND_CODE_MeasureMagnetometerBiasAndSensitivity,
        COMMAND_CODE_AlgorithmInitialise,
        COMMAND_CODE_AlgorithmTare,
        COMMAND_CODE_AlgorithmClearTare,
        COMMAND_CODE_AlgorithmInitialiseThenTare
    };

    enum XIMU_REGISTER_ADDRESSES {
        REGISTER_ADDRESS_FirmwareVersionMajorNum,
        REGISTER_ADDRESS_FirmwareVersionMinorNum,
        REGISTER_ADDRESS_DeviceID,
        REGISTER_ADDRESS_ButtonMode,
        REGISTER_ADDRESS_BatterySensitivity,
        REGISTER_ADDRESS_BatteryBias,
        REGISTER_ADDRESS_ThermometerSensitivity,
        REGISTER_ADDRESS_ThermometerBias,
        REGISTER_ADDRESS_GyroscopeFullScale,
        REGISTER_ADDRESS_GyroscopeSensitivityX,
        REGISTER_ADDRESS_GyroscopeSensitivityY,
        REGISTER_ADDRESS_GyroscopeSensitivityZ,
        REGISTER_ADDRESS_GyroscopeSampledPlus200dpsX,
        REGISTER_ADDRESS_GyroscopeSampledPlus200dpsY,
        REGISTER_ADDRESS_GyroscopeSampledPlus200dpsZ,
        REGISTER_ADDRESS_GyroscopeSampledMinus200dpsX,
        REGISTER_ADDRESS_GyroscopeSampledMinus200dpsY,
        REGISTER_ADDRESS_GyroscopeSampledMinus200dpsZ,
        REGISTER_ADDRESS_GyroscopeBiasAt25degCX,
        REGISTER_ADDRESS_GyroscopeBiasAt25degCY,
        REGISTER_ADDRESS_GyroscopeBiasAt25degCZ,
        REGISTER_ADDRESS_GyroscopeBiasTempSensitivityX,
        REGISTER_ADDRESS_GyroscopeBiasTempSensitivityY,
        REGISTER_ADDRESS_GyroscopeBiasTempSensitivityZ,
        REGISTER_ADDRESS_GyroscopeSample1Temp,
        REGISTER_ADDRESS_GyroscopeSample1BiasX,
        REGISTER_ADDRESS_GyroscopeSample1BiasY,
        REGISTER_ADDRESS_GyroscopeSample1BiasZ,
        REGISTER_ADDRESS_GyroscopeSample2Temp,
        REGISTER_ADDRESS_GyroscopeSample2BiasX,
        REGISTER_ADDRESS_GyroscopeSample2BiasY,
        REGISTER_ADDRESS_GyroscopeSample2BiasZ,
        REGISTER_ADDRESS_AccelerometerFullScale,
        REGISTER_ADDRESS_AccelerometerSensitivityX,
        REGISTER_ADDRESS_AccelerometerSensitivityY,
        REGISTER_ADDRESS_AccelerometerSensitivityZ,
        REGISTER_ADDRESS_AccelerometerBiasX,
        REGISTER_ADDRESS_AccelerometerBiasY,
        REGISTER_ADDRESS_AccelerometerBiasZ,
        REGISTER_ADDRESS_AccelerometerSampledPlus1gX,
        REGISTER_ADDRESS_AccelerometerSampledPlus1gY,
        REGISTER_ADDRESS_AccelerometerSampledPlus1gZ,
        REGISTER_ADDRESS_AccelerometerSampledMinus1gX,
        REGISTER_ADDRESS_AccelerometerSampledMinus1gY,
        REGISTER_ADDRESS_AccelerometerSampledMinus1gZ,
        REGISTER_ADDRESS_MagnetometerFullScale,
        REGISTER_ADDRESS_MagnetometerSensitivityX,
        REGISTER_ADDRESS_MagnetometerSensitivityY,
        REGISTER_ADDRESS_MagnetometerSensitivityZ,
        REGISTER_ADDRESS_MagnetometerBiasX,
        REGISTER_ADDRESS_MagnetometerBiasY,
        REGISTER_ADDRESS_MagnetometerBiasZ,
        REGISTER_ADDRESS_MagnetometerHardIronBiasX,
        REGISTER_ADDRESS_MagnetometerHardIronBiasY,
        REGISTER_ADDRESS_MagnetometerHardIronBiasZ,
        REGISTER_ADDRESS_AlgorithmMode,
        REGISTER_ADDRESS_AlgorithmKp,
        REGISTER_ADDRESS_AlgorithmKi,
        REGISTER_ADDRESS_AlgorithmInitKp,
        REGISTER_ADDRESS_AlgorithmInitPeriod,
        REGISTER_ADDRESS_AlgorithmMinValidMag,
        REGISTER_ADDRESS_AlgorithmMaxValidMag,
        REGISTER_ADDRESS_AlgorithmTareQuat0,
        REGISTER_ADDRESS_AlgorithmTareQuat1,
        REGISTER_ADDRESS_AlgorithmTareQuat2,
        REGISTER_ADDRESS_AlgorithmTareQuat3,
        REGISTER_ADDRESS_SensorDataMode,
        REGISTER_ADDRESS_DateTimeDataRate,
        REGISTER_ADDRESS_BatteryAndThermometerDataRate,
        REGISTER_ADDRESS_InertialAndMagneticDataRate,
        REGISTER_ADDRESS_QuaternionDataRate,
        REGISTER_ADDRESS_SDcardNewFileName,
        REGISTER_ADDRESS_BatteryShutdownVoltage,
        REGISTER_ADDRESS_SleepTimer,
        REGISTER_ADDRESS_MotionTrigWakeUp,
        REGISTER_ADDRESS_BluetoothPower,
        REGISTER_ADDRESS_AuxiliaryPortMode,
        REGISTER_ADDRESS_DigitalIOdirection,
        REGISTER_ADDRESS_DigitalIOdataRate,
        REGISTER_ADDRESS_AnalogueInputDataMode,
        REGISTER_ADDRESS_AnalogueInputDataRate,
        REGISTER_ADDRESS_AnalogueInputSensitivity,
        REGISTER_ADDRESS_AnalogueInputBias,
        REGISTER_ADDRESS_PWMoutputFrequency,
        REGISTER_ADDRESS_ADXL345busDataMode,
        REGISTER_ADDRESS_ADXL345busDataRate,
        REGISTER_ADDRESS_ADXL345AsensitivityX,
        REGISTER_ADDRESS_ADXL345AsensitivityY,
        REGISTER_ADDRESS_ADXL345AsensitivityZ,
        REGISTER_ADDRESS_ADXL345AbiasX,
        REGISTER_ADDRESS_ADXL345AbiasY,
        REGISTER_ADDRESS_ADXL345AbiasZ,
        REGISTER_ADDRESS_ADXL345BsensitivityX,
        REGISTER_ADDRESS_ADXL345BsensitivityY,
        REGISTER_ADDRESS_ADXL345BsensitivityZ,
        REGISTER_ADDRESS_ADXL345BbiasX,
        REGISTER_ADDRESS_ADXL345BbiasY,
        REGISTER_ADDRESS_ADXL345BbiasZ,
        REGISTER_ADDRESS_ADXL345CsensitivityX,
        REGISTER_ADDRESS_ADXL345CsensitivityY,
        REGISTER_ADDRESS_ADXL345CsensitivityZ,
        REGISTER_ADDRESS_ADXL345CbiasX,
        REGISTER_ADDRESS_ADXL345CbiasY,
        REGISTER_ADDRESS_ADXL345CbiasZ,
        REGISTER_ADDRESS_ADXL345DsensitivityX,
        REGISTER_ADDRESS_ADXL345DsensitivityY,
        REGISTER_ADDRESS_ADXL345DsensitivityZ,
        REGISTER_ADDRESS_ADXL345DbiasX,
        REGISTER_ADDRESS_ADXL345DbiasY,
        REGISTER_ADDRESS_ADXL345DbiasZ,
        REGISTER_ADDRESS_UARTbaudRate,
        REGISTER_ADDRESS_UARThardwareFlowControl,
        REGISTER_ADDRESS_NumRegisters
    };

private:
    void loop(double dt, clock::time_point time) override;

    void init_imudata();

    bool check_len(int len, int check);

    void send_register_read_request(unsigned int reg);

    void send_packet(unsigned char* buf, unsigned int len);
    void process_packet(unsigned char* packet_ptr, int len);
    int decode_packet(unsigned char* packet, int len);
    void process_packet_error_data(unsigned char* ptr, int len);
    void process_packet_command_data(unsigned char* ptr, int len);
    void process_packet_write_register(unsigned char* ptr, int len);
    void process_packet_write_datetime(unsigned char* ptr, int len);
    void process_packet_raw_battery_and_thermometer_data(unsigned char* ptr, int len);
    void process_packet_cal_battery_and_thermometer_data(unsigned char* ptr, int len);
    void process_packet_raw_inertialandmagnetic_data(unsigned char* ptr, int len);
    void process_packet_cal_inertialandmagnetic_data(unsigned char* ptr, int len);
    void process_packet_quaternion_data(unsigned char* ptr, int len);
    void process_digital_io_data(unsigned char* packet_ptr, int len);
    void process_raw_analog_data(unsigned char* packet_ptr, int len);
    void process_cal_analog_data(unsigned char* packet_ptr, int len);
    void process_pwm_data(unsigned char* packet_ptr, int len);
    void process_raw_adxl_bus_data(unsigned char* ptr, int len);
    void process_cal_adxl_bus_data(unsigned char* packet_ptr, int len);

    void left_shift(unsigned char* packet, int len);
    void right_shift(unsigned char* packet, int len);

    float to_float(unsigned char hi, unsigned char lo, unsigned int q)
    {
        int16_t val = (hi << 8) | lo;
        return to_float(val, q);
    }
    float to_float(int d, unsigned int q) { return ((float)d) / ((float)(1 << q)); }

    SerialPort _sp;

    std::vector<std::byte> _rx_packet_buffer;
    int loglevel;
    bool device_detected;

    //euler data
    double imudata_euler[3];
    bool imudata_euler_available;
    // quat data
    double imudata_quat[4];
    bool imudata_quat_available;
    const int NB_OLD_QUAT = 100;
    double _oldQuatValues[100][4];
    //cal data
    double imudata_cal[3][3];
    bool imudata_cal_available;
    //imu date/time
    struct tm imudata_time;
    bool imudata_time_available;

    //keep track of register requests & values
    unsigned int register_raw_value[REGISTER_ADDRESS_NumRegisters];
    bool register_raw_pending[REGISTER_ADDRESS_NumRegisters];

    //keep track of requested commands
    int pending_command;

    //interprocess_semaphore  data_access_mutex;
    std::mutex _mutex;
};

#endif
