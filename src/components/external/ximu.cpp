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

#include "ximu.h"
#include "utils/log/log.h"

#define RXBUFSIZE 256

XIMU::XIMU(const char* filename, int level, int baudrate)
{
    //inits
    fd = -1;
    pending_command = -1;

    set_loglevel(level);

    if (!open_port(filename, baudrate)) {
        _hasOpenPort = false;
        throw std::runtime_error(std::string(filename) + ": " + std::string(strerror(errno)));
    } else {
        _hasOpenPort = true;
    }

    //init
    init_imudata();

    //connect default slots
    init_default_slots();

    //start thread
    boost::thread* thread = new boost::thread(boost::bind(&XIMU::comm_thread, this));
}

XIMU::~XIMU()
{
    if (fd != -1) {
        close(fd);
    }
}

void XIMU::init_imudata()
{
    //init all data
    for (int i = 0; i < 3; i++) {
        imudata_euler[i] = 0.0;
        imudata_quat[i] = 0.0;
        imudata_cal[0][i] = 0.0;
        imudata_cal[1][i] = 0.0;
        imudata_cal[2][i] = 0.0;
    }
    imudata_quat[3] = 0.0;

    imudata_euler_available = false;
    imudata_cal_available = false;
    imudata_time_available = false;
    imudata_quat_available = false;

    for (int i = 0; i < NB_OLD_QUAT; i++)
        for (int j = 0; j < 4; j++)
            _oldQuatValues[i][j] = 0.0;
}

bool XIMU::get_device_detected()
{
    return device_detected;
}

//blocking detection
int XIMU::detect_device()
{
    unsigned int fw_major, fw_minor, device_id;

    if (!get_register(REGISTER_ADDRESS_FirmwareVersionMajorNum, &fw_major)) {
        critical("XIMU ERROR: no reply for firmware major register");
        return 0;
    }

    if (!get_register(REGISTER_ADDRESS_FirmwareVersionMinorNum, &fw_minor)) {
        critical("XIMU ERROR: no reply for firmware minor register");
        return 0;
    }

    if (!get_register(REGISTER_ADDRESS_DeviceID, &device_id)) {
        critical("XIMU ERROR: no reply for device_id register");
        return 0;
    }

    //ok, got minor & major:
    //if ((fw_major != XIMU_SUPPORTED_FIRMWARE_REV_MAJOR) || (fw_minor != XIMU_SUPPORTED_FIRMWARE_REV_MINOR)){
    //printf("XIMU ERROR: unsupported (?) firmware revision %d.%d (expected: %d.%d)\n",fw_major,fw_minor,XIMU_SUPPORTED_FIRMWARE_REV_MAJOR,XIMU_SUPPORTED_FIRMWARE_REV_MINOR);
    //return 0;
    //}

    //sucess!
    debug() << "### XIMU :found XIMU with firmware " << fw_major << "." << fw_minor << " [deviceid 0x" << std::hex << device_id << "]";
    device_detected = true;
    return 1;
}

int XIMU::set_loglevel(int i)
{
    loglevel = i;
    return 0;
}

void XIMU::comm_thread()
{
    rx_packet_buffer_pos = 0;
    int rxcount;
    unsigned char rxbuf[RXBUFSIZE];

    //printf("communication thread running.\n");

    while (1) {
        rxcount = read(fd, rxbuf, 1);

        for (int i = 0; i < rxcount; i++) {
            if (rx_packet_buffer_pos >= RX_PACKET_BUFFER_SIZE) {
                critical("XIMU ERROR: buffer overflow");
            } else {
                //copy to packetbuffer
                rx_packet_buffer[rx_packet_buffer_pos++] = rxbuf[i];
            }

            if (rxbuf[i] & 0x80) {
                //we have a full packet in packet buffer -> process!
                process_packet(rx_packet_buffer, rx_packet_buffer_pos);
                //reset packet counter
                rx_packet_buffer_pos = 0;
            }
        }
    }
}

bool XIMU::get_euler(double* e)
{
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    bool result = imudata_euler_available;

    //access data
    e[0] = imudata_euler[0];
    e[1] = imudata_euler[1];
    e[2] = imudata_euler[2];
    imudata_euler_available = false;
    scoped_lock.unlock();

    return result;
}

bool XIMU::get_quat(double* q)
{
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    bool result = imudata_quat_available;

    //access data
    q[0] = imudata_quat[0];
    q[1] = imudata_quat[1];
    q[2] = imudata_quat[2];
    q[3] = imudata_quat[3];
    imudata_quat_available = false;
    scoped_lock.unlock();

    return result;
}

bool XIMU::get_cal(double* gyro, double* accel, double* mag)
{
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    bool result = imudata_cal_available;

    //access data
    for (int i = 0; i < 3; i++) {
        gyro[i] = imudata_cal[0][i];
        accel[i] = imudata_cal[1][i];
        mag[i] = imudata_cal[2][i];
    }
    imudata_cal_available = false;
    scoped_lock.unlock();

    return result;
}

bool XIMU::get_time(struct tm* time)
{
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    bool result = imudata_time_available;

    //access data
    *time = imudata_time;
    imudata_time_available = false;
    scoped_lock.unlock();

    return result;
}

bool XIMU::open_port(const char* fn, int baudrate)
{
    fd = open(fn, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        return false;
    } else {
        //set serial speed
        struct termios termOptions;
        //get the current options:
        tcgetattr(fd, &termOptions);

        //baudrate
        cfsetispeed(&termOptions, baudrate);
        cfsetospeed(&termOptions, baudrate);

        ///// Input flags - Turn off input processing
        // convert break to null byte, no CR to NL translation,
        // no NL to CR translation, don't mark parity errors or breaks
        // no input parity check, don't strip high bit off,
        // no XON/XOFF software flow control
        //
        termOptions.c_iflag = IGNPAR;
        //
        // Output flags - Turn off output processing
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping,
        // no local output processing
        //
        // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
        //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
        termOptions.c_oflag = 0;
        //
        // No line processing:
        // echo off, echo newline off, canonical mode off,
        // extended input processing off, signal chars off
        //
        termOptions.c_lflag &= 0;
        //
        // Turn off character processing
        // clear current char size mask, no parity checking,
        // no output processing, force 8 bit input
        //
        termOptions.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

        //
        // One input byte is enough to return from read()
        // Inter-character timer off

        // Now set the term options (set immediately)
        tcsetattr(fd, TCSANOW, &termOptions);
        //flush io
        tcflush(fd, TCIOFLUSH);
        usleep(100000);
        tcflush(fd, TCIOFLUSH);
        usleep(100000);
    }
    return true;
}

void XIMU::process_packet(unsigned char* packet_ptr, int len)
{
    //decode packet checks
    if ((len < 4) || (len > 30)) {
        debug() << "### XIMU :packet len invalid (" << len << " != [4...30])";
    }

    //decode data (decoded data is stored in packet ptr & updated len
    len = decode_packet(packet_ptr, len);

    unsigned char checksum = 0x00;

    //build checksum (sum of elements)
    for (int i = 0; i < (len - 1); i++) {
        checksum += packet_ptr[i];
    }

    if (checksum != packet_ptr[len - 1]) {
        debug("### XIMU :checksum mismatch");
        return;
    }

    //process packet:
    switch (packet_ptr[0]) {
    case (PACKET_HEADER_ERROR):
        return process_packet_error_data(packet_ptr, len);
    case (PACKET_HEADER_COMMAND):
        return process_packet_command_data(packet_ptr, len);
    case (PACKET_HEADER_WRITEREGISTER):
        return process_packet_write_register(packet_ptr, len);
    case (PACKET_HEADER_WRITEDATETIME):
        return process_packet_write_datetime(packet_ptr, len);
    case (PACKET_HEADER_RAWBATTERYANDTHERMOMETERDATA):
        return process_packet_raw_battery_and_thermometer_data(packet_ptr, len);
    case (PACKET_HEADER_CALBATTERYANDTHERMOMETERDATA):
        return process_packet_cal_battery_and_thermometer_data(packet_ptr, len);
    case (PACKET_HEADER_RAWINERTIALANDMAGNETICDATA):
        return process_packet_raw_inertialandmagnetic_data(packet_ptr, len);
    case (PACKET_HEADER_CALINERTIALANDMAGNETICDATA):
        return process_packet_cal_inertialandmagnetic_data(packet_ptr, len);
    case (PACKET_HEADER_QUATERNIONDATA):
        return process_packet_quaternion_data(packet_ptr, len);
    case (PACKET_HEADER_DIGITALIODATA):
        return process_digital_io_data(packet_ptr, len);
    case (PACKET_HEADER_RAWANALOGUEINPUTDATA):
        return process_raw_analog_data(packet_ptr, len);
    case (PACKET_HEADER_CALANALOGUEINPUTDATA):
        return process_cal_analog_data(packet_ptr, len);
    case (PACKET_HEADER_PWMOUTPUTDATA):
        return process_pwm_data(packet_ptr, len);
    case (PACKET_HEADER_RAWADXL345BUSDATA):
        return process_raw_adxl_bus_data(packet_ptr, len);
    case (PACKET_HEADER_CALADXL345BUSDATA):
        return process_cal_adxl_bus_data(packet_ptr, len);

    //NOT DEFINED IN API?! i think this packet type will never occur
    case (PACKET_HEADER_READDATETIME):
        return;
    case (PACKET_HEADER_READREGISTER):
        return;

    default:
        debug() << "### XIMU :ERROR: invalid packet header type 0x" << std::hex << packet_ptr[0];
        return;
    }
}

//blocking register read
int XIMU::get_register(unsigned int register_address, unsigned int* val)
{
    bool waiting_for_reply;
    unsigned int timeout;

    if (register_address >= REGISTER_ADDRESS_NumRegisters) {
        debug() << "### XIMU :ERROR: register address " << register_address << " too large (>=" << REGISTER_ADDRESS_NumRegisters << " -> ignored)";
        return 0;
    }

    boost::mutex::scoped_lock scoped_lock(data_access_mutex);
    register_raw_pending[register_address] = true;
    scoped_lock.unlock();

    //try n times to get a reply:
    for (int i = 0; i < XIMU_READ_REGISTER_TRIES; i++) {
        //send request
        send_register_read_request(register_address);

        //wait up to 100ms for a reply
        timeout = 1000;
        while (timeout) {
            //test if we got the result
            scoped_lock.lock();
            waiting_for_reply = register_raw_pending[register_address];
            scoped_lock.unlock();

            if (!waiting_for_reply) {
                //we received a reply -> process & return
                debug() << "### XIMU :GOT REGISTER REPLY within " << (i * 100 + (1000 - timeout) / 10.0) << "ms";
                *val = register_raw_value[register_address];
                return 1;
            }
            usleep(100);
            timeout--;
        }

        debug() << "### XIMU :TIMEOUT: 100ms timeout during get register, retrying " << (XIMU_READ_REGISTER_TRIES - i) << " more times";
    }

    //last chance:
    scoped_lock.lock();
    waiting_for_reply = register_raw_pending[register_address];
    scoped_lock.unlock();

    if (waiting_for_reply) {
        debug() << "### XIMU :ERROR: request for register read " << register_address << " timed out";
        return 0;
    }

    //else: success
    *val = register_raw_value[register_address];
    return 1;
}

bool XIMU::send_command(unsigned int command_code)
{
    unsigned char packet[4];
    int count;

    //build packet
    packet[0] = PACKET_HEADER_COMMAND;
    packet[1] = (command_code >> 8) & 0xFF;
    packet[2] = (command_code)&0xFF;

    //calc checksum
    packet[3] = packet[0] + packet[1] + packet[2];

    //track command execution:
    pending_command = command_code;

    //how many retries?
    unsigned int retries = 10;
    while ((pending_command == (int)command_code) && (retries > 0)) {
        //transmit:
        //printf("sending packet 0x%02X 0x%02X 0x%02X 0x%02X\n",packet[0],packet[1],packet[2],packet[3]);
        send_packet(packet, 4);

        //wait for command to be finished
        //printf("waiting for command to finish\n");

        //timeout: 200ms
        for (count = 200 / 10; count > 0; count--) {
            //finished execution?
            if (pending_command != (int)command_code) {
                break;
            }

            //sleep 10ms
            usleep(10 * 1000);

            //debug
            //printf("waiting (10ms)\n");
        }

        //abort after n failures
        retries--;

        //timed out?
        if (count == 0) {
            //printf("command timed out! %d retries left\n",retries);
        } else {
            //printf("command finished\n");
        }
    }

    //success?
    if (retries != 0) {
        return true;
    } else {
        return false;
    }
}

bool XIMU::send_command_algorithm_init()
{
    return send_command(COMMAND_CODE_AlgorithmInitialise);
}

bool XIMU::send_command_algorithm_init_then_tare()
{
    return send_command(COMMAND_CODE_AlgorithmInitialiseThenTare);
}

bool XIMU::send_command_algorithm_tare()
{
    return send_command(COMMAND_CODE_AlgorithmTare);
}

bool XIMU::send_command_algorithm_clear_tare()
{
    return send_command(COMMAND_CODE_AlgorithmClearTare);
}

void XIMU::send_register_read_request(unsigned int register_address)
{
    unsigned char packet[4];

    //build packet
    packet[0] = PACKET_HEADER_READREGISTER;
    packet[1] = (register_address >> 8) & 0xFF;
    packet[2] = (register_address)&0xFF;

    //calc checksum
    packet[3] = packet[0] + packet[1] + packet[2];

    //transmit:
    //printf("sending packet 0x%02X 0x%02X 0x%02X 0x%02X\n",packet[0],packet[1],packet[2],packet[3]);
    send_packet(packet, 4);
}

void XIMU::send_packet(unsigned char* buf, unsigned int len)
{
    //encode packet data before sending
    int encoded_len = (int)(ceil(((float)len * 1.125f) + 0.125f));
    // CHANGED unsigned char shift_reg[encoded_len];
    // CHANGED unsigned char encoded_buf[encoded_len];
    unsigned char shift_reg[100];
    unsigned char encoded_buf[100];

    //copy data to shift_reg
    for (int i = 0; i < encoded_len; i++) {
        if (i < (int)len) {
            shift_reg[i] = buf[i];
        } else {
            shift_reg[i] = 0;
        }
        encoded_buf[i] = 0;
    }

    //encode
    for (int i = 0; i < encoded_len; i++) {
        right_shift(shift_reg, encoded_len); //shift
        encoded_buf[i] = shift_reg[i]; //copy byte i
        shift_reg[i] = 0; //clear byte i
    }

    //set msb of framing byte
    encoded_buf[encoded_len - 1] |= 0x80;

    write(fd, encoded_buf, encoded_len);
    //printf("wrote %d bytes [%d->%d]\n",res,len,encoded_len);
}

//decode packet data
//->decodes a packet with consecutive left shifts so that the msb of each encoded byte is removed.
//return processed values in place of old packet ptr
int XIMU::decode_packet(unsigned char* packet, int len)
{
    int unshiftet_len = (int)(floor(((float)len - 0.125f) / 1.125f));
    // CHANGED unsigned char buf[len];
    unsigned char buf[100];

    for (int i = len - 1; i >= 0; i--) {
        buf[i] = packet[i];
        left_shift(buf, len);
    }

    for (int i = 0; i < unshiftet_len; i++) {
        packet[i] = buf[i];
    }
    return unshiftet_len;
}

void XIMU::left_shift(unsigned char* packet, int len)
{
    packet[0] = packet[0] << 1;
    for (int i = 1; i < len; i++) {
        if (packet[i] & 0x80) {
            packet[i - 1] |= 0x01;
        }
        packet[i] = packet[i] << 1;
    }
}

void XIMU::right_shift(unsigned char* packet, int len)
{
    packet[len - 1] = packet[len - 1] >> 1;
    for (int i = len - 2; i >= 0; i--) {
        if (packet[i] & 0x01) {
            packet[i + 1] |= 0x80;
        }
        packet[i] = packet[i] >> 1;
    }
}

bool XIMU::check_len(int len, int check)
{
    if (len != check) {
        //printf("invalid len %d (!=%d)\n",len,check);
        return false;
    }
    return true;
}

void XIMU::process_packet_error_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 4))
        return;

    unsigned int errorcode = (ptr[1] << 8) | ptr[2];
    std::string error = "";
    switch (errorcode) {
    case (ERROR_CODE_NoError):
        error = "No error.";
        break;
    case (ERROR_CODE_FactoryResetFailed):
        error = "Factory reset failed.";
        break;
    case (ERROR_CODE_LowBattery):
        error = "Low battery.";
        break;
    case (ERROR_CODE_USBreceiveBufferOverrun):
        error = "USB receive buffer overrun.";
        break;
    case (ERROR_CODE_USBtransmitBufferOverrun):
        error = "USB transmit buffer overrun.";
        break;
    case (ERROR_CODE_BluetoothReceiveBufferOverrun):
        error = "Bluetooth receive buffer overrun.";
        break;
    case (ERROR_CODE_BluetoothTransmitBufferOverrun):
        error = "Bluetooth transmit buffer overrun.";
        break;
    case (ERROR_CODE_SDcardWriteBufferOverrun):
        error = "SD card write buffer overrun.";
        break;
    case (ERROR_CODE_TooFewBytesInPacket):
        error = "Too few bytes in packet.";
        break;
    case (ERROR_CODE_TooManyBytesInPacket):
        error = "Too many bytes in packet.";
        break;
    case (ERROR_CODE_InvalidChecksum):
        error = "Invalid checksum.";
        break;
    case (ERROR_CODE_UnknownHeader):
        error = "Unknown packet header.";
        break;
    case (ERROR_CODE_InvalidNumBytesForPacketHeader):
        error = "Invalid number of bytes for packet header.";
        break;
    case (ERROR_CODE_InvalidRegisterAddress):
        error = "Invalid register address.";
        break;
    case (ERROR_CODE_RegisterReadOnly):
        error = "Cannot write to read-only register.";
        break;
    case (ERROR_CODE_InvalidRegisterValue):
        error = "Invalid register value.";
        break;
    case (ERROR_CODE_InvalidCommand):
        error = "Invalid command.";
        break;
    case (ERROR_CODE_GyroscopeAxisNotAt200dps):
        error = "Gyroscope axis not at 200 ˚/s. Operation aborted.";
        break;
    case (ERROR_CODE_GyroscopeNotStationary):
        error = "Gyroscope not stationary. Operation aborted.";
        break;
    case (ERROR_CODE_AcceleroemterAxisNotAt1g):
        error = "Acceleroemter axis not at ±1 g. Operation aborted.";
        break;
    case (ERROR_CODE_MagnetometerSaturation):
        error = "Magnetometer saturation occurred. Operation aborted.";
        break;
    case (ERROR_CODE_IncorrectAuxillaryPortMode):
        error = "Auxiliary port in incorrect mode.";
        break;
    case (ERROR_CODE_UARTreceiveBufferOverrun):
        error = "UART receive buffer overrun.";
        break;
    case (ERROR_CODE_UARTtransmitBufferOverrun):
        error = "UART transmit buffer overrun.";
        break;
    default:
        error = "unknown errorcode";
        break;
    }
    debug() << "### XIMU: ERROR: " << error;
}

void XIMU::process_packet_command_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 4))
        return;

    int code = (ptr[1] << 8) | ptr[2];
    std::string command = "";
    switch (code) {
    case (COMMAND_CODE_NullCommand):
        command = "Null command.";
        break;
    case (COMMAND_CODE_FactoryReset):
        command = "Factory reset.";
        break;
    case (COMMAND_CODE_Reset):
        command = "Reset.";
        break;
    case (COMMAND_CODE_Sleep):
        command = "Sleep.";
        break;
    case (COMMAND_CODE_ResetSleepTimer):
        command = "Reset sleep timer.";
        break;
    case (COMMAND_CODE_SampleGyroscopeAxisAt200dps):
        command = "Sample gyroscope axis at ±200 dps.";
        break;
    case (COMMAND_CODE_CalculateGyroscopeSensitivity):
        command = "Calculate gyroscope sensitivity.";
        break;
    case (COMMAND_CODE_SampleGyroscopeBiasTemp1):
        command = "Sample gyroscope bias at temperature 1.";
        break;
    case (COMMAND_CODE_SampleGyroscopeBiasTemp2):
        command = "Sample gyroscope bias at temperature 2.";
        break;
    case (COMMAND_CODE_CalculateGyroscopeBiasParameters):
        command = "Calculate gyroscope bias parameters.";
        break;
    case (COMMAND_CODE_SampleAccelerometerAxisAt1g):
        command = "Sample accelerometer axis at ±1 g.";
        break;
    case (COMMAND_CODE_CalculateAccelerometerBiasAndSensitivity):
        command = "Calculate accelerometer bias and sensitivity";
        break;
    case (COMMAND_CODE_MeasureMagnetometerBiasAndSensitivity):
        command = "Measure magnetometer bias and sensitivity.";
        break;
    case (COMMAND_CODE_AlgorithmInitialise):
        command = "Algorithm initialise.";
        break;
    case (COMMAND_CODE_AlgorithmTare):
        command = "Algorithm tare.";
        break;
    case (COMMAND_CODE_AlgorithmClearTare):
        command = "Algorithm clear tare.";
        break;
    case (COMMAND_CODE_AlgorithmInitialiseThenTare):
        command = "Algorithm initialise then tare.";
        break;
    default:
        command = "unknown command";
        break;
    }

    //pending request?
    if (pending_command == code) {
        pending_command = -1;
    }

    debug() << "### XIMU :COMMANDCODE 0x" << std::hex << code << " = " << command;
}

void XIMU::process_packet_write_register(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 6))
        return;

    int hi = (ptr[1] << 8) | ptr[2];
    int lo = (ptr[3] << 8) | ptr[4];

    //value too big -> ignore
    if (hi >= REGISTER_ADDRESS_NumRegisters) {
        debug() << "### XIMU :ERROR: reply for register address 0x" << std::hex << hi << " too large (>=0x" << REGISTER_ADDRESS_NumRegisters << " -> ignored)";
        return;
    }

    //store internally:
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    register_raw_value[hi] = lo; //value
    register_raw_pending[hi] = false; //no longer pending as we received a reply
    scoped_lock.unlock();

    info() << "XIMU INFO : WRITE REGISTER 0x" << std::hex << hi << lo;
}

void XIMU::process_packet_write_datetime(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 8))
        return;

    int year = (int)(10 * ((ptr[1] & 0xF0) >> 4) + (ptr[1] & 0x0F)) + 2000;
    int month = (int)(10 * ((ptr[2] & 0xF0) >> 4) + (ptr[2] & 0x0F));
    int day = (int)(10 * ((ptr[3] & 0xF0) >> 4) + (ptr[3] & 0x0F));
    int hour = (int)(10 * ((ptr[4] & 0xF0) >> 4) + (ptr[4] & 0x0F));
    int minute = (int)(10 * ((ptr[5] & 0xF0) >> 4) + (ptr[5] & 0x0F));
    int second = (int)(10 * ((ptr[6] & 0xF0) >> 4) + (ptr[6] & 0x0F));

    // DATAGET printf("WRITE DATETIME: %4d.%02d.%02d %02d:%02d.%02d\n",year,month,day,hour,minute,second);

    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    imudata_time.tm_year = year - 1900;
    imudata_time.tm_mon = month - 1;
    imudata_time.tm_mday = day;
    imudata_time.tm_hour = hour;
    imudata_time.tm_min = minute;
    imudata_time.tm_sec = second;
    //set time (day etc)
    mktime(&imudata_time);
    scoped_lock.unlock();

    signal_incoming_data_time(imudata_time);
}

void XIMU::process_packet_raw_battery_and_thermometer_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 6))
        return;

    int d0 = (ptr[1] << 8) | ptr[2];
    int d1 = (ptr[3] << 8) | ptr[4];

    printf("XIMU : RAW THERMOMETER & BATTERY DATA 0x%04X 0x%04X [FIXME: add implementation]\n", d0, d1);
}

void XIMU::process_packet_cal_battery_and_thermometer_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 6))
        return;

    //to floating point:
    float bat = to_float(ptr[1], ptr[2], 12); //12
    float therm = to_float(ptr[3], ptr[4], 8); //8

    printf("XIMU : CAL THERMOMETER & BATTERY DATA -> %4.2fV    %5.2f°C\n", bat, therm);
}

void XIMU::process_packet_raw_inertialandmagnetic_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 20))
        return;

    uint16_t raw[3][3];

    //asign data, should be [1,2 3,4 5,6] [7,8 9,10 11,12] [13,14 15,16 17,18]
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            raw[j][i] = (ptr[1 + 6 * j + 2 * i] << 8) | ptr[2 + 6 * j + 2 * i];
        }
    }

    printf("XIMU : RAW DATA: gyro[0x%08X, 0x%08X, 0x%08X] accel[0x%08X, 0x%08X, 0x%08X] mag[0x%08X, 0x%08X, 0x%08X]\n",
        raw[0][0], raw[0][1], raw[0][2], raw[1][0], raw[1][1], raw[1][2], raw[2][0], raw[2][1], raw[2][2]);
    //fire signal
    signal_incoming_data_raw(raw[0], raw[1], raw[2]);
}

void XIMU::process_packet_cal_inertialandmagnetic_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 20))
        return;

    double gyro[3];
    double accel[3];
    double mag[3];
    for (int i = 0; i < 3; i++) {
        gyro[i] = to_float(ptr[1 + 2 * i], ptr[2 + 2 * i], 4);
        accel[i] = to_float(ptr[7 + 2 * i], ptr[8 + 2 * i], 11);
        mag[i] = to_float(ptr[13 + 2 * i], ptr[14 + 2 * i], 11);
    }

    // DATAGET printf("CAL DATA: gyro[%8.6f, %8.6f, %8.6f] accel[%8.6f, %8.6f, %8.6f] mag[%8.6f, %8.6f, %8.6f]\n",gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],mag[0],mag[1],mag[2]);

    //store data
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    for (int i = 0; i < 3; i++) {
        imudata_cal[0][i] = gyro[i];
        imudata_cal[1][i] = accel[i];
        imudata_cal[2][i] = mag[i];
    }
    imudata_cal_available = true;
    scoped_lock.unlock();

    //fire signal
    signal_incoming_data_cal(imudata_cal[0], imudata_cal[1], imudata_cal[2]);
}

void XIMU::process_packet_quaternion_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 10))
        return;

    float quat[4];
    for (int i = 0; i < 4; i++) {
        quat[i] = to_float(ptr[1 + 2 * i], ptr[2 + 2 * i], 15);
    }

    //normlize
    float norm = (float)sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;

    float phi = (float)atan2(2 * (quat[2] * quat[3] - quat[0] * quat[1]), 2 * quat[0] * quat[0] - 1 + 2 * quat[3] * quat[3]);
    float theta = (float)-atan((2.0 * (quat[1] * quat[3] + quat[0] * quat[2])) / sqrt(1.0 - pow((2.0 * quat[1] * quat[3] + 2.0 * quat[0] * quat[2]), 2.0)));
    float psi = (float)atan2(2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * quat[0] * quat[0] - 1 + 2 * quat[1] * quat[1]);

    phi *= 180.0 / M_PI;
    theta *= 180.0 / M_PI;
    psi *= 180.0 / M_PI;

    // DATAGET printf("QUATERNION DATA: [%8.6f, %8.6f, %8.6f, %8.6f] --> Euler %8.5f, %8.5f, %8.5f\n",quat[0],quat[1],quat[2],quat[3],phi,theta,psi);

    //store data
    boost::mutex::scoped_lock scoped_lock(data_access_mutex); //will be freed on function exit
    imudata_euler[0] = phi;
    imudata_euler[1] = theta;
    imudata_euler[2] = psi;
    imudata_euler_available = true;

    imudata_quat[0] = quat[0];
    imudata_quat[1] = quat[1];
    imudata_quat[2] = quat[2];
    imudata_quat[3] = quat[3];
    imudata_quat_available = true;
    scoped_lock.unlock();

    //fire signal
    signal_incoming_data_euler(imudata_euler);
    signal_incoming_data_quat(imudata_quat);
}

void XIMU::process_digital_io_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 4))
        return;

    int d0 = (ptr[1] << 8) | ptr[2];

    // DATAGET printf("RAW DIGITAL IO DATA 0x%04X [FIXME: add implementation]\n",d0);
}

void XIMU::process_raw_analog_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 18))
        return;

    uint16_t raw[8];
    for (int i = 0; i < 8; i++) {
        raw[i] = ((ptr[1 + 2 * i] << 8) | ptr[2 + 2 * i]);
    }

    if (loglevel >= XIMU_LOGLEVEL_DEBUG) {
        printf("XIMU : RAW ANALOG DATA: [");
        for (int i = 0; i < 8; i++) {
            printf("0x%04X, ", raw[i]);
        }
        printf("] [FIXME: add implementation]\n");
    }
}

void XIMU::process_cal_analog_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 18))
        return;

    float raw[8];
    for (int i = 0; i < 8; i++) {
        raw[i] = to_float(ptr[1 + 2 * i], ptr[2 + 2 * i], 12);
    }

    if (loglevel >= XIMU_LOGLEVEL_DEBUG) {
        printf("XIMU : CAL ANALOG DATA: [");
        for (int i = 0; i < 8; i++) {
            printf("%8.5f, ", raw[i]);
        }
        printf("] [FIXME: add implementation]\n");
    }
}

void XIMU::process_pwm_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 10))
        return;

    uint16_t raw[4];
    for (int i = 0; i < 4; i++) {
        raw[i] = ((ptr[1 + 2 * i] << 8) | ptr[2 + 2 * i]);
    }

    if (loglevel >= XIMU_LOGLEVEL_DEBUG) {
        printf("XIMU : RAW PWM DATA: [");
        for (int i = 0; i < 4; i++) {
            printf("0x%04X, ", raw[i]);
        }
        printf("] [FIXME: add implementation]\n");
    }
}

void XIMU::process_raw_adxl_bus_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 26))
        return;

    uint16_t raw[4][3];

    //asign data, should be [1,2 3,4 5,6] [7,8 9,10 11,12] [13,14 15,16 17,18]
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 3; i++) {
            raw[j][i] = (ptr[1 + 6 * j + 2 * i] << 8) | ptr[2 + 6 * j + 2 * i];
        }
    }

    if (loglevel >= XIMU_LOGLEVEL_DEBUG) {
        printf("XIMU : RAW ADXL BUS DATA: ");
        for (int j = 0; j < 4; j++) {
            printf("[%d][", j);
            for (int i = 0; i < 3; i++) {
                printf("0x%04X, ", raw[j][i]);
            }
            printf("] ");
        }
        printf(" [FIXME: add implementation]\n");
    }
}

void XIMU::process_cal_adxl_bus_data(unsigned char* ptr, int len)
{
    //check packet len
    if (!check_len(len, 26))
        return;

    float raw[4][3];
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 3; i++) {
            raw[j][i] = to_float(ptr[1 + 6 * j + 2 * i], ptr[2 + 6 * j + 2 * i], 10);
        }
    }

    if (loglevel >= XIMU_LOGLEVEL_DEBUG) {
        printf("XIMU : CAL ADXL BUS DATA: [");
        for (int j = 0; j < 4; j++) {
            printf("[%d][", j);
            for (int i = 0; i < 3; i++) {
                printf("%8.5f, ", raw[j][i]);
            }
            printf("] ");
        }
        printf(" [FIXME: add implementation]\n");
    }
}

///slot handling
void XIMU::init_default_slots()
{
    connect_slot_incoming_data_euler(default_slot_incoming_data_euler);
    connect_slot_incoming_data_quat(default_slot_incoming_data_quat);
    connect_slot_incoming_data_raw(default_slot_incoming_data_raw);
    connect_slot_incoming_data_cal(default_slot_incoming_data_cal);
    connect_slot_incoming_data_time(default_slot_incoming_data_time);
}

void XIMU::default_slot_incoming_data_euler(double* euler)
{
#if XIMU_DEBUG_DEFAULT_SLOTS
    printf("XIMU : DEFAULT SLOT: incoming euler angles [%6.2f %6.2f %6.2f]\n", euler[0], euler[1], euler[2]);
#endif
}

void XIMU::default_slot_incoming_data_quat(double* quat)
{
#if XIMU_DEBUG_DEFAULT_SLOTS
    printf("XIMU : DEFAULT SLOT: incoming quat [%6.2f %6.2f %6.2f %6.2f]\n", quat[0], quat[1], quat[2], quat[3]);
#endif
}

void XIMU::default_slot_incoming_data_raw(uint16_t* gyro, uint16_t* accel, uint16_t* mag)
{
#if XIMU_DEBUG_DEFAULT_SLOTS
    printf("XIMU : DEFAULT SLOT: incoming RAW  gyro[0x%04X 0x%04X 0x%04X] accel[0x%04X 0x%04X 0x%04X] mag[0x%04X 0x%04X 0x%04X]\n",
        gyro[0], gyro[1], gyro[2],
        accel[0], accel[1], accel[2],
        mag[0], mag[1], mag[2]);
#endif
}

void XIMU::default_slot_incoming_data_cal(double* gyro, double* accel, double* mag)
{
#if XIMU_DEBUG_DEFAULT_SLOTS
    printf("XIMU : DEFAULT SLOT: incoming CAL gyro[%6.2f %6.2f %6.2f] accel[%6.2f %6.2f %6.2f] mag[%6.2f %6.2f %6.2f]\n",
        gyro[0], gyro[1], gyro[2],
        accel[0], accel[1], accel[2],
        mag[0], mag[1], mag[2]);
#endif
}

void XIMU::default_slot_incoming_data_time(struct tm now)
{
#if XIMU_DEBUG_DEFAULT_SLOTS
    char buf[80];
    strftime(buf, 80, "%c.", &now);
    printf("XIMU : DEFAULT SLOT: incoming TIME %s\n", buf);
#endif
}

void XIMU::connect_slot_incoming_data_euler(const signal_incoming_data_euler_t::slot_type& slot)
{
    signal_incoming_data_euler.disconnect_all_slots();
    signal_incoming_data_euler.connect(slot);
}

void XIMU::connect_slot_incoming_data_quat(const signal_incoming_data_quat_t::slot_type& slot)
{
    signal_incoming_data_quat.disconnect_all_slots();
    signal_incoming_data_quat.connect(slot);
}

void XIMU::connect_slot_incoming_data_raw(const signal_incoming_data_raw_t::slot_type& slot)
{
    signal_incoming_data_raw.disconnect_all_slots();
    signal_incoming_data_raw.connect(slot);
}

void XIMU::connect_slot_incoming_data_cal(const signal_incoming_data_cal_t::slot_type& slot)
{
    signal_incoming_data_cal.disconnect_all_slots();
    signal_incoming_data_cal.connect(slot);
}

void XIMU::connect_slot_incoming_data_time(const signal_incoming_data_time_t::slot_type& slot)
{
    signal_incoming_data_time.disconnect_all_slots();
    signal_incoming_data_time.connect(slot);
}

//disconnect slots & assign default handler (for debugging)
void XIMU::disconnect_slot_incoming_data_euler()
{
    signal_incoming_data_euler.disconnect_all_slots();
    connect_slot_incoming_data_euler(default_slot_incoming_data_euler);
}

void XIMU::disconnect_slot_incoming_data_quat()
{
    signal_incoming_data_quat.disconnect_all_slots();
    connect_slot_incoming_data_quat(default_slot_incoming_data_quat);
}

void XIMU::disconnect_slot_incoming_data_raw()
{
    signal_incoming_data_raw.disconnect_all_slots();
    connect_slot_incoming_data_raw(default_slot_incoming_data_raw);
}

void XIMU::disconnect_slot_incoming_data_cal()
{
    signal_incoming_data_cal.disconnect_all_slots();
    connect_slot_incoming_data_cal(default_slot_incoming_data_cal);
}

void XIMU::disconnect_slot_incoming_data_time()
{
    signal_incoming_data_time.disconnect_all_slots();
    connect_slot_incoming_data_time(default_slot_incoming_data_time);
}

bool XIMU::areQuatConsistent(double currentQuatW, double currentQuatX, double currentQuatY, double currentQuatZ)
{
    // Update values
    for (int i = NB_OLD_QUAT - 1; i > 0; i--) {
        for (int j = 0; j < 4; j++)
            _oldQuatValues[i][j] = _oldQuatValues[i - 1][j];
    }

    _oldQuatValues[0][0] = currentQuatW;
    _oldQuatValues[0][1] = currentQuatX;
    _oldQuatValues[0][2] = currentQuatY;
    _oldQuatValues[0][3] = currentQuatZ;

    bool areAllQuatEqual = true;
    for (int i = 0; i < NB_OLD_QUAT - 1; i++) {
        for (int j = 0; j < 4; j++)
            if (_oldQuatValues[i][j] != _oldQuatValues[i + 1][j]) {
                areAllQuatEqual = false;
                break;
            }
    }
    return !areAllQuatEqual;
}
