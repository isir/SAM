#include "general_formulation.h"
#include "peripherals/roboclaw/factory.h"
#include "utils/check_ptr.h"

#include "qmath.h"
#include "wiringPi.h"
#include <QDir>
#include <QNetworkDatagram>
#include <iostream>

GeneralFormulation::GeneralFormulation(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("General Formulation")
    , _robot(robot)
    , _Lt(40)
    , _Lua(0.)
    , _Lfa(0.)
    , _l(0.)
    , _lambdaW(0)
    , _lambda(0)
    , _thresholdW(5.)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.wrist_flexion, _robot->sensors.optitrack)) {
        throw std::runtime_error("General Formulation Control is missing components");
    }

    _settings.beginGroup("GeneralFormulation");
    set_period(_settings.value("period", 0.01).toDouble());

    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &GeneralFormulation::receiveData);
    if (!_receiver.bind(QHostAddress::AnyIPv4, 45454)) {
        qCritical() << _receiver.errorString();
    }

    _menu->set_description("GeneralFormulation");
    _menu->set_code("gf");
    _menu->add_item("Tare IMUs", "tare", [this](QString) { this->tare_IMU(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand->menu());

    _pin_up = _settings.value("pin_up", 24).toInt();
    _pin_down = _settings.value("pin_down", 22).toInt();
    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

GeneralFormulation::~GeneralFormulation()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop();
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &GeneralFormulation::receiveData);
}

void GeneralFormulation::tare_IMU()
{
    //    double qFA[4];
    //    _robot->sensors.arm_imu->get_quat(qFA);
    //    qDebug("qarm: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
    //    _robot->sensors.fa_imu->get_quat(qFA);
    //    qDebug("qFA: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);

    _robot->sensors.arm_imu->send_command_algorithm_init_then_tare();
    _robot->sensors.trunk_imu->send_command_algorithm_init_then_tare();

    _robot->sensors.fa_imu->send_command_algorithm_init_then_tare();

    qDebug("Wait ...");

    usleep(6 * 1000000);
    _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::TRIPLE_BUZZ);
    //    _robot->sensors.arm_imu->get_quat(qFA);
    //    qDebug("qarm after tare: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
    //    _robot->sensors.fa_imu->get_quat(qFA);
    //    qDebug("qFA after tare: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
}

void GeneralFormulation::receiveData()
{
    printf("in receiveData \n");
    while (_receiver.hasPendingDatagrams()) {
        QByteArray dataReceived = _receiver.receiveDatagram().data();
        QTextStream ts(&dataReceived);
        int tmp;

        ts >> tmp;
        _Lua = tmp;

        ts >> tmp;
        _Lfa = tmp;

        ts >> tmp;
        _l = tmp;

        ts >> tmp;
        _lambda = tmp;

        ts >> tmp;
        _lambdaW = tmp;

        ts >> tmp;
        _threshold = tmp * M_PI / 180.; // dead zone limit for beta change, in rad.

        ts >> tmp;
        _thresholdW = tmp * M_PI / 180; // dead zone limit for wrist angle change, in rad.
        printf("lambdaW:%d\n", _lambdaW);
    }
}

void GeneralFormulation::displayPin()
{
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);
    qDebug() << "PinUp: " << pin_up_value;
    qDebug() << "PinDown: " << pin_down_value;
}

bool GeneralFormulation::setup()
{
    // Calibrations
    _robot->joints.hand->take_ownership();
    _robot->joints.hand->init_sequence();
    _robot->joints.elbow_flexion->calibrate();
    if (_robot->joints.wrist_flexion) {
        _robot->joints.wrist_flexion->calibrate();
    }
    _robot->joints.wrist_pronation->calibrate();

    _robot->joints.wrist_pronation->set_encoder_position(0);

    // File of recorded data
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &GeneralFormulation::receiveData);

    QString filename = QString("GalF");

    int cnt = 0;
    QString extension = QString(".txt");
    do {
        ++cnt;
        QString suffix = QString("_") + QString::number(cnt);
        _file.setFileName(filename + suffix + extension);
    } while (_file.exists());

    if (!_file.open(QIODevice::ReadWrite)) {
        qCritical() << "Failed to open" << _file.fileName() << "-" << _file.errorString();
        return false;
    }
    _need_to_write_header = true;

    _cnt = 0;
    _time.start();
    return true;
}

void GeneralFormulation::loop(double, double)
{
    int init_cnt = 10;
    QTime t;
    t.start();
    double timeWithDelta = _time.elapsed() / 1000.;
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    double debugData[10];

    /// WRITE FILE HEADERS
    if (_need_to_write_header) {
        _file.write(" time, pinUp, pinDown,");
        _file.write(" qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,");
        _file.write(" qFA.w, qFA.x, qFA.y, qFA.z,");
        _file.write(" to complete");
        _file.write(" nbRigidBodies");
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file.write(", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z");
        }
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    /// CONTROL LOOP

    ///GET DATA
    /// WRIST
    double pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    double wristFlexEncoder = _robot->joints.wrist_flexion->read_encoder_position();
    /// ELBOW
    double elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
    /// IMU
    double qBras[4], qTronc[4], qFA[4];
    _robot->sensors.arm_imu->get_quat(qBras);
    _robot->sensors.trunk_imu->get_quat(qTronc);
    _robot->sensors.fa_imu->get_quat(qFA);
    /// PIN PUSH-BUTTONS CONTROL
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

    QTextStream ts(&_file);
    ts << timeWithDelta << ' ' << pin_down_value << ' ' << pin_up_value;
    ts << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    ts << ' ' << qFA[0] << ' ' << qFA[1] << ' ' << qFA[2] << ' ' << qFA[3];
    /// A COMPLETER !!!!
    ts << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << debugData[3];
    ts << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << pronoSupEncoder << ' ' << wristFlexEncoder << ' ' << elbowEncoder;
    ts << ' ' << data.nRigidBodies;

    for (int i = 0; i < data.nRigidBodies; i++) {
        ts << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        ts << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        ts << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    ts << endl;

    ++_cnt;
    std::cout << QTime::currentTime().toString("mm:ss:zzz").toStdString() << " comp // " << t.elapsed() << "ms" << std::endl;
}

void GeneralFormulation::cleanup()
{
    _robot.elbow->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    _robot->joints.wrist_flexion->forward(0);
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &GeneralFormulation::receiveData);
    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &GeneralFormulation::receiveData);
    _file.close();
}
