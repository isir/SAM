#include "compensation_imu.h"
#include "peripherals/roboclaw/factory.h"

#include "control/algorithms/lawimu.h"
#include "qmath.h"
#include "wiringPi.h"
#include <QDir>
#include <QNetworkDatagram>
#include <iostream>

CompensationIMU::CompensationIMU(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt)
    : BasicController(mqtt)
    , _robot(robot)
    , _Lt(40)
    , _Lua(0.)
    , _Lfa(0.)
    , _l(0.)
    , _lambdaW(0)
    , _lambda(0)
    , _thresholdW(5.)
{
    _settings.beginGroup("CompensationIMU");
    set_period(_settings.value("period", 0.01).toDouble());

    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &CompensationIMU::receiveData);
    if (!_receiver.bind(QHostAddress::AnyIPv4, 45454)) {
        qCritical() << _receiver.errorString();
    }

    QObject::connect(&_receiverArduino, &QUdpSocket::readyRead, this, &CompensationIMU::listenArduino);
    if (!_receiverArduino.bind(QHostAddress::AnyIPv4, 45455)) {
        qCritical() << "Arduino receiver" << _receiverArduino.errorString();
    }

    _menu.set_title("CompensationIMU");
    _menu.set_code("imu");
    _menu.addItem(ConsoleMenuItem("Tare IMUs", "tare", [this](QString) { this->tare_IMU(); }));
    if (_robot.wrist_pronosup) {
        _menu.addItem(_robot.wrist_pronosup->menu());
    }
    if (_robot.hand) {
        _menu.addItem(_robot.hand->menu());
    }
}

CompensationIMU::~CompensationIMU()
{
    //    if (_robot.elbow) {
    //        _robot.elbow->forward(0);
    //    }
    if (_robot.wrist_pronosup) {
        _robot.wrist_pronosup->forward(0);
    }

    stop();
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &CompensationIMU::receiveData);
}

void CompensationIMU::tare_IMU()
{
    _robot.arm_imu->send_command_algorithm_init_then_tare();
    _robot.trunk_imu->send_command_algorithm_init_then_tare();

    _robot.fa_imu->send_command_algorithm_init_then_tare();

    double qFA[4];
    _robot.fa_imu->get_quat(qFA);
    qDebug("qFA: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
    qDebug("Wait for triple bip");

    usleep(6 * 1000000);
    _robot.buzzer->makeNoise(BuzzerConfig::TRIPLE_BUZZ);
}

void CompensationIMU::receiveData()
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

void CompensationIMU::listenArduino()
{
    while (_receiverArduino.hasPendingDatagrams()) {
        QByteArray dataReceivedArduino = _receiverArduino.receiveDatagram().data();
        QTextStream tsA(&dataReceivedArduino);
        int tmp;

        tsA >> tmp;
        _pinArduino = tmp;
        qDebug("pinArduino:%d", _pinArduino);
    }
}

bool CompensationIMU::setup()
{
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &CompensationIMU::receiveData);
    QObject::disconnect(&_receiverArduino, &QUdpSocket::readyRead, this, &CompensationIMU::listenArduino);
    QObject::connect(&_receiverArduino, &QUdpSocket::readyRead, this, &CompensationIMU::listenArduino);
    //    if (_robot.elbow) {
    //        _robot.elbow->calibrate();
    //    }
    if (_robot.wrist_pronosup) {
        _robot.wrist_pronosup->set_encoder_position(0);
    }

    QString filename = QString("compensationIMU");

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

void CompensationIMU::loop(double, double)
{
    int init_cnt = 10;
    QTime t;
    t.start();
    double timeWithDelta = _time.elapsed() / 1000.;
    optitrack_data_t data = _robot.optitrack->get_last_data();
    double debugData[10];

    if (_need_to_write_header) {
        _file.write(" time, pinArduino,");
        _file.write(" qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,");
        _file.write(" qFA.w, qFA.x, qFA.y, qFA.z,");
        _file.write(" phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW, wristEncoder,");
        _file.write(" nbRigidBodies");
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file.write(", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z");
        }
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    /// WRIST
    double wristAngleEncoder = _robot.wrist_pronosup->read_encoder_position();

    double qBras[4], qTronc[4], qFA[4];
    _robot.arm_imu->get_quat(qBras);
    _robot.trunk_imu->get_quat(qTronc);
    _robot.fa_imu->get_quat(qFA);
    Eigen::Quaterniond qFA_record;
    qFA_record.w() = qFA[0];
    qFA_record.x() = qFA[1];
    qFA_record.y() = qFA[2];
    qFA_record.z() = qFA[3];

    if (_cnt == 0) {
        _lawimu.initialization();
    } else if (_cnt <= init_cnt) {
        _lawimu.initialPositions(qFA_record, _cnt, init_cnt);
    } else {
        _lawimu.rotationMatrices(qFA_record);
        _lawimu.controlLawWrist(_lambdaW, _thresholdW);

        if (_lawimu.returnWristVel_deg() > 0) {
            _robot.wrist_pronosup->move_to(350, _lawimu.returnWristVel_deg());
        } else if (_lawimu.returnWristVel_deg() < 0) {
            _robot.wrist_pronosup->move_to(-350, -_lawimu.returnWristVel_deg());
        } else if (_lawimu.returnWristVel_deg() == 0) {
            _robot.wrist_pronosup->forward(0);
        }

        if (_cnt % _settings.value("display_count", 50).toInt() == 0) {
            _lawimu.displayData();
            // qDebug("lambdaW: %d", _lambdaW);
            //            printf("lambdaW: %d\n", _lambdaW);
        }
    }

    _lawimu.writeDebugData(debugData);

    QTextStream ts(&_file);
    ts << timeWithDelta << ' ' << _pinArduino;
    ts << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    ts << ' ' << qFA[0] << ' ' << qFA[1] << ' ' << qFA[2] << ' ' << qFA[3];
    ts << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << debugData[3];
    ts << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << wristAngleEncoder;
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

void CompensationIMU::cleanup()
{
    //    _robot.elbow->forward(0);
    _robot.wrist_pronosup->forward(0);
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &CompensationIMU::receiveData);
    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &CompensationIMU::receiveData);
    _file.close();
}
