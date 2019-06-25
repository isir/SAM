#include "compensation_imu.h"
#include "peripherals/roboclaw/factory.h"

#include "control/algorithms/lawimu.h"
#include "qmath.h"
#include <QNetworkDatagram>
#include <iostream>
#include <wiringPi.h>

CompensationIMU::CompensationIMU(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt)
    : BasicController(mqtt)
    , _robot(robot)
{
    _settings.beginGroup("CompensationIMU");
    set_period(_settings.value("period", 0.02).toDouble());

    _menu.set_title("CompensationIMU");
    _menu.set_code("imu");
    _menu.addItem(_robot.elbow->menu());
}

CompensationIMU::~CompensationIMU()
{
    _robot.elbow->forward(0);
    _robot.wrist_pronosup->forward(0);
    stop();
}

bool CompensationIMU::setup()
{
    //_osmer.calibration();
    _robot.wrist_pronosup->set_encoder_position(0);
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
    return true;

    _cnt = 0;
    unsigned int init_cnt = _settings.value("init_count", 10).toInt();
    _time.start();
}

void CompensationIMU::loop(double, double)
{

    QTime t;
    t.start();
    double timeWithDelta = _time.elapsed() / 1000.;
    optitrack_data_t data = _robot.optitrack->get_last_data();
    if (_need_to_write_header) {
        _file.write(" time, emg1, emg2,");
        _file.write(" qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z");
        _file.write(" phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW, wristEncoder,");
        _file.write(" nbRigidBodies");
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file.write(", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z");
        }
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    /// WRIST
    double wristAngle = _robot.wrist_pronosup->read_encoder_position();

    double qBras[4], qTronc[4], qFA_record[4];
    _robot.arm_imu->get_quat(qBras);
    _robot.trunk_imu->get_quat(qTronc);
    _robot.fa_imu->get_quat(qFA_record);

    if (_cnt == 0) {
        _lawimu.initialization();
    } else if (_cnt <= init_cnt) {
        _lawimu.initialPositions(qFA_record, _cnt, init_cnt);
    } else {
        _lawimu.rotationMatrices(qFA_record, _cnt, init_cnt);
        _lawimu.controlLawWrist(_lambdaW, _thresholdW);

        if (_lawimu.returnWristVel_deg() > 0)
            _robot.wrist_pronosup->move_to(6000, _lawimu.returnWristVel_deg() * 100, 6000, 35000);
        else if (_lawimu.returnWristVel_deg() < 0)
            _robot.wrist_pronosup->move_to(6000, -_lawimu.returnWristVel_deg() * 100, 6000, -35000);
        else if (_lawimu.returnWristVel_deg() == 0)
            _robot.wrist_pronosup->forward(0);

        _lawimu.bufferingOldValues();

        if (_cnt % _settings.value("display_count", 50).toInt() == 0) {
            _lawimu.displayData();
        }
    }

    _lawimu.writeDebugData(debugData);

    QTextStream ts(&_file);
    ts << timeWithDelta << ' ' << _robot.adc->readADC_SingleEnded(0) << ' ' << _robot.adc->readADC_SingleEnded(1);
    ts << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    ts << ' ' << qFA_record[0] << ' ' << qFA_record[1] << ' ' << qFA_record[2] << ' ' << qFA_record[3];
    ts << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2];
    ts << ' ' << debugData[3] << ' ' << debugData[4] << ' ' << debugData[5] << ' ' << debugData[6] << ' ' << debugData[7] << ' ' << debugData[8];
    ts << ' ' << debugData[9] << ' ' << debugData[10] << ' ' << debugData[11] << ' ' << debugData[12];
    ts << ' ' << debugData[13] << ' ' << debugData[14] << ' ' << debugData[15] << ' ' << debugData[16] << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << wristAngleEncoder;
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
    //_robot.elbow->forward(0);
    _robot.wrist_pronosup->forward(0);
    _file.close();
}
