#include "compensationoptitrack.h"
#include "wiringPi.h"
#include <QDir>
#include <QNetworkDatagram>
#include <iostream>
#include <qmath.h>

CompensationOptitrack::CompensationOptitrack(QObject* parent)
    : QObject(parent)
    , _osmer(OsmerElbow::instance())
    , _pronosup(PronoSupination::instance())
    , _hand(TouchBionicsHand::instance())
    , _optitrack(OptiListener::instance())
    , _adc("/dev/i2c-1", 0x48)
    //    , _imu_bras("/dev/ximu_red", XIMU::XIMU_LOGLEVEL_NONE, 115200)
    //    , _imu_tronc("/dev/ximu_white", XIMU::XIMU_LOGLEVEL_NONE, 115200)
    , _imu_bras("/dev/ttyUSB1", XIMU::XIMU_LOGLEVEL_NONE, 115200)
    , _imu_tronc("/dev/ttyUSB2", XIMU::XIMU_LOGLEVEL_NONE, 115200)
    , _Lt(40)
    , _Lua(0.)
    , _Lfa(0.)
    , _l(0.)
    , _lsh(-35)
    , _lambda(0)
    , _threshold(0.)
    , _lambdaW(0)
    , _thresholdW(5.)
    , _buzzer(29)
    , _pinArduino(0)
{
    _settings.beginGroup("CompensationOptitrack");

    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &CompensationOptitrack::on_def);
    if (!_receiver.bind(QHostAddress::AnyIPv4, 45454)) {
        qCritical() << _receiver.errorString();
    }

    QObject::connect(&_receiverArduino, &QUdpSocket::readyRead, this, &CompensationOptitrack::listenArduino);
    if (!_receiverArduino.bind(QHostAddress::AnyIPv4, 45455)) {
        qCritical() << 'Arduino receiver' << _receiverArduino.errorString();
    }

    _abs_time.start();

    _menu.set_title("Control with optitrack recording");
    _menu.set_code("opti");
    _menu.addItem(ConsoleMenuItem("Start (+ filename [comp for compensation, vol for voluntary control])", "1", [this](QString args) { this->start(args); }));
    _menu.addItem(ConsoleMenuItem("Stop", "s", [this](QString) { this->stop(); }));
    _menu.addItem(ConsoleMenuItem("Back to 0°", "zero", [this](QString) { this->zero(); }));
    _menu.addItem(ConsoleMenuItem("Display law parameters", "disp", [this](QString) { this->display_parameters(); }));
    _menu.addItem(ConsoleMenuItem("Display Arduino data", "ard", [this](QString) { this->displayArduino(); }));
    _menu.addItem(ConsoleMenuItem("Display anatomical lengths", "al", [this](QString) { this->display_lengths(); }));
    _menu.addItem(ConsoleMenuItem("Tare IMU", "tare", [this](QString) { this->tareIMU(); }));
    _menu.addItem(_osmer.menu());
    _menu.addItem(_pronosup.menu());
    _menu.addItem(_hand.menu());

    QObject::connect(&_menu, &ConsoleMenu::finished, this, &CompensationOptitrack::stop);
    QObject::connect(&_menu, &ConsoleMenu::activated, this, &CompensationOptitrack::on_activated);

    _optitrack.begin(_settings.value("port", 1511).toInt());

    _pin_up = _settings.value("pin_up", 24).toInt();
    _pin_down = _settings.value("pin_down", 22).toInt();
    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

CompensationOptitrack::~CompensationOptitrack()
{
    stop();
}

void CompensationOptitrack::zero()
{
    _osmer.move_to_angle(0, 10, true);
}

void CompensationOptitrack::tareIMU()
{
    _imu_bras.send_command_algorithm_init_then_tare();
    _imu_tronc.send_command_algorithm_init_then_tare();
    qDebug("Wait for triple bip");

    usleep(6 * 1000000);
    _buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ);

    double qBras[4], qTronc[4];
    _imu_bras.get_quat(qBras);
    _imu_tronc.get_quat(qTronc);
    qDebug() << "IMU Bras : " << qBras[0] << " " << qBras[1] << " " << qBras[2] << " " << qBras[3];
    qDebug() << "IMU Tronc : " << qTronc[0] << " " << qTronc[1] << " " << qTronc[2] << " " << qTronc[3];
}

void CompensationOptitrack::display_parameters()
{
    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &CompensationOptitrack::on_def);
    qDebug("lambda:%d", _lambda);
    qDebug("Lfa:%lf", _Lfa);
    qDebug("Lua:%lf", _Lua);
    qDebug("l: %lf", _l);
    qDebug("Threshold (in rad):%lf", _threshold);
    qDebug("Threshold wrist (in rad):%lf", _thresholdW);
    qDebug("lambda wrist :%d", _lambdaW);
}

void CompensationOptitrack::display_lengths()
{
    _ind = 0;
    qInfo("Wait for Optitrack data");
    printf("Wait for Optitrack data");
    QObject::connect(&_optitrack, &OptiListener::new_data, this, &CompensationOptitrack::read_optiData);
}

void CompensationOptitrack::read_optiData(optitrack_data_t data)
{
    unsigned int opti_freq = _settings.value("optitrack_frequency", 200).toInt();

    int index_acromion = -1, index_EE = -1, index_elbow = -1, index_hip = -1;
    Eigen::Vector3d posA = Eigen::Vector3d::Zero(), posElbow = Eigen::Vector3d::Zero(), posFA = Eigen::Vector3d::Zero(), posEE = Eigen::Vector3d::Zero(), posHip = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qHip, qFA_record;
    qHip.w() = 0.;
    qFA_record.w() = 0.;
    qHip.x() = 0.;
    qFA_record.x() = 0.;
    qHip.y() = 0.;
    qFA_record.y() = 0.;
    qHip.z() = 0.;
    qFA_record.z() = 0.;

    for (int i = 0; i < data.nRigidBodies; i++) {
        if (data.rigidBodies[i].ID == _settings.value("acromion_id", 3).toInt()) {
            posA[0] = data.rigidBodies[i].x * 100;
            posA[1] = data.rigidBodies[i].y * 100;
            posA[2] = data.rigidBodies[i].z * 100;
            index_acromion = i;
        } else if (data.rigidBodies[i].ID == _settings.value("fa_id", 4).toInt()) {
            posFA[0] = data.rigidBodies[i].x * 100;
            posFA[1] = data.rigidBodies[i].y * 100;
            posFA[2] = data.rigidBodies[i].z * 100;

            qFA_record.w() = data.rigidBodies[i].qw;
            qFA_record.x() = data.rigidBodies[i].qx;
            qFA_record.y() = data.rigidBodies[i].qy;
            qFA_record.z() = data.rigidBodies[i].qz;
        } else if (data.rigidBodies[i].ID == _settings.value("elbow_id", 6).toInt()) {
            posElbow[0] = data.rigidBodies[i].x * 100;
            posElbow[1] = data.rigidBodies[i].y * 100;
            posElbow[2] = data.rigidBodies[i].z * 100;
            index_elbow = i;
        } else if (data.rigidBodies[i].ID == _settings.value("ee_id", 9).toInt()) {
            posEE[0] = data.rigidBodies[i].x * 100;
            posEE[1] = data.rigidBodies[i].y * 100;
            posEE[2] = data.rigidBodies[i].z * 100;
            index_EE = i;
        } else if (data.rigidBodies[i].ID == _settings.value("hip_id", 10).toInt()) {
            posHip[0] = data.rigidBodies[i].x * 100;
            posHip[1] = data.rigidBodies[i].y * 100;
            posHip[2] = data.rigidBodies[i].z * 100;
            qHip.w() = data.rigidBodies[i].qw;
            qHip.x() = data.rigidBodies[i].qx;
            qHip.y() = data.rigidBodies[i].qy;
            qHip.z() = data.rigidBodies[i].z;
            index_hip = i;
        }
    }

    _lawopti.initialization(posA, posEE, posHip, qHip, opti_freq);
    _lawopti.rotationMatrices(qHip, qFA_record, 1, 10);
    _lawopti.computeEEfromFA(posFA, _l, qFA_record);
    _lawopti.projectionInHip(posA, posElbow, posHip, 1, 10);
    _Lua = qRound((_lawopti.returnPosElbowinHip() - _lawopti.returnPosAinHip()).norm());
    _Lfa = qRound((posElbow - posFA).norm());
    _l = qRound((posFA - posEE).norm());

    if (_ind == 0) {
        qDebug("posA: %lf, %lf, %lf", posA[0], posA[1], posA[2]);
        qDebug("posFA: %lf, %lf, %lf", posFA[0], posFA[1], posFA[2]);
        qDebug("posEE: %lf, %lf, %lf", posEE[0], posEE[1], posEE[2]);
        qDebug("posHip: %lf, %lf, %lf", posHip[0], posHip[1], posHip[2]);
        qDebug("qHip: %lf, %lf, %lf, %lf", qHip.w(), qHip.x(), qHip.y(), qHip.z());
        qDebug("Lua: %lf, Lfa: %lf, l: %lf", _Lua, _Lfa, _l);
        _ind = 1;
    }
}

void CompensationOptitrack::start(QString filename = QString())
{
    if (filename.isEmpty())
        filename = "test";

    int cnt = 0;
    QString extension = QString(".txt");
    do {
        ++cnt;
        QString suffix = QString("_") + QString::number(cnt);
        _file.setFileName(filename + suffix + extension);
    } while (_file.exists());

    if (!_file.open(QIODevice::ReadWrite)) {
        qCritical() << "Failed to open" << _file.fileName() << "-" << _file.errorString();
        return;
    }
    _need_to_write_header = true;

    _time.start();
    _old_time = 0.;
    _previous_elapsed = 0;

    _cnt = 0;
    _infoSent = 0;
    QObject::disconnect(&_receiver, &QUdpSocket::readyRead, this, &CompensationOptitrack::on_def);
    QObject::connect(&_receiverArduino, &QUdpSocket::readyRead, this, &CompensationOptitrack::listenArduino);

    if ((filename == QString("comp")) || (filename == QString("n")) || (filename == QString("b"))) {
        QObject::connect(&_optitrack, &OptiListener::new_data, this, &CompensationOptitrack::on_new_data_compensation);
    } else if (filename == QString("vol")) {
        QObject::connect(&_optitrack, &OptiListener::new_data, this, &CompensationOptitrack::on_new_data_vol);
    } else {
        qDebug("Filename does not correspond to one of the control mode.\n 'n' for natural; 'b' for blocked; 'opti' for compensation control; 'vol', for voluntary");
    }
}

void CompensationOptitrack::stop()
{
    QObject::disconnect(&_optitrack, &OptiListener::new_data, this, &CompensationOptitrack::on_new_data_compensation);
    QObject::disconnect(&_optitrack, &OptiListener::new_data, this, &CompensationOptitrack::on_new_data_vol);
    QObject::disconnect(&_optitrack, &OptiListener::new_data, this, &CompensationOptitrack::read_optiData);
    _infoSent = 0;

    _osmer.forward(0);
    _pronosup.forward(0);
    _file.close();
    QObject::connect(&_receiver, &QUdpSocket::readyRead, this, &CompensationOptitrack::on_def);
}

void CompensationOptitrack::on_new_data_compensation(optitrack_data_t data)
{
    unsigned int init_cnt = _settings.value("init_count", 10).toInt();
    int btn_sync = digitalRead(_settings.value("btn_sync", 29).toInt());

    Eigen::Vector3d posA, posElbow, posFA, posEE, posHip;
    Eigen::Quaterniond qHip, qFA_record;
    double timeWithDelta = _time.elapsed() / 1000.;
    double deltaTtable = timeWithDelta - _previous_elapsed;
    double absTtable = _abs_time.elapsed() / 1000.;

    //    while ((timeWithDelta - _old_time) < 0.01) {
    //        timeWithDelta = _time.elapsed() / 1000.;
    //        qDebug("Wait in the time loop");
    //        //        qDebug("Time elasped:%f", timeWithDelta);
    //        //        qDebug("Old time: %f", _old_time);
    //    }
    //    _old_time = timeWithDelta;
    _previous_elapsed = timeWithDelta;

    int timerTask = 1;

    double qBras[4], qTronc[4];
    _imu_bras.get_quat(qBras);
    _imu_tronc.get_quat(qTronc);
    //    qDebug() << "IMU Bras : " << qBras[0] << " " << qBras[1] << " " << qBras[2] << " " << qBras[3];
    //    qDebug() << "IMU Tronc : " << qTronc[0] << " " << qTronc[1] << " " << qTronc[2] << " " << qTronc[3];

    double wristAngleEncoder = _pronosup.read_encoder_position();
    double debugData[35];

    int index_acromion = -1, index_FA = -1, index_EE = -1, index_elbow = -1, index_hip = -1;

    for (int i = 0; i < data.nRigidBodies; i++) {
        if (data.rigidBodies[i].ID == _settings.value("acromion_id", 3).toInt()) {
            posA[0] = data.rigidBodies[i].x * 100;
            posA[1] = data.rigidBodies[i].y * 100;
            posA[2] = data.rigidBodies[i].z * 100;
            index_acromion = i;
        } else if (data.rigidBodies[i].ID == _settings.value("fa_id", 4).toInt()) {
            posFA[0] = data.rigidBodies[i].x * 100;
            posFA[1] = data.rigidBodies[i].y * 100;
            posFA[2] = data.rigidBodies[i].z * 100;
            qFA_record.w() = data.rigidBodies[i].qw;
            qFA_record.x() = data.rigidBodies[i].qx;
            qFA_record.y() = data.rigidBodies[i].qy;
            qFA_record.z() = data.rigidBodies[i].qz;
            index_FA = i;
        } else if (data.rigidBodies[i].ID == _settings.value("elbow_id", 6).toInt()) {
            posElbow[0] = data.rigidBodies[i].x * 100;
            posElbow[1] = data.rigidBodies[i].y * 100;
            posElbow[2] = data.rigidBodies[i].z * 100;
            index_elbow = i;
        } else if (data.rigidBodies[i].ID == _settings.value("ee_id", 9).toInt()) {
            posEE[0] = data.rigidBodies[i].x * 100;
            posEE[1] = data.rigidBodies[i].y * 100;
            posEE[2] = data.rigidBodies[i].z * 100;
            index_EE = i;
        } else if (data.rigidBodies[i].ID == _settings.value("hip_id", 10).toInt()) {
            posHip[0] = data.rigidBodies[i].x * 100;
            posHip[1] = data.rigidBodies[i].y * 100;
            posHip[2] = data.rigidBodies[i].z * 100;
            qHip.w() = data.rigidBodies[i].qw;
            qHip.x() = data.rigidBodies[i].qx;
            qHip.y() = data.rigidBodies[i].qy;
            qHip.z() = data.rigidBodies[i].qz;
            index_hip = i;
        }
    }

    double beta = _osmer.angle() * M_PI / 180.;

    if (_need_to_write_header) {
        _file.write("delta, time, btn_sync, abs_time, emg1, emg2, timerTask, pinArduino,");
        _file.write(" qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z");
        _file.write(" index_acromion, index_EE, index_elbow, initialAcromionPosition.x, initialAcromionPosition.y, initialAcromionPosition.z, AcromionPosition.x, AcromionPosition.y, AcromionPosition.z,");
        _file.write(" positionEE_inHip.x, positionEE_inHip.y, positionEE_inHip.z, delta, beta_new, beta, dBeta, betaDot, lambda, threshold, nbRigidBodies,");
        _file.write(" phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW");
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file.write(", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z");
        }
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    if (_cnt == 0) {
        unsigned int opti_freq = _settings.value("optitrack_frequency", 100).toInt();
        if (_Lua == 0 && _Lfa == 0) {
            //            _Lua = qRound((posElbow - posA).norm());
            //            //            _Lfa = qRound((posElbow - posEE).norm());
            //            _Lfa = qRound((posElbow - posFA).norm());
            //            _l = qRound((posFA - posEE).norm());
            //            qDebug("Lua: %lf, Lfa: %lf, l: %lf", _Lua, _Lfa, _l);
            qDebug("Anatomical lengths not defined");
        }
        _lawopti.initialization(posA, posEE, posHip, qHip, opti_freq);
    } else if (_cnt <= init_cnt) {
        _lawopti.initialPositions(posA, posHip, qHip, qFA_record, _cnt, init_cnt);
        if (_cnt == init_cnt) {
            /// Computation posA0 in hip frame and move posA0 in prosthetic arm plane
            _lawopti.rotationMatrices(qHip, qFA_record, _cnt, init_cnt);
            _lawopti.computeEEfromFA(posFA, _l, qFA_record);
            _lawopti.projectionInHip(posA, posElbow, posHip, _cnt, init_cnt);
            _lawopti.bufferingOldValues();
        }
        _lawopti.filter_optitrackData(posA, posEE);
    } else {
        _lawopti.rotationMatrices(qHip, qFA_record, _cnt, init_cnt);
        _lawopti.computeEEfromFA(posFA, _l, qFA_record);
        _lawopti.projectionInHip(posA, posElbow, posHip, _cnt, init_cnt);
        _lawopti.controlLaw(posEE, beta, _Lua, _Lfa, _l, _lambda, _threshold);
        _lawopti.controlLawWrist(_lambdaW, _thresholdW);
        _osmer.set_velocity(_lawopti.returnBetaDot_deg());

        if (_lawopti.returnWristVel_deg() > 0)
            _pronosup.move_to(6000, _lawopti.returnWristVel_deg() * 100, 6000, 35000);
        else if (_lawopti.returnWristVel_deg() < 0)
            _pronosup.move_to(6000, -_lawopti.returnWristVel_deg() * 100, 6000, -35000);
        else if (_lawopti.returnWristVel_deg() == 0)
            _pronosup.forward(0);

        _lawopti.bufferingOldValues();

        if (_cnt % _settings.value("display_count", 50).toInt() == 0) {
            _lawopti.displayData(posEE, beta);
            //            qDebug() << "betaDot in deg:" << _lawopti.returnBetaDot_deg();
        }
        // buzzer after 1s, to indicate the start of the task
        if (_cnt == 100) {
            _buzzer.makeNoise(BuzzerConfig::STANDARD_BUZZ);
        }
    }

    _lawopti.writeDebugData(debugData, posEE, beta);

    QTextStream ts(&_file);
    //ts.setPadChar(' ');
    if (_cnt == 0) {
        ts << _Lua << ' ' << _Lfa << ' ' << _l << endl;
    }
    ts << deltaTtable << ' ' << timeWithDelta << ' ' << btn_sync << ' ' << absTtable << ' ' << _adc.readADC_SingleEnded(0) << ' ' << _adc.readADC_SingleEnded(1) << ' ' << timerTask;
    ts << ' ' << _pinArduino;
    ts << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    ts << ' ' << index_acromion << ' ' << index_EE << ' ' << index_elbow << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << posA[0] << ' ' << posA[1] << ' ' << posA[2];
    ts << ' ' << debugData[3] << ' ' << debugData[4] << ' ' << debugData[5] << ' ' << debugData[6] << ' ' << debugData[7] << ' ' << debugData[8];
    ts << ' ' << debugData[9] << ' ' << debugData[10] << ' ' << debugData[11] << ' ' << debugData[12] << ' ' << _lambda << ' ' << _threshold << ' ' << data.nRigidBodies;
    ts << ' ' << debugData[13] << ' ' << debugData[14] << ' ' << debugData[15] << ' ' << debugData[16] << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << wristAngleEncoder;

    for (int i = 0; i < data.nRigidBodies; i++) {
        ts << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        ts << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        ts << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    ts << endl;

    ++_cnt;
}

void CompensationOptitrack::on_new_data_vol(optitrack_data_t data)
{
    double timeWithDelta = _time.elapsed() / 1000.;

    //    while ((timeWithDelta - _old_time) < 0.01) {
    //        timeWithDelta = _time.elapsed() / 1000.;
    //        qDebug("Wait in the time loop");
    //        qDebug("Time elasped:%f", timeWithDelta);
    //        qDebug("Old time: %f", _old_time);
    //    }
    _old_time = timeWithDelta;
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

    /// ELBOW
    //    double beta = _osmer.angle() * M_PI / 180.;

    //    if (pin_down_value == 0 && prev_pin_down_value == 1) {
    //        _osmer.set_velocity(30);
    //    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
    //        _osmer.set_velocity(-30);
    //    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
    //        _osmer.set_velocity(0);
    //        //        sleep(2);
    //        //        _osmer.set_velocity(0);
    //    }

    /// WRIST
    double wristAngle = _pronosup.read_encoder_position();

    if (pin_down_value == 0 && prev_pin_down_value == 1) {
        _pronosup.move_to(6000, 5000, 6000, 35000);
    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
        _pronosup.move_to(6000, 5000, 6000, -35000);
    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _pronosup.forward(0);
    }

    prev_pin_down_value = pin_down_value;
    prev_pin_up_value = pin_up_value;

    double qBras[4], qTronc[4];
    _imu_bras.get_quat(qBras);
    _imu_tronc.get_quat(qTronc);
    if (_need_to_write_header) {
        //        _file.write("period, btnUp, btnDown, beta");
        _file.write("time, btnUp, btnDown, pinArduino, wristAngle,");
        _file.write(" qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z");
        _file.write(" nbRigid Bodies");
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file.write(", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z");
        }
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    QTextStream ts(&_file);
    //ts.setPadChar(' ');
    //    ts << return_period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << beta;
    ts << timeWithDelta << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << _pinArduino << ' ' << wristAngle;
    ts << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    ts << ' ' << data.nRigidBodies;

    for (int i = 0; i < data.nRigidBodies; i++) {
        ts << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        ts << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        ts << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    ts << endl;
}

void CompensationOptitrack::on_activated()
{
    _osmer.calibration();
    _pronosup.set_encoder_position(0);
}

void CompensationOptitrack::on_def()
{
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
    }
}

void CompensationOptitrack::listenArduino()
{
    while (_receiverArduino.hasPendingDatagrams()) {
        if (_infoSent == 0) {
            qDebug("Listening Arduino");
            _infoSent = 1;
        }
        QByteArray dataReceivedArduino = _receiverArduino.receiveDatagram().data();
        QTextStream tsA(&dataReceivedArduino);
        int tmp;

        tsA >> tmp;
        _pinArduino = tmp;
    }
}

void CompensationOptitrack::displayArduino()
{
    QObject::connect(&_receiverArduino, &QUdpSocket::readyRead, this, &CompensationOptitrack::listenArduino);
    qDebug("Pin Arduino: %d", _pinArduino);
}
