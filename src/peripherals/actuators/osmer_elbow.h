#ifndef OSMERELBOW_H
#define OSMERELBOW_H

#include "peripherals/roboclaw/client.h"
#include "ui/consolemenu.h"
#include "utils/settings.h"

/**
 * @brief The OsmerElbow class creates an instance to control the osmer elbow through roboclaw.
 */
class OsmerElbow : public RoboClaw::Client {
    Q_OBJECT
public:
    OsmerElbow(std::shared_ptr<QMqttClient> mqtt);
    ~OsmerElbow();

    ConsoleMenu& menu();

    void calibration();
    double angle();
    void move_to_angle(double angle, double speed = 30, bool block = false);
    void set_velocity(double value);

private:
    ConsoleMenu _menu;
    Settings _settings;

    bool _calibrated;
    int _calibration_velocity_threshold;
    int _incs_per_deg;
    qint32 _accel_decel;

    double _min_angle;
    double _max_angle;

private slots:
    void on_exit();
};
#endif // OSMERELBOW_H