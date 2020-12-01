#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "roboclaw/roboclaw.h"
#include "utils/interfaces/menu_user.h"
#include <string>

class Actuator : public RC::RoboClaw, public MenuUser {
public:
    Actuator(std::string name);
    virtual ~Actuator() {}

    virtual void calibrate() {}

    double pos();
    using RC::RoboClaw::move_to;
    virtual void move_to(double deg, double speed, bool block = false);
    void set_velocity(double deg_s);
    virtual void set_velocity_safe(double deg_s);
    virtual bool is_calibrated() { return _calibrated; }
    virtual uint32_t r_incs_per_deg() { return _incs_per_deg; }

    double get_max_angle();
    double get_min_angle();
    uint32_t get_acc();

protected:
    virtual void on_exit();

    void calibrate(double velocity_deg_s, double final_pos, double velocity_threshold_deg_s, bool use_velocity_control = true);
    void set_params_limits(double lower_limit_deg, double upper_limit_deg);
    void set_params_technical(unsigned int incs_per_deg, unsigned int deg_s2);
    void set_params_velocity(float kp, float ki, float kd, unsigned int qpps);
    void set_params_position(float kp, float ki, float kd, uint32_t i_max, uint32_t deadzone, int32_t lower_limit, int32_t upper_limit);

private:
    std::string _name;

    bool _connected;
    bool _calibrated;

    uint32_t _incs_per_deg;
    uint32_t _acc;

    double _min_angle;
    double _max_angle;
};

#endif // ACTUATOR_H
