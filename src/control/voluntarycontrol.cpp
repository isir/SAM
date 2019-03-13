#include "voluntarycontrol.h"
#include "peripherals/roboclaw/factory.h"

#include <wiringPi.h>
#include <iostream>

VoluntaryControl::VoluntaryControl() : _osmer(OsmerElbow::instance())
{
    _settings.beginGroup("VoluntaryControl");
    _pin_up = _settings.value("pin_up", 24).toInt();
    _pin_down = _settings.value("pin_down", 22).toInt();
    set_period(_settings.value("period",0.01).toDouble());

    _menu.set_title("Voluntary Control");
    _menu.set_code("vc");
    _menu.addItem(_osmer.menu());

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

VoluntaryControl::~VoluntaryControl() {
    _osmer.forward(0);
}

void VoluntaryControl::setup() {
    _osmer.calibration();
}

void VoluntaryControl::loop(double, double) {
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

    if(pin_down_value == 0 && prev_pin_down_value == 1) {
        _osmer.set_velocity(30);
    }
    else if(pin_up_value == 0 && prev_pin_up_value == 1) {
        _osmer.set_velocity(-30);
    }
    else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _osmer.move_to_angle(_osmer.angle());
    }

    prev_pin_down_value = pin_down_value;
    prev_pin_up_value = pin_up_value;
}

void VoluntaryControl::cleanup() {
    _osmer.forward(0);
}
