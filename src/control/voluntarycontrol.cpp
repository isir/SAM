#include "voluntarycontrol.h"
#include "peripherals/roboclaw/factory.h"

#include <wiringPi.h>
#include <iostream>

VoluntaryControl::VoluntaryControl() : _osmer(OsmerElbow::instance())
{
    _settings.beginGroup("VoluntaryControl");
    _pin_up = _settings.value("pin_up", 23).toInt();
    _pin_down = _settings.value("pin_down", 24).toInt();
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
    if(digitalRead(_pin_down)) {
        _osmer.set_velocity(30);
    }
    else if(digitalRead(_pin_up)) {
        _osmer.set_velocity(-30);
    }
    else {
        _osmer.set_velocity(0);
    }
}

void VoluntaryControl::cleanup() {
    _osmer.forward(0);
}
