#include "test.h"
#include "ui/visual/ledstrip.h"
#include <filesystem>
#include <iostream>
#include "utils/check_ptr.h"

Test::Test(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Test", .01)
    , _robot(robot)
{
    _menu->set_description("Test");
    _menu->set_code("test");
    _menu->add_item(_robot->sensors.touchsensor->menu());
}

Test::~Test()
{
    stop_and_join();
}

bool Test::setup()
{
    _robot->user_feedback.leds->set(LedStrip::white, 11);

    if (saveData) {
        std::string filename("test");
        std::string suffix;

        int cnt = 0;
        std::string extension(".txt");
        do {
            ++cnt;
            suffix = "_" + std::to_string(cnt);
        } while (std::filesystem::exists(filename + suffix + extension));

        _file = std::ofstream(filename + suffix + extension);
        if (!_file.good()) {
            critical() << "Failed to open" << (filename + suffix + extension);
            return false;
        }
    }

    _start_time = clock::now();
    return true;
}

void Test::loop(double, clock::time_point time)
{
    double timeWithDelta = std::chrono::duration_cast<std::chrono::microseconds>(time - _start_time).count();

    static int etatCourant = 0;
    static int cnt = 0;
    static int previous = 0;

    int keyStatus = _robot->sensors.touchsensor->readKeyStatus();
    std::cout << keyStatus << "\t" << etatCourant << std::endl;
    _mqtt.publish("sam/emg/time/1", std::to_string(keyStatus));
    _file << timeWithDelta << "\t" << keyStatus << std::endl;

    switch (etatCourant) {
    case 0:
        cnt = 0;
        previous = 0;
        if (keyStatus==8)
            etatCourant = 1;
        else if (keyStatus == 2)
            etatCourant = 5;
        else if (keyStatus == 1)
            etatCourant = 9;
        else if (keyStatus == 4)
            etatCourant = 13;
        break;
    case 1:
        if (keyStatus==10)
            etatCourant = 2;
        else if (keyStatus != 8)
            etatCourant = 17;
            previous = 8;
        break;
    case 2:
        if (keyStatus==2)
            etatCourant = 3;
        else if (keyStatus != 10)
            etatCourant = 0;
        break;
    case 3:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 2)
            etatCourant = 0;
        break;
    case 4 :
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 4)
            etatCourant = 0;
        break;
    case 5:
        if (keyStatus==10)
            etatCourant = 6;
        else if (keyStatus != 2)
            etatCourant = 17;
            previous = 2;
        break;
    case 6:
        if (keyStatus==8)
            etatCourant = 7;
        else if (keyStatus != 10)
            etatCourant = 0;
        break;
    case 7:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 8)
            etatCourant = 0;
        break;
    case 8:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 1)
            etatCourant = 0;
        break;
    case 9:
        if (keyStatus==5)
            etatCourant = 10;
        else if (keyStatus != 1)
            etatCourant = 17;
            previous = 1;
        break;
    case 10:
        if (keyStatus==4)
            etatCourant = 11;
        else if (keyStatus != 5)
            etatCourant = 0;
        break;
    case 11:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 4)
            etatCourant = 0;
        break;
    case 12:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 2)
            etatCourant = 0;
        break;
    case 13:
        if (keyStatus==5)
            etatCourant = 14;
        else if (keyStatus != 4)
            etatCourant = 17;
            previous = 4;
        break;
    case 14:
        if (keyStatus==1)
            etatCourant = 15;
        else if (keyStatus != 5)
            etatCourant = 0;
        break;
    case 15:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 1)
            etatCourant = 0;
        break;
    case 16:
        if (keyStatus==0) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            etatCourant = 0;
        } else if (keyStatus != 8)
            etatCourant = 0;
        break;
    case 17:
        if (cnt > 15) {
            etatCourant = 0;
        } else {
            if (keyStatus==1 && previous==2) {
                etatCourant = 8;
            } else if (keyStatus==2 && previous==1) {
                etatCourant = 12;
            } else if (keyStatus==4 && previous==8) {
                etatCourant = 4;
            } else if (keyStatus==8 && previous==4) {
                etatCourant = 16;
            } else if (keyStatus==0) {
                cnt++;
            } else {
                etatCourant = 0;
            }
        }
        break;
    }
}

void Test::cleanup()
{
    _file.close();
}
