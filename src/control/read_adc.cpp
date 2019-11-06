#include "read_adc.h"
#include "ui/visual/ledstrip.h"

ReadADC::ReadADC(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Read ADC", .1)
    , _robot(robot)
{
    _menu->set_description("Read ADC");
    _menu->set_code("adc");

    _th_low = 5000;
    _th_high = 10000;
}

ReadADC::~ReadADC()
{
    stop_and_join();
}

bool ReadADC::setup()
{
    _mqtt.publish("sam/emg/th/1", std::to_string(_th_low));
    _mqtt.publish("sam/emg/th/2", std::to_string(_th_high));
    _mqtt.subscribe("sam/emg/th/1", Mosquittopp::Client::QoS1)->add_callback(this, [this](Mosquittopp::Message msg) {this->_th_low = std::stoi(msg.payload()); });
    _mqtt.subscribe("sam/emg/th/2", Mosquittopp::Client::QoS1)->add_callback(this, [this](Mosquittopp::Message msg) {this->_th_high = std::stoi(msg.payload()); });
    _robot->user_feedback.leds->set(LedStrip::white, 10);
    return true;
}

void ReadADC::loop(double, clock::time_point)
{
    uint16_t nb_electrodes = 6;
    std::vector<uint16_t> electrodes(nb_electrodes,0);

    std::vector<LedStrip::color> colors(10, LedStrip::white);
    uint16_t flag_led = nb_electrodes+1;

    std::string payload;

    electrodes[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    electrodes[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    electrodes[2] = _robot->sensors.adc2->readADC_SingleEnded(2);
    electrodes[3] = _robot->sensors.adc2->readADC_SingleEnded(0);
    electrodes[4] = _robot->sensors.adc3->readADC_SingleEnded(2);
    electrodes[5] = _robot->sensors.adc3->readADC_SingleEnded(0);

    for (uint16_t i=0; i<nb_electrodes; i++)
    {
        colors[i] = LedStrip::white;

        if (electrodes[i] > _th_high)
        {
            flag_led = i;
            for (uint16_t j=0; j<nb_electrodes; j++)
            {
                if (electrodes[j] > _th_low && j!=i)
                {
                    flag_led = nb_electrodes+1;
                }
            }
        }

        payload += std::to_string(electrodes[i]/1000) + " ";
        _mqtt.publish("sam/emg/time/" + std::to_string(i),std::to_string(electrodes[i]));
        std::cout << electrodes[i] << "\t";
    }

    std::cout << _th_low << "\t" << _th_high << std::endl;

    if (flag_led<=nb_electrodes)
    {
        colors[flag_led] = LedStrip::red;
    }
    _robot->user_feedback.leds->set(colors);
    flag_led = nb_electrodes+1;

}

void ReadADC::cleanup()
{
    _robot->user_feedback.leds->set(LedStrip::white, 10);
}
