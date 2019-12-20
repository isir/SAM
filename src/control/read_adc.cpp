#include "read_adc.h"
#include "ui/visual/ledstrip.h"

ReadADC::ReadADC(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Read ADC", .01)
    , _robot(robot)
{
    _menu->set_description("Read ADC");
    _menu->set_code("adc");

    _th_low[0] = 3000;
    _th_low[1] = 1500;
    _th_low[2] = 2000;
    _th_low[3] = 3000;
    _th_low[4] = 2000;
    _th_low[5] = 1000;

    _th_high[0] = 3000;
    _th_high[1] = 3000;
    _th_high[2] = 3000;
    _th_high[3] = 3000;
    _th_high[4] = 2000;
    _th_high[5] = 5000;

}

ReadADC::~ReadADC()
{
    stop_and_join();
}

bool ReadADC::setup()
{
    _robot->joints.elbow_flexion->calibrate();
    if (_robot->joints.wrist_flexion) {
        _robot->joints.wrist_flexion->calibrate();
    }
    _robot->joints.wrist_pronation->calibrate();


    //_mqtt.publish("sam/emg/th/1", std::to_string(_th_low[0]));
    //_mqtt.publish("sam/emg/th/2", std::to_string(_th_high[0]));
    //_mqtt.subscribe("sam/emg/th/1", Mosquittopp::Client::QoS1)->add_callback(this, [this](Mosquittopp::Message msg) {this->_th_low[0] = std::stoi(msg.payload()); });
    //_mqtt.subscribe("sam/emg/th/2", Mosquittopp::Client::QoS1)->add_callback(this, [this](Mosquittopp::Message msg) {this->_th_high[0] = std::stoi(msg.payload()); });

    _robot->user_feedback.leds->set(LedStrip::white, 10);

    return true;
}

void ReadADC::loop(double, clock::time_point)
{
    std::vector<uint16_t> electrodes(_n_electrodes,0);

    std::vector<LedStrip::color> colors(_n_electrodes, LedStrip::white);

    std::string payload;

    electrodes[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    electrodes[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    electrodes[2] = _robot->sensors.adc2->readADC_SingleEnded(2);
    electrodes[3] = _robot->sensors.adc2->readADC_SingleEnded(0);
    electrodes[4] = _robot->sensors.adc3->readADC_SingleEnded(2);
    electrodes[5] = _robot->sensors.adc3->readADC_SingleEnded(0);

    if (electrodes[2] > _th_high[2] && electrodes[4] < _th_low[4] && electrodes[3] < _th_high[3])
    {
        _robot->joints.elbow_flexion->set_velocity_safe(25);
    } else if (electrodes[4] > _th_high[4] && electrodes[2] < _th_low[2] && electrodes[3] < _th_high[3])
    {
        _robot->joints.elbow_flexion->set_velocity_safe(-25);
    } else
    {
        _robot->joints.elbow_flexion->set_velocity_safe(0);
    }

    if (electrodes[3] > _th_high[3] && electrodes[5] < _th_low[5])
    {
        _robot->joints.wrist_pronation->set_velocity_safe(-40);
    } else if (electrodes[5] > _th_high[5] && electrodes[3] < _th_low[3])
    {
        _robot->joints.wrist_pronation->set_velocity_safe(40);
    } else
    {
        _robot->joints.wrist_pronation->set_velocity_safe(0);
    }

    if (electrodes[0] > _th_high[0] && electrodes[1] < _th_low[1])
    {
        // _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,1);
    } else if (electrodes[1] > _th_high[1] && electrodes[0] < _th_low[0])
    {
        // _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,1);
    } else
    {
    }


    for (uint16_t i=0; i<_n_electrodes; i++)
    {
        colors[i] = LedStrip::white;

        if (electrodes[i] > _th_high[i])
        {
            colors[i] = LedStrip::red_bright;
        } else if (electrodes[i] > _th_low[i]) {
            colors[i] = LedStrip::green;
        }
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(electrodes[i]));
        std::cout << electrodes[i] << "\t";
    }

    std::cout << std::endl;
    _robot->user_feedback.leds->set(colors);

}

void ReadADC::cleanup()
{
    _robot->joints.wrist_pronation->set_velocity_safe(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _robot->user_feedback.leds->set(LedStrip::white, 10);
}
