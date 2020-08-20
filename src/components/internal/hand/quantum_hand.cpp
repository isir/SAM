#include "quantum_hand.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"

QuantumHand::QuantumHand()
    : ThreadedLoop("quantum_hand", 0.)
{
    _dac = std::make_unique<MCP4728>("/dev/i2c-1", 0x61);

    _menu->set_description("Quantum Hand ");
    _menu->set_code("quantum");

    _menu->add_item("sc", "Short Contraction (1-2)", [this](std::string args) { if(args.length() == 0) args = "1"; makeContraction(SHORT_CONTRACTION, std::stoi(args)); });
    _menu->add_item("lc", "Long Contraction (1-2)", [this](std::string args) { if(args.length() == 0) args = "1"; makeContraction(LONG_CONTRACTION, std::stoi(args)); });
    _menu->add_item("dc", "Double Contraction", [this](std::string) { makeContraction(DOUBLE_CONTRACTION); });
    _menu->add_item("tc", "Triple Contraction", [this](std::string) { makeContraction(TRIPLE_CONTRACTION); });
    _menu->add_item("cc", "Co Contraction", [this](std::string) { makeContraction(CO_CONTRACTION); });
}

QuantumHand::~QuantumHand()
{
    stop_and_join();
}

bool QuantumHand::setup()
{
    return true;
}

void QuantumHand::loop(double, clock::time_point)
{
    if (_cfg.opening_before) {
        if (_cfg.emg1_activated) {
            _dac->analogWrite(2,_cfg.intensity*250);
        }
        if (_cfg.emg2_activated) {
            _dac->analogWrite(3,_cfg.intensity*250);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(_cfg.time_opening_before_us));

        if (_cfg.emg1_activated) {
            _dac->analogWrite(2,0);
        }
        if (_cfg.emg2_activated) {
            _dac->analogWrite(3,0);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(_cfg.time_between_contractions_us*5));
    }

    for (; _cfg.n_contractions > 0; --_cfg.n_contractions) {
        if (_cfg.emg1_activated) {
            _dac->analogWrite(2,_cfg.intensity*250);
        }
        if (_cfg.emg2_activated) {
            _dac->analogWrite(3,_cfg.intensity*250);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(_cfg.time_contraction_us));

        if (_cfg.emg1_activated) {
            _dac->analogWrite(2,0);
        }
        if (_cfg.emg2_activated) {
            _dac->analogWrite(3,0);
        }

        if (_cfg.n_contractions > 1)
            std::this_thread::sleep_for(std::chrono::microseconds(_cfg.time_between_contractions_us));
    }
    stop();
}

void QuantumHand::cleanup()
{
    _dac->analogWrite(2,0);
    _dac->analogWrite(3,0);
}

void QuantumHand::makeContraction(CONTRACTION_TYPE contraction_type, int emg, uint16_t intensity, bool opening)
{
    if (_thread.joinable()) {
        _thread.join();
    }

    _cfg.time_contraction_us = 200000;
    _cfg.time_between_contractions_us = 100000;

    _cfg.intensity = intensity;
    if (intensity > 4) {
        _cfg.intensity = 4;
    } else if (intensity < 1) {
        _cfg.intensity = 1;
    }

    if (emg == 1) {
        _cfg.emg1_activated = true;
        _cfg.emg2_activated = false;
    } else {
        _cfg.emg2_activated = true;
        _cfg.emg1_activated = false;
    }

    _cfg.n_contractions = 1;

    _cfg.opening_before = false;
    _cfg.time_opening_before_us = 1250000;

    switch (contraction_type) {
    case SHORT_CONTRACTION:
        _cfg.time_contraction_us = 100000;
        break;
    case LONG_CONTRACTION:
        _cfg.time_contraction_us = 5000000/ _cfg.intensity;
        break;
    case DOUBLE_CONTRACTION:
        _cfg.n_contractions = 2;
        _cfg.emg2_activated = true;
        _cfg.emg1_activated = false;
        _cfg.intensity = 4;
        _cfg.opening_before = opening;
        break;
    case TRIPLE_CONTRACTION:
        _cfg.n_contractions = 3;
        _cfg.emg2_activated = true;
        _cfg.emg1_activated = false;
        _cfg.intensity = 4;
        _cfg.opening_before = opening;
        break;
    case CO_CONTRACTION:
        _cfg.time_contraction_us = 100000;
        _cfg.emg1_activated = true;
        _cfg.emg2_activated = true;
        _cfg.intensity = 4;
        break;
    }

    start();
}
