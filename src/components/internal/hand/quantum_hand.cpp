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
    _menu->add_item("dc", "Double Contraction (1-2)", [this](std::string args) { if(args.length() == 0) args = "1"; makeContraction(DOUBLE_CONTRACTION, std::stoi(args)); });
    _menu->add_item("tc", "Triple Contraction (1-2)", [this](std::string args) { if(args.length() == 0) args = "1"; makeContraction(TRIPLE_CONTRACTION, std::stoi(args)); });
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
}

void QuantumHand::makeContraction(CONTRACTION_TYPE contraction_type, int emg, uint16_t intensity)
{
    if (_thread.joinable()) {
        _thread.join();
    }

    _cfg.time_contraction_us = 500000;
    _cfg.time_between_contractions_us = 100000;

    _cfg.intensity = intensity;
    if (intensity > 4) {
        _cfg.intensity = 4;
    } else if (intensity < 1) {
        _cfg.intensity = 1;
    }


    _cfg.emg1_activated  = true;
    if (emg == 2) {
        _cfg.emg2_activated = true;
        _cfg.emg1_activated = false;
    } else {
        _cfg.emg1_activated = true;
        _cfg.emg2_activated = false;
    }

    _cfg.n_contractions = 1;

    switch (contraction_type) {
    case SHORT_CONTRACTION:
        _cfg.time_contraction_us = 100000;
        break;
    case LONG_CONTRACTION:
        _cfg.time_contraction_us = 5000000/intensity;
        break;
    case DOUBLE_CONTRACTION:
        _cfg.n_contractions = 2;
        break;
    case TRIPLE_CONTRACTION:
        _cfg.n_contractions = 3;
        break;
    case CO_CONTRACTION:
        _cfg.emg1_activated = true;
        _cfg.emg2_activated = true;
        _cfg.time_contraction_us = 100000;
        break;
    }

    start();
}
