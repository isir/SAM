#ifndef QUANTUMHAND_H
#define QUANTUMHAND_H

#include "utils/threaded_loop.h"
#include "components/internal/dac/mcp4728.h"
#include "utils/interfaces/mqtt_user.h"

class QuantumHand : public ThreadedLoop
{
public:
    enum CONTRACTION_TYPE {
        SHORT_CONTRACTION,
        LONG_CONTRACTION,
        DOUBLE_CONTRACTION,
        TRIPLE_CONTRACTION,
        CO_CONTRACTION,
        STOP
    };

    struct ContractionConfig {
        bool time_limited;
        int n_contractions;
        int time_contraction_us;
        int time_between_contractions_us;
        bool emg1_activated;
        bool emg2_activated;
        bool opening_before;
        int time_opening_before_us;
        uint16_t intensity;
    };

    QuantumHand();
    ~QuantumHand() override;

    void makeContraction(CONTRACTION_TYPE contraction_type=SHORT_CONTRACTION, int emg=2, uint16_t intensity=2, bool opening=true);

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::unique_ptr<MCP4728> _dac;
    ContractionConfig _cfg;
};

#endif // QUANTUMHAND_H
