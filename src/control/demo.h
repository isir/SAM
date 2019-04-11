#ifndef DEMO_H
#define DEMO_H

#include "basiccontroller.h"
#include "peripherals/buzzer.h"
#include "peripherals/helpers/osmerelbow.h"
#include "peripherals/helpers/pronosupination.h"
#include "peripherals/myoband/myoband.h"
#include "peripherals/touch_bionics/touch_bionics_hand.h"

class Demo : public BasicController {
public:
    Demo();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

private:
    Buzzer _buzzer;
    PronoSupination& _pronosup;
    OsmerElbow& _osmer;
    Myoband _myoband;
    TouchBionicsHand& _hand;
};

#endif // DEMO_H
