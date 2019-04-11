#ifndef REMOTECOMPUTERCONTROL_H
#define REMOTECOMPUTERCONTROL_H

#include "basiccontroller.h"
#include "peripherals/buzzer.h"
#include "peripherals/helpers/osmerelbow.h"
#include "peripherals/helpers/pronosupination.h"
#include "peripherals/touch_bionics/touch_bionics_hand.h"
#include "ui/consolemenu.h"
#include "utils/settings.h"

class RemoteComputerControl : public BasicController {
public:
    explicit RemoteComputerControl();
    ~RemoteComputerControl();

private:
    Settings _settings;
    bool setup();
    void loop(double dt, double time);
    void cleanup();
    Buzzer _buzzer;
    PronoSupination& _pronosup;
    OsmerElbow& _osmer;
    TouchBionicsHand& _hand;
};

#endif // REMOTECOMPUTERCONTROL_H
