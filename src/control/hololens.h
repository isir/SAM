#ifndef HOLOLENS_H
#define HOLOLENS_H

#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"

class Hololens : public ThreadedLoop
{
public:
    explicit Hololens(std::shared_ptr<SAM::Components> robot);
    ~Hololens() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    OscError sendIntMessage(int number, struct sockaddr addr);
    OscError sendOscContents(OscMessage oscContents, struct sockaddr addr);

    OscError processData(OscMessage oscMessage, struct sockaddr addr);

    std::shared_ptr<SAM::Components> _robot;
    Socket _socket;

    OscSlipDecoder _oscSlipDecoder;
    OscMessage _oscMessage;
    OscTimeTag _oscTimeTag;

    //interprocess_semaphore  data_access_mutex;
    std::mutex _mutex;
};

#endif // HOLOLENS_H
