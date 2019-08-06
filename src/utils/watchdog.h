#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "control/threaded_loop.h"
#include <chrono>

class Watchdog : public ThreadedLoop {
public:
    Watchdog();
    Watchdog(
        std::chrono::seconds timeout_s, std::function<void(void)> on_timeout = [] {});
    ~Watchdog() override;

    void set_timeout(std::chrono::seconds timeout_s);
    void set_callback(std::function<void(void)> on_timeout);

    void ping();

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::chrono::seconds _timeout;
    std::chrono::steady_clock::time_point _last_ping;
    std::function<void(void)> _on_timeout;
};

#endif // WATCHDOG_H
