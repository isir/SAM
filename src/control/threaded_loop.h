#ifndef THREADED_LOOP_H
#define THREADED_LOOP_H

#include "ui/menu_user.h"
#include "utils/named_object.h"
#include "utils/param.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class ThreadedLoop : public NamedObject, public MenuUser {
public:
    ThreadedLoop(std::string name, double period_s = 1);
    virtual ~ThreadedLoop();

    void set_period(double seconds);
    void set_preferred_cpu(int cpu);
    void set_prio(int prio);
    double period() { return _period_s; }

    void start();
    void stop();
    void stop_and_join();

protected:
    using clock = std::chrono::steady_clock;

    virtual bool setup();
    virtual void loop(double dt, clock::time_point time) = 0;
    virtual void cleanup() = 0;

    void _set_preferred_cpu_internal(int cpu);
    void _set_prio_internal(int prio);

    std::atomic<bool> _loop_condition;
    std::mutex _mutex;
    std::thread _thread;

private:
    void run();

    Param<double> _period_s;
    Param<int> _pref_cpu;
    Param<int> _prio;
};

#endif // THREADED_LOOP_H
