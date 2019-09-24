#include "threaded_loop.h"
#include <cmath>

ThreadedLoop::ThreadedLoop(std::string name, double period_s)
    : NamedObject(name)
    , MenuUser("", "", [this] { stop(); })
    , _period_s("period_ms", BaseParam::ReadWrite, this, period_s)
    , _pref_cpu("pref_cpu", BaseParam::ReadWrite, this, 1)
    , _prio("prio", BaseParam::ReadWrite, this, 20)
{
    _menu->add_item("start", "Start loop", [this](std::string) { start(); });
    _menu->add_item("stop", "Stop loop", [this](std::string) { stop_and_join(); });
}

ThreadedLoop::~ThreadedLoop()
{
    stop_and_join();
}

void ThreadedLoop::set_period(double seconds)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _period_s = seconds;
}

void ThreadedLoop::set_preferred_cpu(int cpu)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _pref_cpu = cpu;
}

void ThreadedLoop::set_prio(int prio)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _prio = prio;
}

void ThreadedLoop::start()
{
    if (!_thread.joinable())
        _thread = std::thread(&ThreadedLoop::run, this);
}

void ThreadedLoop::stop()
{
    _loop_condition = false;
}

void ThreadedLoop::stop_and_join()
{
    stop();
    if (_thread.joinable()) {
        _thread.join();
    }
}

bool ThreadedLoop::setup()
{
    return true;
}

void ThreadedLoop::cleanup()
{
}

void ThreadedLoop::_set_prio_internal(int prio)
{
    struct sched_param sp = {};
    sp.sched_priority = prio;
    sched_setscheduler(0, SCHED_RR, &sp);
}

void ThreadedLoop::_set_preferred_cpu_internal(int cpu)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &set);
}

void ThreadedLoop::run()
{
    pthread_setname_np(pthread_self(), _name.c_str());

    std::chrono::microseconds dt;

    _loop_condition = true;

    if (!setup()) {
        return;
    }

    _set_preferred_cpu_internal(_pref_cpu);
    _set_prio_internal(_prio);

    clock::time_point prev_period, next_period;
    prev_period = std::chrono::steady_clock::now();
    next_period = prev_period;

    while (true) {
        next_period += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(_period_s.to()));
        std::this_thread::sleep_until(next_period);

        dt = std::chrono::duration_cast<std::chrono::microseconds>(next_period - prev_period);

        loop(dt.count() / 1000000., clock::now());

        if (_pref_cpu.changed()) {
            _set_preferred_cpu_internal(_pref_cpu);
        }

        if (_prio.changed()) {
            _set_prio_internal(_prio);
        }

        if (!_loop_condition)
            break;
    }
    cleanup();
}
