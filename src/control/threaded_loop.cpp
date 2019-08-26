#include "threaded_loop.h"
#include <QDebug>
#include <QMutexLocker>
#include <QtGlobal>

static double timespec_diff(struct timespec* start, struct timespec* stop)
{
    struct timespec result;
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result.tv_sec = stop->tv_sec - start->tv_sec - 1;
        result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result.tv_sec = stop->tv_sec - start->tv_sec;
        result.tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    return result.tv_sec + (result.tv_nsec * 1e-9);
}

ThreadedLoop::ThreadedLoop(QString name, double period_s)
    : NamedObject(name)
    , _period_s("period_s", this, period_s)
    , _pref_cpu("pref_cpu", this, 0)
    , _prio("prio", this, 20)
{
    QObject::connect(_menu.get(), &MenuBackend::finished, this, &ThreadedLoop::stop);
    _menu->add_item("start", "Start loop", [this](QString) { this->start(); });
    _menu->add_item("stop", "Stop loop", [this](QString) { this->stop(); });
}

ThreadedLoop::~ThreadedLoop()
{
    stop();
}

void ThreadedLoop::set_period(double seconds)
{
    QMutexLocker locker(&_mutex);
    _period_s = seconds;
}

void ThreadedLoop::set_preferred_cpu(int cpu)
{
    QMutexLocker locker(&_mutex);
    _pref_cpu = cpu;
}

void ThreadedLoop::set_prio(int prio)
{
    QMutexLocker locker(&_mutex);
    _prio = prio;
}

void ThreadedLoop::enable_watchdog(int timeout_ms)
{
    QObject::disconnect(this, &ThreadedLoop::ping, &_watchdog_timer, qOverload<>(&QTimer::start));
    QObject::disconnect(&_watchdog_timer, &QTimer::timeout, this, &ThreadedLoop::unresponsive_callback);

    if (timeout_ms > 0) {
        _watchdog_timer.setInterval(timeout_ms);
        _watchdog_timer.setSingleShot(true);
        QObject::connect(this, &ThreadedLoop::ping, &_watchdog_timer, qOverload<>(&QTimer::start));
        QObject::connect(&_watchdog_timer, &QTimer::timeout, this, &ThreadedLoop::unresponsive_callback);
    }
}

void ThreadedLoop::stop()
{
    if (isRunning()) {
        QMutexLocker locker(&_mutex);
        _loop_condition = false;
    }
}

void ThreadedLoop::unresponsive_callback()
{
    terminate();
    qCritical() << "Watchdog timed out, current thread was terminated.";
}

void ThreadedLoop::run()
{
    double dt = 0;
    unsigned int cnt = 0;
    double min = std::numeric_limits<double>::max(), max = 0, avg = 0;

    _loop_condition = true;

    emit ping();

    if (!setup()) {
        return;
    }

    struct timespec prev_period, next_period;
    clock_gettime(CLOCK_MONOTONIC, &prev_period);
    next_period = prev_period;

    while (true) {
        long period_ns = qRound(_period_s * 1e9);
        next_period.tv_nsec += period_ns;
        while (next_period.tv_nsec >= 1e9) {
            next_period.tv_sec++;
            next_period.tv_nsec -= 1e9;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, nullptr);
        dt = timespec_diff(&prev_period, &next_period);
        clock_gettime(CLOCK_MONOTONIC, &prev_period);

        loop(dt, prev_period.tv_sec + (prev_period.tv_nsec * 1e-9));

        emit ping();

        ++cnt;
        if (dt < min)
            min = dt;
        else if (dt > max)
            max = dt;
        avg = (avg * (cnt - 1) + dt) / cnt;

        QMutexLocker locker(&_mutex);

        if (_pref_cpu.changed()) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(_pref_cpu, &set);
            pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &set);
        }

        if (_prio.changed()) {
            struct sched_param sp = {};
            sp.sched_priority = _prio;
            sched_setscheduler(0, SCHED_RR, &sp);
        }

        if (!_loop_condition)
            break;
    }
    cleanup();
    enable_watchdog(-1);
    qInfo() << name() << "- Thread finished - Avg loop time = " << avg * 1000. << "ms, min = " << min * 1000. << "ms, max = " << max * 1000. << "ms";
}
