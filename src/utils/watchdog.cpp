#include "watchdog.h"

Watchdog::Watchdog()
    : ThreadedLoop("watchdog", 0.1)
{
}

Watchdog::Watchdog(std::chrono::seconds timeout_s, std::function<void(void)> on_timeout)
    : ThreadedLoop("watchdog", 0.1)
    , _timeout(timeout_s)
    , _on_timeout(on_timeout)
{
}

Watchdog::~Watchdog()
{
    stop_and_join();
}

void Watchdog::set_timeout(std::chrono::seconds timeout_s)
{
    std::lock_guard lock(_mutex);

    _timeout = timeout_s;
}

void Watchdog::set_callback(std::function<void()> on_timeout)
{
    std::lock_guard lock(_mutex);

    _on_timeout = on_timeout;
}

void Watchdog::ping()
{
    std::lock_guard lock(_mutex);

    if (_last_ping.time_since_epoch() == std::chrono::seconds(0)) {
        start();
    }
    _last_ping = clock::now();
}

bool Watchdog::setup()
{
    return true;
}

void Watchdog::loop(double, clock::time_point)
{
    if ((clock::now() - _last_ping) > _timeout) {
        _on_timeout();
        stop();
    }
}

void Watchdog::cleanup()
{
}
