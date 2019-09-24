#include "worker.h"

Worker::Worker(std::string thread_name, RunType rt)
    : _worker_loop_condition(true)
    , _worker_cv_check(false)
    , _worker_runtype(rt)
{
    auto f = [this, thread_name] {
        pthread_setname_np(pthread_self(), thread_name.c_str());

        std::unique_lock lock(_worker_mutex);

        _worker_cv.wait(lock, [this] { return static_cast<bool>(_worker_cv_check); });
        _worker_cv_check = false;

        while (true) {
            if (!_worker_loop_condition) {
                return;
            }

            lock.unlock();

            work();

            lock.lock();

            if (_worker_runtype == OneShot) {
                _worker_cv.wait(lock, [this] { return static_cast<bool>(_worker_cv_check); });
                _worker_cv_check = false;
            }
        }
    };

    _worker_thread = std::thread(f);
}

Worker::~Worker()
{
    stop();
}

void Worker::stop()
{
    _worker_loop_condition = false;
    if (_worker_thread.joinable()) {
        _worker_cv_check = true;
        _worker_cv.notify_one();
        _worker_thread.join();
    }
}

void Worker::do_work()
{
    std::unique_lock lock(_worker_mutex);
    _worker_cv_check = true;
    _worker_cv.notify_one();
}

void Worker::work()
{
}
