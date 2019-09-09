#include "worker.h"

Worker::Worker(std::string thread_name)
    : _worker_loop_condition(true)
    , _worker_cv_check(false)
{
    auto f = [this, thread_name] {
        pthread_setname_np(pthread_self(), thread_name.c_str());
        std::unique_lock lock(_worker_mutex);

        while (true) {
            _worker_cv.wait(lock, [this] { return static_cast<bool>(_worker_cv_check); });
            _worker_cv_check = false;

            if (!_worker_loop_condition) {
                return;
            }

            work();
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
    _worker_cv_check = true;
    _worker_cv.notify_one();
}

void Worker::work()
{
}
