#ifndef WORKER_H
#define WORKER_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

class Worker {
public:
    enum RunType {
        Continuous,
        OneShot
    };

    Worker(std::string thread_name = std::string(), RunType rt = OneShot);
    virtual ~Worker();

    void stop();
    void do_work();

protected:
    virtual void work();

    std::atomic<bool> _worker_loop_condition;

private:
    std::atomic<bool> _worker_cv_check;
    std::condition_variable _worker_cv;
    std::mutex _worker_mutex;
    std::thread _worker_thread;
    RunType _worker_runtype;
};

#endif // WORKER_H
