#include <iostream>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>

#ifndef MTIMER_HPP
#define MTIMER_HPP

class MTimer {
public:
    MTimer() : isRunning(false) {}

    bool start(int duration) {
        if (!isRunning) {
            isRunning = true;
            return wait(duration);
        }
        return true;
    }

    bool wait(int duration){
         std::unique_lock<std::mutex> lock(mutex);
         auto result =conditionVariable.wait_for(lock, std::chrono::seconds(duration), [this] { return !isRunning; });

         isRunning = false;
         return result;
    }
    
    void block(){
        if (!isRunning) {
            isRunning = true;
            std::unique_lock<std::mutex> lock(mutex);
            conditionVariable.wait(lock, [this] { return !isRunning; });
        }
    }

    void quit() {
        if (isRunning) {
            isRunning = false;
            conditionVariable.notify_all();
        }
    }

private:
    bool isRunning;
    std::condition_variable conditionVariable;
    std::mutex mutex;
};

#endif
