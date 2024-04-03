// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-06-08 16:13
// Email        : 2650982628@qq.com
// Filename     : MThread.hpp
// Description  : 
// ******************************************************

#ifndef MTHREAD_H
#define MTHREAD_H

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <condition_variable>

// gettid 实现
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(__NR_gettid)

class MThread{
    public:
        MThread(){
            m_if_run  = false;
            m_if_exit = true;
            m_flag_if_stop = true;
            m_tID     = 0;
        }

        bool Start(void){
            if(m_if_run) {
                std::cout << "The thread is already running !" <<std::endl;
                return false;
            }
            if(!m_if_exit) {
                wait();
            }
            std::lock_guard<std::mutex> lock(m_mutex);
            m_if_exit = false;
            m_tID     = 0;
            std::thread thr(std::bind(&MThread::func,this));
            __mthread = std::move(thr);
            __mthread.detach();
            return true;
        }

        bool wait(std::chrono::milliseconds waitTime){
            std::unique_lock<std::mutex> lock(m_mutex);
            if(!m_if_run) {
                return true;
            }
            while (!m_if_exit) {
                if(m_cv.wait_for(lock, waitTime) == std::cv_status::timeout){
                    return false;
                }
            }
            return true;
        }

        void wait(){
            std::unique_lock<std::mutex> lock(m_mutex);
            if(!m_if_run) {
                return;
            }
            while (!m_if_exit) {
                m_cv.wait(lock);
            }
        }

        void func(void){
            try {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_if_run = true;
                    m_tID    = gettid();
                }

                run();
            }
            catch (...) {
                std::cerr << "Exception caught in thread " << m_tID << std::endl;
            }

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_if_run  = false;
                m_if_exit = true;
            }
            m_cv.notify_all();
        }

        virtual void run(void){
            usleep(1000*1000);
        }

        virtual void stop(){
            m_flag_if_stop = true;
            wait();
        }

        void quit(){
            if(m_if_exit) return;
            stop();
            // {
                // std::lock_guard<std::mutex> lock(m_mutex);
                // m_if_run  = false;
                // m_if_exit = true;
            // }
            // m_cv.notify_all();
        }

        virtual ~MThread(){
            quit();
            if(__mthread.joinable()) {
                __mthread.join();
            }
            std::cout << "----------Thread " << m_tID << " exiting----------" <<std::endl; 
        }


    protected:
        MThread(const MThread&) = delete;
        MThread& operator=(const MThread&) = delete;

        bool m_flag_if_stop;

    private:
        std::thread __mthread;
        bool m_if_run;
        bool m_if_exit;
        
        std::mutex m_mutex;
        std::condition_variable m_cv;

        unsigned long m_tID;
};

#endif

