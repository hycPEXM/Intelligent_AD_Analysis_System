// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-10 20:40
// Email        : 2650982628@qq.com
// Filename     : display.h
// Description  : 
// ******************************************************

#ifndef DISPLAY_H
#define DISPLAY_H

#include <cstdint>
#include "MTimer.hpp"
class Display
{
private:
   MTimer m_timer; 
   std::string m_default;
   bool m_flag_if_need_init;

public:
    Display(std::string defalut_pic = nullptr,  bool if_need_init =false);
    int Init();
    void DeInit();
    int SendFrame(std::string data,uint32_t h,uint32_t w, int duration);
    void stopDisplay(){ m_timer.quit(); }
    int ShowDefault();
    void setflag(int flag){
        if(flag == 0)
            m_flag_if_need_init = false;
        else
            m_flag_if_need_init = true;
    }
    virtual ~Display() = default;

private:
    void wait_end(int duration);
};

#endif /* DISPLAY_H */

