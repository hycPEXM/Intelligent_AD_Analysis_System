// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-10 20:52
// Email        : 2650982628@qq.com
// Filename     : display.cpp
// Description  : 
// ******************************************************

#include "display.h"
#include <cstdint>
#include <cstdio>
#include <hi_type.h>
#include <iostream>
#include <thread>
#include <functional>
#include "ad_show.h"

Display::Display(std::string defalut,bool if_need_init):
    m_timer(),m_default(defalut),m_flag_if_need_init(if_need_init)
{
}

int Display::Init(){
    hi_s32 ret = HI_SUCCESS;
    if(m_flag_if_need_init){
         ret = InitSystyem(1080, 1920);
    }
    SAMPLE_CHECK_EXPR_RET(ret != HI_SUCCESS ,ret,"Error(%#x),Display Init Failed!\n",ret);
    return HI_SUCCESS;
}

void Display::DeInit(){
    if (m_flag_if_need_init) {
        DeInitSystem();
    }
}

int Display::SendFrame(std::string data, uint32_t h, uint32_t w, int duration){
	
    int ret;

    ret =  BufferToFrm(data.c_str(), w, h);
    SAMPLE_CHECK_EXPR_RET(ret != HI_SUCCESS ,ret,"Error(%#x),Send Frame to Display Failed!\n",ret);

    auto func = std::bind(&Display::wait_end,this,duration);
    std::thread thr(func);
    // m_thr = std::move(thr);
    thr.detach();
    return HI_SUCCESS;
}


void Display::wait_end(int duration){

    bool ret = true;
    if(duration == -1)
        m_timer.block();
    else
    {
       ret =  m_timer.start(duration);
    }
    if(!ret && !m_default.empty())
    {	
        ShowDefault();
    }

}

int Display::ShowDefault(){
    hi_s32 ret = BufferToFrm(m_default.c_str(), 1920, 1080);
    SAMPLE_CHECK_EXPR_RET(ret != HI_SUCCESS ,ret,"Error(%#x),Send Frame to Display Failed!\n",ret);
    return HI_SUCCESS;
}
