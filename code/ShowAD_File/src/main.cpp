// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-07-24 22:14
// Email        : 2650982628@qq.com
// Filename     : main.cpp
// Description  : 
// ******************************************************

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include "MTimer.hpp"
#include "display.h"
#include "UDPSocket.h"
#include "Function.hpp"

using picData = std::string;

static std::vector<picData> picBuffer;


static int InitPicBuffer(){
    
    std::string picture;
    uint32_t length;
    for(int i =0; i <= 5; i++){
        picture = "./data/"+std::to_string(i)+"_nv12.bin";

        picBuffer.emplace_back(picture);
    }
    return 0;
}

static void DeInitPicBuffer(){
    picBuffer.clear();
}

void MsgProcess(Display& disp ,const std::string& msg ){
    
       auto msglist =  Function::splitString(msg,':');
       if(msglist[0] == "Stop"){
            disp.stopDisplay();
            disp.ShowDefault();
       }
       else {
            disp.stopDisplay();
            int dura,index;
            try {
                    index = std::stoi(msglist[1]);
                    dura = std::stoi(msglist[2]);
                } catch (...) {
                    disp.ShowDefault();
                    return;
                }
            disp.SendFrame(picBuffer[index], 1080, 1920, dura);
       }
}


int main(int argc, char *argv[])
{
    InitPicBuffer();
    Display disp(picBuffer[0],true);
    std::cout << "create class success" << std::endl;
    int ret = disp.Init();
    if (ret < 0) {
        std::cout << "init Failed" << std::endl;
        return -1;
    }
    std::cout << "init Display success" << std::endl;

    UDPSocket sock("0.0.0.0",3000);
    auto func = std::bind(MsgProcess, std::ref(disp),std::placeholders::_1);
    sock.setFunction(func);
    sock.Start();

    disp.ShowDefault();

    while (true) {
        if(Function::kbhit()&&(fgetc(stdin) == 'q')){
            std::cout << std::endl;
            disp.DeInit();
            break;
        }
        usleep(50*1000);
    }
    DeInitPicBuffer();
    return 0;
}


