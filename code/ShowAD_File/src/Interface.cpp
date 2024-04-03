#include "UDPSocket.h"
#include "display.h"
#include "Function.hpp"
#include "sample_comm_ive.h"
#include <cstdio>
#include <hi_type.h>

using picData = std::pair<void*, uint32_t>;

static std::vector<picData> picBuffer;
static Display disp(picBuffer[0].first,picBuffer[0].second,false);
static UDPSocket sock("0.0.0.0",4000);
static bool flag_if_init_ = false;

static int InitPicBuffer(){
    void* picture;
    uint32_t length;
    for(int i =0; i <= 5; i++){
        picture = Function::readFileToBuffer("./data/"+std::to_string(i)+"_nv12.bin", length);
        if (!picture) {
            std::cout << "read bin error  " << std::endl;
            return -1;
        }
        picBuffer.emplace_back(picData(picture,length));
    }
    return 0;
}

static void DeInitPicBuffer(){

    for (auto& item : picBuffer){
        delete [] (char*)item.first;
        item.first = nullptr;
    }
    picBuffer.clear();
}

static void MsgProcess(Display& display ,const std::string& msg ){
    
       auto msglist =  Function::splitString(msg,':');
       if(msglist[0] == "Stop"){
            display.stopDisplay();
            display.ShowDefault();
       }
       else {
            display.stopDisplay();
            int dura,index;
            try {
                    index = std::stoi(msglist[1]);
                    dura = std::stoi(msglist[2]);
                } catch (...) {
                    display.ShowDefault();
                    return;
                }
            display.SendFrame(picBuffer[index].first, 1080, 1920, picBuffer[index].second, dura);
       }
}

hi_s32 InitShowAD(int flag_if_init){

    if (flag_if_init_) {
        printf("AD Show Model Can't Be Inited more than 1 times");
        return HI_FAILURE;
    }
    hi_s32 ret;
    ret = InitPicBuffer();
    SAMPLE_CHECK_EXPR_RET(ret < 0 ,HI_FAILURE,"Init AD Picture Failed!\n");
    printf("Init AD Picture Success!\n");

    disp.setflag(flag_if_init);
    ret = disp.Init();
    SAMPLE_CHECK_EXPR_RET(ret != HI_SUCCESS ,ret,"Init AD Display Model Failed!\n");
    printf("Init Display Model Success!\n");

    auto func = std::bind(MsgProcess, std::ref(disp),std::placeholders::_1);
    sock.setFunction(func);
    printf("Init UDP Socket Success!\n");

    flag_if_init_ = true;
    
    return HI_SUCCESS;
}

hi_s32 StartShowAD(){
  
    if (!flag_if_init_) {
        printf("AD Show Model need Init!\n");
        return HI_FAILURE;
    }
    sock.Start();
    hi_s32 ret = disp.ShowDefault();
    SAMPLE_CHECK_EXPR_RET(ret != HI_SUCCESS ,ret,"Error(%#x),Send Frame to Display Failed!\n",ret);
    return HI_SUCCESS;
}

void StopShowAD(){
    sock.setFunction(nullptr);
}

void DeInitShowAD(){
    if (!flag_if_init_) {
        printf("AD Show Model has not been inited\n");
        return;
    }
    disp.DeInit();
    DeInitPicBuffer();
    flag_if_init_ = false;
}


