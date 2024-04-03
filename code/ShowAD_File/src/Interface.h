// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-14 22:40
// Email        : 2650982628@qq.com
// Filename     : Interface.h
// Description  : 
// ******************************************************

#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "sample_comm_ive.h"

extern hi_s32 InitShowAD(int flag_if_init);
extern hi_s32 StartShowAD();
extern void StopShowAD();
extern void DeInitShowAD();

#ifdef __cplusplus
}
#endif

#endif
