// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-14 20:54
// Email        : 2650982628@qq.com
// Filename     : ad_show.h
// Description  : 
// ******************************************************

#include<stdio.h>
#ifdef __cplusplus
extern "C"{
#endif

#include "sample_comm_ive.h"

hi_s32 InitSystyem(HI_U32 h,HI_U32 w);
hi_s32 BufferToFrm(const char* buffer, HI_U32 w, HI_U32 h);
hi_void DeInitSystem();

#ifdef __cplusplus
}
#endif
