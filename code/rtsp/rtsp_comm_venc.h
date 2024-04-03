// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-05 20:28
// Email        : 2650982628@qq.com
// Filename     : rtsp_comm_venc.h
// Description  : 
// ******************************************************

#ifndef RTSP_COMM_VENC_H
#define RTSP_COMM_VENC_H

#include "sample_comm.h"


HI_S32 RTSP_COMM_VENC_GetGopAttr(VENC_GOP_MODE_E enGopMode, VENC_GOP_ATTR_S *pstGopAttr);
HI_S32 RTSP_COMM_VENC_Create(VENC_CHN VencChn, PAYLOAD_TYPE_E enType, PIC_SIZE_E enSize, SAMPLE_RC_E enRcMode,HI_U32 u32Profile, HI_BOOL bRcnRefShareBuf, VENC_GOP_ATTR_S *pstGopAttr);

HI_S32 RTSP_COMM_VENC_Start(VENC_CHN VencChn, PAYLOAD_TYPE_E enType, PIC_SIZE_E enSize, SAMPLE_RC_E enRcMode,HI_U32 u32Profile, HI_BOOL bRcnRefShareBuf, VENC_GOP_ATTR_S *pstGopAttr);

HI_S32 RTSP_COMM_VENC_StartGetStream(VENC_CHN VeChn[], HI_S32 s32Cnt);

HI_S32 RTSP_COMM_VENC_Stop(VENC_CHN VencChn);
HI_S32 RTSP_COMM_VENC_StopGetStream(void);

#endif
