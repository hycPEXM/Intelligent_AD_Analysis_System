/* ****************************************************** */
/* Author       : Joey Qiu */
/* Last modified: 2023-08-05 17:12 */
/* Email        : 2650982628@qq.com */
/* Filename     : nnie_rtsp.c */
/* Description  :  */
/* ****************************************************** */

#include <hi_type.h>
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "sample_comm.h"
#include "nnie_rtsp.h"
#include "rtsp_comm_venc.h"

#define STREAM_SIZE    PIC_2592x1944
#define CHN_NUM_MAX    1

static hi_bool g_rtsp_exit = HI_FALSE;

int NNIE_START_RTSP(void){

    HI_S32 ret;
    SIZE_S          stSize;
    PIC_SIZE_E      enSize = STREAM_SIZE;
    HI_S32          s32ChnNum  = 1;
    VENC_CHN        VencChn    = 0;
    HI_U32          u32Profile = 0;
    PAYLOAD_TYPE_E  enPayLoad  = PT_H264;
    VENC_GOP_MODE_E enGopMode = VENC_GOPMODE_NORMALP;
    VENC_GOP_ATTR_S stGopAttr;
    /* SAMPLE_RC_E     enRcMode = SAMPLE_RC_QVBR; */
    SAMPLE_RC_E     enRcMode = SAMPLE_RC_CBR;
    HI_BOOL         bRcnRefShareBuf = HI_FALSE;

    ret = RTSP_COMM_VENC_GetGopAttr(enGopMode, &stGopAttr);
    if (ret!= HI_SUCCESS) {
        SAMPLE_PRT("Venc Get GopAttr for %#x!\n", ret);
        return ret;
    }

    ret = RTSP_COMM_VENC_Start(VencChn, enPayLoad, enSize, enRcMode,u32Profile,bRcnRefShareBuf,&stGopAttr);
    if (HI_SUCCESS != ret)
    {
        SAMPLE_PRT("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    /******************************************
     venc stream process
    ******************************************/
    ret = RTSP_COMM_VENC_StartGetStream(&VencChn, s32ChnNum);
    if (ret !=  HI_SUCCESS) {
        SAMPLE_PRT("Start Venc failed!\n");
        goto EXIT_VENC_H264_STOP;
    }

    return HI_SUCCESS;

    EXIT_VENC_H264_STOP:
        RTSP_COMM_VENC_Stop(VencChn);

        return ret;

}


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
