/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <hi_comm_venc.h>
#include <hi_type.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>
#include <limits.h>
#include "sample_comm.h"
#include "autoconf.h"
#include "rtsp_comm_venc.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#define TEMP_BUF_LEN    8
#define PER_UNIT        1024
#define MAX_THM_SIZE    (64 * 1024)

static pthread_t gs_VencPid;
static pthread_t gs_VencQpmapPid;
static SAMPLE_VENC_GETSTREAM_PARA_S gs_stPara;
static SAMPLE_VENC_QPMAP_SENDFRAME_PARA_S stQpMapSendFramePara;

#include "ffmpeg_rtsp.h"

#define rtsp_path "rtsp://0.0.0.0:8554/cam"
static int g_ffmpeg_if_init = -1;
static uint64_t count = 0;

static hi_s32 ffmpeg_init_rtsp(){

    if (g_ffmpeg_if_init > 0)
        return HI_SUCCESS;
    int ret = init_rtsp_stream(rtsp_path,1944,2592,30);
    if (ret < 0) {
        return HI_FAILURE;
    }
    g_ffmpeg_if_init = 1;
    return HI_SUCCESS;
}                                                                                                                                                                                                                                   

static hi_s32
ffmpeg_send_frame(VENC_STREAM_S *stream)
{
    if(g_ffmpeg_if_init < 0) {
        return HI_FAILURE;
    }

    for (hi_u32 i = 0; i < stream->u32PackCount; i++) {
        hi_s32 ret = send_packet_to_server(stream->pstPack[i].pu8Addr + stream->pstPack[i].u32Offset,stream->pstPack[i].u32Len - stream->pstPack[i].u32Offset,
                                           stream->stH264Info.bPSkip,count);
        count ++;
        if(ret < 0) {
            return HI_FAILURE;
        }
    }

    return HI_SUCCESS;
}

static hi_void ffmpeg_deinit_rtsp(){
    if (g_ffmpeg_if_init < 0) {
        return;
    }
    deinit_rtsp_stream();
    g_ffmpeg_if_init = -1;
    count = 0;
}



HI_S32 RTSP_COMM_VENC_MemConfig(HI_VOID)
{
    HI_S32 i = 0;
    HI_S32 s32Ret;
    HI_CHAR *pcMmzName = HI_NULL;
    MPP_CHN_S stMppChnVENC;

    for (i = 0; i < 64; i++) { /* group, venc max chn is 64 */
        stMppChnVENC.enModId = HI_ID_VENC;
        stMppChnVENC.s32DevId = 0;
        stMppChnVENC.s32ChnId = i;
        pcMmzName = NULL;

        s32Ret = HI_MPI_SYS_SetMemConfig(&stMppChnVENC, pcMmzName);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("HI_MPI_SYS_SetMemConfig with %#x!\n", s32Ret);
            return HI_FAILURE;
        }
    }
    return HI_SUCCESS;
}

static HI_S32 RTSP_COMM_VENC_GetFilePostfix(PAYLOAD_TYPE_E enPayload, HI_CHAR *szFilePostfix, HI_U8 len)
{
    if (szFilePostfix == NULL) {
        SAMPLE_PRT("null pointer\n");
        return HI_FAILURE;
    }

    if (PT_H264 == enPayload) {
        if (strcpy_s(szFilePostfix,len,".h264") != EOK) {
            return HI_FAILURE;
        }
    } else if (PT_H265 == enPayload) {
        if (strcpy_s(szFilePostfix, len, ".h265") != EOK) {
            return HI_FAILURE;
        }
    } else if (PT_JPEG == enPayload) {
        if (strcpy_s(szFilePostfix, len, ".jpg") != EOK) {
            return HI_FAILURE;
        }
    } else if (PT_MJPEG == enPayload) {
        if (strcpy_s(szFilePostfix, len, ".mjp") != EOK) {
            return HI_FAILURE;
        }
    } else {
        SAMPLE_PRT("payload type err!\n");
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

HI_S32 RTSP_COMM_VENC_GetGopAttr(VENC_GOP_MODE_E enGopMode, VENC_GOP_ATTR_S *pstGopAttr)
{
    if (pstGopAttr == NULL) {
        SAMPLE_PRT("null pointer\n");
        return HI_FAILURE;
    }
    switch (enGopMode) {
        case VENC_GOPMODE_NORMALP:
            pstGopAttr->enGopMode = VENC_GOPMODE_NORMALP;
            pstGopAttr->stNormalP.s32IPQpDelta = 2; /* set 2 */
            break;
        case VENC_GOPMODE_SMARTP:
            pstGopAttr->enGopMode = VENC_GOPMODE_SMARTP;
            pstGopAttr->stSmartP.s32BgQpDelta = 4; /* set 4 */
            pstGopAttr->stSmartP.s32ViQpDelta = 2; /* set 2 */
            pstGopAttr->stSmartP.u32BgInterval = 90; /* set 90 */
            break;

        case VENC_GOPMODE_DUALP:
            pstGopAttr->enGopMode = VENC_GOPMODE_DUALP;
            pstGopAttr->stDualP.s32IPQpDelta = 4; /* set 4 */
            pstGopAttr->stDualP.s32SPQpDelta = 2; /* set 2 */
            pstGopAttr->stDualP.u32SPInterval = 3; /* set 3 */
            break;

        case VENC_GOPMODE_BIPREDB:
            pstGopAttr->enGopMode = VENC_GOPMODE_BIPREDB;
            pstGopAttr->stBipredB.s32BQpDelta = -2; /* set -2 */
            pstGopAttr->stBipredB.s32IPQpDelta = 3; /* set 3 */
            pstGopAttr->stBipredB.u32BFrmNum = 2; /* set 2 */
            break;

        default:
            SAMPLE_PRT("not support the gop mode !\n");
            return HI_FAILURE;
            break;
    }
    return HI_SUCCESS;
}

HI_S32 RTSP_COMM_VENC_SaveStream(FILE *pFd, VENC_STREAM_S *pstStream)
{
    HI_U32 i;

    if ((pFd == NULL) || (pstStream == NULL)) {
        SAMPLE_PRT("null pointer\n");
        return HI_FAILURE;
    }
    for (i = 0; i < pstStream->u32PackCount; i++) {
        (HI_VOID)fwrite(pstStream->pstPack[i].pu8Addr + pstStream->pstPack[i].u32Offset,
            pstStream->pstPack[i].u32Len - pstStream->pstPack[i].u32Offset, 1, pFd);

        (HI_VOID)fflush(pFd);
    }

    return HI_SUCCESS;
}

static HI_S32 RTSP_COMM_VENC_CloseReEncode(VENC_CHN VencChn)
{
    HI_S32 s32Ret;
    VENC_RC_PARAM_S stRcParam;
    VENC_CHN_ATTR_S stChnAttr;

    s32Ret = HI_MPI_VENC_GetChnAttr(VencChn, &stChnAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("GetChnAttr failed!\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_VENC_GetRcParam(VencChn, &stRcParam);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("GetRcParam failed!\n");
        return HI_FAILURE;
    }

    if (VENC_RC_MODE_H264CBR == stChnAttr.stRcAttr.enRcMode) {
        stRcParam.stParamH264Cbr.s32MaxReEncodeTimes = 0;
    } else if (VENC_RC_MODE_H264VBR == stChnAttr.stRcAttr.enRcMode) {
        stRcParam.stParamH264Vbr.s32MaxReEncodeTimes = 0;
    } else if (VENC_RC_MODE_H265CBR == stChnAttr.stRcAttr.enRcMode) {
        stRcParam.stParamH265Cbr.s32MaxReEncodeTimes = 0;
    } else if (VENC_RC_MODE_H265VBR == stChnAttr.stRcAttr.enRcMode) {
        stRcParam.stParamH265Vbr.s32MaxReEncodeTimes = 0;
    } else {
        return HI_SUCCESS;
    }
    s32Ret = HI_MPI_VENC_SetRcParam(VencChn, &stRcParam);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("SetRcParam failed!\n");
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

HI_S32 RTSP_COMM_VENC_Create(VENC_CHN VencChn, PAYLOAD_TYPE_E enType, PIC_SIZE_E enSize, SAMPLE_RC_E enRcMode,
    HI_U32 u32Profile, HI_BOOL bRcnRefShareBuf, VENC_GOP_ATTR_S *pstGopAttr)
{
    HI_S32 s32Ret;
    SIZE_S stPicSize;
    VENC_CHN_ATTR_S stVencChnAttr;
    VENC_ATTR_JPEG_S stJpegAttr;
    SAMPLE_VI_CONFIG_S stViConfig;
    HI_U32 u32FrameRate;
    HI_U32 u32StatTime;
    HI_U32 u32Gop = 10; /* default set 30 */
    u32FrameRate = 30; /* just test */

    if (pstGopAttr == NULL) {
        SAMPLE_PRT("pstGopAttr is null!\n");
        return HI_FAILURE;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enSize, &stPicSize);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("Get picture size failed!\n");
        return HI_FAILURE;
    }

    // get framerate by sensor 
    /* SAMPLE_COMM_VI_GetSensorInfo(&stViConfig);
    if (stViConfig.astViInfo[0].stSnsInfo.enSnsType == SAMPLE_SNS_TYPE_BUTT) {
        SAMPLE_PRT("Not set SENSOR%d_TYPE !\n", 0);
        return HI_FAILURE;
    }
    s32Ret = SAMPLE_COMM_VI_GetFrameRateBySensor(stViConfig.astViInfo[0].stSnsInfo.enSnsType, &u32FrameRate);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("SAMPLE_COMM_VI_GetFrameRateBySensor failed!\n");
        return s32Ret;
    } */

    /* step 1:  Create Venc Channel */
    stVencChnAttr.stVencAttr.enType = enType;
    stVencChnAttr.stVencAttr.u32MaxPicWidth = stPicSize.u32Width;
    stVencChnAttr.stVencAttr.u32MaxPicHeight = stPicSize.u32Height;
    stVencChnAttr.stVencAttr.u32PicWidth = stPicSize.u32Width;   /* the picture width */
    stVencChnAttr.stVencAttr.u32PicHeight = stPicSize.u32Height; /* the picture height */

    if (enType == PT_MJPEG || enType == PT_JPEG) {
        stVencChnAttr.stVencAttr.u32BufSize =
            HI_ALIGN_UP(stPicSize.u32Width, 16) * HI_ALIGN_UP(stPicSize.u32Height, 16); /* 16 align */
    } else {
        stVencChnAttr.stVencAttr.u32BufSize =
            HI_ALIGN_UP(stPicSize.u32Width * stPicSize.u32Height * 3 / 4, 64); /* * 3 / 4, 64 align */
    }
    stVencChnAttr.stVencAttr.u32Profile = u32Profile;
    stVencChnAttr.stVencAttr.bByFrame = HI_TRUE; /* get stream mode is slice mode or frame mode? */

    if (pstGopAttr->enGopMode == VENC_GOPMODE_SMARTP) {
        u32StatTime = pstGopAttr->stSmartP.u32BgInterval / u32Gop;
    } else {
        u32StatTime = 1;
    }

    switch (enType) {
        case PT_H265: {
            if (enRcMode == SAMPLE_RC_CBR) {
                VENC_H265_CBR_S stH265Cbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
                stH265Cbr.u32Gop = u32Gop;
                stH265Cbr.u32StatTime = u32StatTime;       /* stream rate statics time(s) */
                stH265Cbr.u32SrcFrameRate = u32FrameRate;  /* input (vi) frame rate */
                stH265Cbr.fr32DstFrameRate = u32FrameRate; /* target frame rate */
                switch (enSize) {
                    case PIC_720P:
                        stH265Cbr.u32BitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH265Cbr.u32BitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH265Cbr.u32BitRate = PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH265Cbr.u32BitRate = PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH265Cbr.u32BitRate = PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265Cbr, sizeof(VENC_H265_CBR_S), &stH265Cbr,
                    sizeof(VENC_H265_CBR_S));
            } else if (enRcMode == SAMPLE_RC_FIXQP) {
                VENC_H265_FIXQP_S stH265FixQp;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265FIXQP;
                stH265FixQp.u32Gop = u32Gop;
                stH265FixQp.u32SrcFrameRate = u32FrameRate;
                stH265FixQp.fr32DstFrameRate = u32FrameRate;
                stH265FixQp.u32IQp = 25; /* set 25 */
                stH265FixQp.u32PQp = 30; /* set 30 */
                stH265FixQp.u32BQp = 32; /* set 32 */
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265FixQp, sizeof(VENC_H265_FIXQP_S), &stH265FixQp,
                    sizeof(VENC_H265_FIXQP_S));
            } else if (enRcMode == SAMPLE_RC_VBR) {
                VENC_H265_VBR_S stH265Vbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
                stH265Vbr.u32Gop = u32Gop;
                stH265Vbr.u32StatTime = u32StatTime;
                stH265Vbr.u32SrcFrameRate = u32FrameRate;
                stH265Vbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_720P:
                        stH265Vbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH265Vbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH265Vbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH265Vbr.u32MaxBitRate = PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH265Vbr.u32MaxBitRate = PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265Vbr, sizeof(VENC_H265_VBR_S), &stH265Vbr,
                    sizeof(VENC_H265_VBR_S));
            } else if (enRcMode == SAMPLE_RC_AVBR) {
                VENC_H265_AVBR_S stH265AVbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265AVBR;
                stH265AVbr.u32Gop = u32Gop;
                stH265AVbr.u32StatTime = u32StatTime;
                stH265AVbr.u32SrcFrameRate = u32FrameRate;
                stH265AVbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_720P:
                        stH265AVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH265AVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH265AVbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH265AVbr.u32MaxBitRate = PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH265AVbr.u32MaxBitRate = PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265AVbr, sizeof(VENC_H265_AVBR_S), &stH265AVbr,
                    sizeof(VENC_H265_AVBR_S));
            } else if (enRcMode == SAMPLE_RC_QVBR) {
                VENC_H265_QVBR_S stH265QVbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265QVBR;
                stH265QVbr.u32Gop = u32Gop;
                stH265QVbr.u32StatTime = u32StatTime;
                stH265QVbr.u32SrcFrameRate = u32FrameRate;
                stH265QVbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_720P:
                        stH265QVbr.u32TargetBitRate =
                            PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH265QVbr.u32TargetBitRate =
                            PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH265QVbr.u32TargetBitRate =
                            PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH265QVbr.u32TargetBitRate =
                            PER_UNIT * 5 + PER_UNIT * 5  * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH265QVbr.u32TargetBitRate =
                            PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265QVbr, sizeof(VENC_H265_QVBR_S), &stH265QVbr,
                    sizeof(VENC_H265_QVBR_S));
            } else if (enRcMode == SAMPLE_RC_CVBR) {
                VENC_H265_CVBR_S stH265CVbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CVBR;
                stH265CVbr.u32Gop = u32Gop;
                stH265CVbr.u32StatTime = u32StatTime;
                stH265CVbr.u32SrcFrameRate = u32FrameRate;
                stH265CVbr.fr32DstFrameRate = u32FrameRate;
                stH265CVbr.u32LongTermStatTime = 1;
                stH265CVbr.u32ShortTermStatTime = u32StatTime;
                switch (enSize) {
                    case PIC_720P:
                        stH265CVbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * u32FrameRate / FPS_30; /* 3M + 1M */
                        stH265CVbr.u32LongTermMaxBitrate =
                            PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        stH265CVbr.u32LongTermMinBitrate = 512; /* 512kbps */
                        break;
                    case PIC_1080P:
                        stH265CVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        stH265CVbr.u32LongTermMaxBitrate =
                            PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        stH265CVbr.u32LongTermMinBitrate = PER_UNIT;
                        break;
                    case PIC_2592x1944:
                        stH265CVbr.u32MaxBitRate = PER_UNIT * 4 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 4M + 3M */
                        stH265CVbr.u32LongTermMaxBitrate =
                            PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        stH265CVbr.u32LongTermMinBitrate = PER_UNIT * 2; /* 2M */
                        break;
                    case PIC_3840x2160:
                        stH265CVbr.u32MaxBitRate = PER_UNIT * 8 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 8M + 5M */
                        stH265CVbr.u32LongTermMaxBitrate =
                            PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        stH265CVbr.u32LongTermMinBitrate = PER_UNIT * 3; /* 3M */
                        break;
                    default:
                        stH265CVbr.u32MaxBitRate = PER_UNIT * 24 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 24M + 5M */
                        stH265CVbr.u32LongTermMaxBitrate =
                            PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        stH265CVbr.u32LongTermMinBitrate = PER_UNIT * 5; /* 5M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265CVbr, sizeof(VENC_H265_CVBR_S), &stH265CVbr,
                    sizeof(VENC_H265_CVBR_S));
            } else if (enRcMode == SAMPLE_RC_QPMAP) {
                VENC_H265_QPMAP_S stH265QpMap;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265QPMAP;
                stH265QpMap.u32Gop = u32Gop;
                stH265QpMap.u32StatTime = u32StatTime;
                stH265QpMap.u32SrcFrameRate = u32FrameRate;
                stH265QpMap.fr32DstFrameRate = u32FrameRate;
                stH265QpMap.enQpMapMode = VENC_RC_QPMAP_MODE_MEANQP;
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH265QpMap, sizeof(VENC_H265_QPMAP_S), &stH265QpMap,
                    sizeof(VENC_H265_QPMAP_S));
            } else {
                SAMPLE_PRT("%s,%d,enRcMode(%d) not support\n", __FUNCTION__, __LINE__, enRcMode);
                return HI_FAILURE;
            }
            stVencChnAttr.stVencAttr.stAttrH265e.bRcnRefShareBuf = bRcnRefShareBuf;
            break;
        }
        case PT_H264: {
            if (enRcMode == SAMPLE_RC_CBR) {
                VENC_H264_CBR_S stH264Cbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
                stH264Cbr.u32Gop = u32Gop;                 /* the interval of IFrame */
                stH264Cbr.u32StatTime = u32StatTime;       /* stream rate statics time(s) */
                stH264Cbr.u32SrcFrameRate = u32FrameRate;  /* input (vi) frame rate */
                stH264Cbr.fr32DstFrameRate = u32FrameRate; /* target frame rate */
                switch (enSize) {
                    case PIC_720P:
                        stH264Cbr.u32BitRate = PER_UNIT * 3 + PER_UNIT * u32FrameRate / FPS_30; /* 3M + 1M */
                        break;
                    case PIC_1080P:
                        stH264Cbr.u32BitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH264Cbr.u32BitRate = PER_UNIT * 4 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 4M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH264Cbr.u32BitRate = PER_UNIT * 8 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 8M + 5M */
                        break;
                    default:
                        stH264Cbr.u32BitRate = PER_UNIT * 24 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 24M + 5M */
                        break;
                }

                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264Cbr, sizeof(VENC_H264_CBR_S), &stH264Cbr,
                    sizeof(VENC_H264_CBR_S));
            } else if (enRcMode == SAMPLE_RC_FIXQP) {
                VENC_H264_FIXQP_S stH264FixQp;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264FIXQP;
                stH264FixQp.u32Gop = u32Gop;
                stH264FixQp.u32SrcFrameRate = u32FrameRate;
                stH264FixQp.fr32DstFrameRate = u32FrameRate;
                stH264FixQp.u32IQp = 25; /* set 25 */
                stH264FixQp.u32PQp = 30; /* set 30 */
                stH264FixQp.u32BQp = 32; /* set 32 */
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264FixQp, sizeof(VENC_H264_FIXQP_S), &stH264FixQp,
                    sizeof(VENC_H264_FIXQP_S));
            } else if (enRcMode == SAMPLE_RC_VBR) {
                VENC_H264_VBR_S stH264Vbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
                stH264Vbr.u32Gop = u32Gop;
                stH264Vbr.u32StatTime = u32StatTime;
                stH264Vbr.u32SrcFrameRate = u32FrameRate;
                stH264Vbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_360P:
                        stH264Vbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_720P:
                        stH264Vbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH264Vbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH264Vbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH264Vbr.u32MaxBitRate = PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH264Vbr.u32MaxBitRate = PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264Vbr, sizeof(VENC_H264_VBR_S), &stH264Vbr,
                    sizeof(VENC_H264_VBR_S));
            } else if (enRcMode == SAMPLE_RC_AVBR) {
                VENC_H264_VBR_S stH264AVbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264AVBR;
                stH264AVbr.u32Gop = u32Gop;
                stH264AVbr.u32StatTime = u32StatTime;
                stH264AVbr.u32SrcFrameRate = u32FrameRate;
                stH264AVbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_360P:
                        stH264AVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_720P:
                        stH264AVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH264AVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH264AVbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH264AVbr.u32MaxBitRate = PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH264AVbr.u32MaxBitRate = PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264AVbr, sizeof(VENC_H264_AVBR_S), &stH264AVbr,
                    sizeof(VENC_H264_AVBR_S));
            } else if (enRcMode == SAMPLE_RC_QVBR) {
                VENC_H264_QVBR_S stH264QVbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264QVBR;
                stH264QVbr.u32Gop = u32Gop;
                stH264QVbr.u32StatTime = u32StatTime;
                stH264QVbr.u32SrcFrameRate = u32FrameRate;
                stH264QVbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_360P:
                        stH264QVbr.u32TargetBitRate =
                            PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_720P:
                        stH264QVbr.u32TargetBitRate =
                            PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30; /* 2M + 1M */
                        break;
                    case PIC_1080P:
                        stH264QVbr.u32TargetBitRate =
                            PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        break;
                    case PIC_2592x1944:
                        stH264QVbr.u32TargetBitRate =
                            PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 3M + 3M */
                        break;
                    case PIC_3840x2160:
                        stH264QVbr.u32TargetBitRate =
                            PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 5M + 5M */
                        break;
                    default:
                        stH264QVbr.u32TargetBitRate =
                            PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264QVbr, sizeof(VENC_H264_QVBR_S), &stH264QVbr,
                    sizeof(VENC_H264_QVBR_S));
            } else if (enRcMode == SAMPLE_RC_CVBR) {
                VENC_H264_CVBR_S stH264CVbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CVBR;
                stH264CVbr.u32Gop = u32Gop;
                stH264CVbr.u32StatTime = u32StatTime;
                stH264CVbr.u32SrcFrameRate = u32FrameRate;
                stH264CVbr.fr32DstFrameRate = u32FrameRate;
                stH264CVbr.u32LongTermStatTime = 1;
                stH264CVbr.u32ShortTermStatTime = u32StatTime;
                switch (enSize) {
                    case PIC_720P:
                        stH264CVbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * u32FrameRate / FPS_30; /* 3M + 2M */
                        stH264CVbr.u32LongTermMaxBitrate = PER_UNIT * 2 + PER_UNIT * u32FrameRate / FPS_30;
                        stH264CVbr.u32LongTermMinBitrate = 512; /* 512kbps */
                        break;
                    case PIC_1080P:
                        stH264CVbr.u32MaxBitRate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 2M + 2M */
                        stH264CVbr.u32LongTermMaxBitrate = PER_UNIT * 2 + PER_UNIT * 2 * u32FrameRate / FPS_30;
                        stH264CVbr.u32LongTermMinBitrate = PER_UNIT;
                        break;
                    case PIC_2592x1944:
                        stH264CVbr.u32MaxBitRate = PER_UNIT * 4 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 4M + 3M */
                        stH264CVbr.u32LongTermMaxBitrate = PER_UNIT * 3 + PER_UNIT * 3 * u32FrameRate / FPS_30;
                        stH264CVbr.u32LongTermMinBitrate = PER_UNIT * 2; /* 2M */
                        break;
                    case PIC_3840x2160:
                        stH264CVbr.u32MaxBitRate = PER_UNIT * 8 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 8M + 5M */
                        stH264CVbr.u32LongTermMaxBitrate = PER_UNIT * 5 + PER_UNIT * 5 * u32FrameRate / FPS_30;
                        stH264CVbr.u32LongTermMinBitrate = PER_UNIT * 3; /* 3M */
                        break;
                    default:
                        stH264CVbr.u32MaxBitRate = PER_UNIT * 24 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 24M + 5M */
                        stH264CVbr.u32LongTermMaxBitrate =
                            PER_UNIT * 15 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 15M + 2M */
                        stH264CVbr.u32LongTermMinBitrate = PER_UNIT * 5; /* 5M */
                        break;
                }
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264CVbr, sizeof(VENC_H264_CVBR_S), &stH264CVbr,
                    sizeof(VENC_H264_CVBR_S));
            } else if (enRcMode == SAMPLE_RC_QPMAP) {
                VENC_H264_QPMAP_S stH264QpMap;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264QPMAP;
                stH264QpMap.u32Gop = u32Gop;
                stH264QpMap.u32StatTime = u32StatTime;
                stH264QpMap.u32SrcFrameRate = u32FrameRate;
                stH264QpMap.fr32DstFrameRate = u32FrameRate;
                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stH264QpMap, sizeof(VENC_H264_QPMAP_S), &stH264QpMap,
                    sizeof(VENC_H264_QPMAP_S));
            } else {
                SAMPLE_PRT("%s,%d,enRcMode(%d) not support\n", __FUNCTION__, __LINE__, enRcMode);
                return HI_FAILURE;
            }
            stVencChnAttr.stVencAttr.stAttrH264e.bRcnRefShareBuf = bRcnRefShareBuf;
                break;
        }
        case PT_MJPEG: {
            if (enRcMode == SAMPLE_RC_FIXQP) {
                VENC_MJPEG_FIXQP_S stMjpegeFixQp;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
                stMjpegeFixQp.u32Qfactor = 95; /* set 95 */
                stMjpegeFixQp.u32SrcFrameRate = u32FrameRate;
                stMjpegeFixQp.fr32DstFrameRate = u32FrameRate;

                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stMjpegFixQp, sizeof(VENC_MJPEG_FIXQP_S), &stMjpegeFixQp,
                    sizeof(VENC_MJPEG_FIXQP_S));
            } else if (enRcMode == SAMPLE_RC_CBR) {
                VENC_MJPEG_CBR_S stMjpegeCbr;

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGCBR;
                stMjpegeCbr.u32StatTime = u32StatTime;
                stMjpegeCbr.u32SrcFrameRate = u32FrameRate;
                stMjpegeCbr.fr32DstFrameRate = u32FrameRate;
                switch (enSize) {
                    case PIC_360P:
                        stMjpegeCbr.u32BitRate = PER_UNIT * 3 + PER_UNIT * u32FrameRate / FPS_30; /* 3M + 1M */
                        break;
                    case PIC_720P:
                        stMjpegeCbr.u32BitRate = PER_UNIT * 5 + PER_UNIT * u32FrameRate / FPS_30; /* 5M + 1M */
                        break;
                    case PIC_1080P:
                        stMjpegeCbr.u32BitRate = PER_UNIT * 8 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 8M + 2M */
                        break;
                    case PIC_2592x1944:
                        stMjpegeCbr.u32BitRate = PER_UNIT * 20 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 20M + 3M */
                        break;
                    case PIC_3840x2160:
                        stMjpegeCbr.u32BitRate = PER_UNIT * 25 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 25M + 5M */
                        break;
                    default:
                        stMjpegeCbr.u32BitRate = PER_UNIT * 20 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 20M + 2M */
                        break;
                }

                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stMjpegCbr, sizeof(VENC_MJPEG_CBR_S), &stMjpegeCbr,
                    sizeof(VENC_MJPEG_CBR_S));
            } else if ((enRcMode == SAMPLE_RC_VBR) || (enRcMode == SAMPLE_RC_AVBR) || (enRcMode == SAMPLE_RC_QVBR) ||
                (enRcMode == SAMPLE_RC_CVBR)) {
                VENC_MJPEG_VBR_S stMjpegVbr;

                if (enRcMode == SAMPLE_RC_AVBR) {
                    SAMPLE_PRT("Mjpege not support AVBR, so change rcmode to VBR!\n");
                }

                stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGVBR;
                stMjpegVbr.u32StatTime = u32StatTime;
                stMjpegVbr.u32SrcFrameRate = u32FrameRate;
                stMjpegVbr.fr32DstFrameRate = 5; /* output 5fps */

                switch (enSize) {
                    case PIC_360P:
                        stMjpegVbr.u32MaxBitRate = PER_UNIT * 3 + PER_UNIT * u32FrameRate / FPS_30; /* 3M + 1M */
                        break;
                    case PIC_720P:
                        stMjpegVbr.u32MaxBitRate = PER_UNIT * 5 + PER_UNIT * u32FrameRate / FPS_30; /* 5M + 1M */
                        break;
                    case PIC_1080P:
                        stMjpegVbr.u32MaxBitRate = PER_UNIT * 8 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 8M + 2M */
                        break;
                    case PIC_2592x1944:
                        stMjpegVbr.u32MaxBitRate = PER_UNIT * 20 + PER_UNIT * 3 * u32FrameRate / FPS_30; /* 20M + 3M */
                        break;
                    case PIC_3840x2160:
                        stMjpegVbr.u32MaxBitRate = PER_UNIT * 25 + PER_UNIT * 5 * u32FrameRate / FPS_30; /* 25M + 5M */
                        break;
                    default:
                        stMjpegVbr.u32MaxBitRate = PER_UNIT * 20 + PER_UNIT * 2 * u32FrameRate / FPS_30; /* 20M + 2M */
                        break;
                }

                (hi_void)memcpy_s(&stVencChnAttr.stRcAttr.stMjpegVbr, sizeof(VENC_MJPEG_VBR_S), &stMjpegVbr,
                    sizeof(VENC_MJPEG_VBR_S));
            } else {
                SAMPLE_PRT("can't support other mode(%d) in this version!\n", enRcMode);
                return HI_FAILURE;
            }
            break;
        }
        case PT_JPEG:
            stJpegAttr.bSupportDCF = HI_FALSE;
            stJpegAttr.stMPFCfg.u8LargeThumbNailNum = 0;
            stJpegAttr.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
            (hi_void)memcpy_s(&stVencChnAttr.stVencAttr.stAttrJpege, sizeof(VENC_ATTR_JPEG_S), &stJpegAttr,
                sizeof(VENC_ATTR_JPEG_S));
            break;
        default:
            SAMPLE_PRT("can't support this enType (%d) in this version!\n", enType);
            return HI_ERR_VENC_NOT_SUPPORT;
    }

    if (enType == PT_MJPEG || enType == PT_JPEG) {
        stVencChnAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
        stVencChnAttr.stGopAttr.stNormalP.s32IPQpDelta = 0;
    } else {
        (hi_void)memcpy_s(&stVencChnAttr.stGopAttr, sizeof(VENC_GOP_ATTR_S), pstGopAttr, sizeof(VENC_GOP_ATTR_S));
        if ((pstGopAttr->enGopMode == VENC_GOPMODE_BIPREDB) && (enType == PT_H264)) {
            if (stVencChnAttr.stVencAttr.u32Profile == 0) {
                stVencChnAttr.stVencAttr.u32Profile = 1;
                SAMPLE_PRT("H.264 base profile not support BIPREDB, so change profile to main profile!\n");
            }
        }

        if ((stVencChnAttr.stRcAttr.enRcMode == VENC_RC_MODE_H264QPMAP) ||
            (stVencChnAttr.stRcAttr.enRcMode == VENC_RC_MODE_H265QPMAP)) {
            if (pstGopAttr->enGopMode == VENC_GOPMODE_ADVSMARTP) {
                stVencChnAttr.stGopAttr.enGopMode = VENC_GOPMODE_SMARTP;
                SAMPLE_PRT("advsmartp not support QPMAP, so change gopmode to smartp!\n");
            }
        }
    }

    s32Ret = HI_MPI_VENC_CreateChn(VencChn, &stVencChnAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("HI_MPI_VENC_CreateChn [%d] failed with %#x! ===\n", VencChn, s32Ret);
        return s32Ret;
    }

    s32Ret = RTSP_COMM_VENC_CloseReEncode(VencChn);
    if (s32Ret != HI_SUCCESS) {
        HI_MPI_VENC_DestroyChn(VencChn);
        return s32Ret;
    }

    return HI_SUCCESS;
}

HI_S32 RTSP_COMM_VENC_Start(VENC_CHN VencChn, PAYLOAD_TYPE_E enType, PIC_SIZE_E enSize, SAMPLE_RC_E enRcMode,
    HI_U32 u32Profile, HI_BOOL bRcnRefShareBuf, VENC_GOP_ATTR_S *pstGopAttr)
{
    HI_S32 s32Ret;
    VENC_RECV_PIC_PARAM_S stRecvParam;

    s32Ret = RTSP_COMM_VENC_Create(VencChn, enType, enSize, enRcMode, u32Profile, bRcnRefShareBuf, pstGopAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("SAMPLE_COMM_VENC_Create failed with%#x! \n", s32Ret);
        return HI_FAILURE;
    }

    stRecvParam.s32RecvPicNum = -1;
    s32Ret = HI_MPI_VENC_StartRecvFrame(VencChn, &stRecvParam);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("HI_MPI_VENC_StartRecvPic failed with%#x! \n", s32Ret);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

HI_S32 RTSP_COMM_VENC_Stop(VENC_CHN VencChn)
{
    HI_S32 s32Ret;

    s32Ret = HI_MPI_VENC_StopRecvFrame(VencChn);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("HI_MPI_VENC_StopRecvPic vechn[%d] failed with %#x!\n", VencChn, s32Ret);
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_VENC_DestroyChn(VencChn);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("HI_MPI_VENC_DestroyChn vechn[%d] failed with %#x!\n", VencChn, s32Ret);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

static HI_VOID *RTSP_COMM_VENC_GetVencStreamProc(HI_VOID *p)
{
    HI_S32 i;
    HI_S32 s32ChnTotal;
    VENC_CHN_ATTR_S stVencChnAttr;
    SAMPLE_VENC_GETSTREAM_PARA_S *pstPara = HI_NULL;
    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    fd_set read_fds;
    HI_U32 u32PictureCnt[VENC_MAX_CHN_NUM] = {0};
    HI_S32 VencFd[VENC_MAX_CHN_NUM];
    HI_CHAR aszFileName[VENC_MAX_CHN_NUM][FILE_NAME_LEN];
    HI_CHAR real_file_name[VENC_MAX_CHN_NUM][PATH_MAX];
    FILE* pFile[VENC_MAX_CHN_NUM] = {HI_NULL};
    HI_S32 fd[VENC_MAX_CHN_NUM] = {0};
    char szFilePostfix[10]; /* length set 10 */
    VENC_CHN_STATUS_S stStat;
    VENC_STREAM_S stStream;
    HI_S32 s32Ret;
    VENC_CHN VencChn;
    PAYLOAD_TYPE_E enPayLoadType[VENC_MAX_CHN_NUM];
    VENC_STREAM_BUF_INFO_S stStreamBufInfo[VENC_MAX_CHN_NUM];

    prctl(PR_SET_NAME, "GetVencStream", 0, 0, 0);

    pstPara = (SAMPLE_VENC_GETSTREAM_PARA_S *)p;
    s32ChnTotal = pstPara->s32Cnt;
    if (s32ChnTotal >= VENC_MAX_CHN_NUM) {
        SAMPLE_PRT("input count invalid\n");
        return NULL;
    }
    for (i = 0; i < s32ChnTotal; i++) {
        /* decide the stream file name, and open file to save stream */
        VencChn = pstPara->VeChn[i];
        s32Ret = HI_MPI_VENC_GetChnAttr(VencChn, &stVencChnAttr);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("HI_MPI_VENC_GetChnAttr chn[%d] failed with %#x!\n", VencChn, s32Ret);
            return NULL;
        }
        enPayLoadType[i] = stVencChnAttr.stVencAttr.enType;

        s32Ret = RTSP_COMM_VENC_GetFilePostfix(enPayLoadType[i], szFilePostfix, sizeof(szFilePostfix));
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("SAMPLE_COMM_VENC_GetFilePostfix [%d] failed with %#x!\n", stVencChnAttr.stVencAttr.enType,
                s32Ret);
            return NULL;
        }
        VencFd[i] = HI_MPI_VENC_GetFd(VencChn);
        if (VencFd[i] < 0) {
            SAMPLE_PRT("HI_MPI_VENC_GetFd failed with %#x!\n", VencFd[i]);
            return NULL;
        }
        if (maxfd <= VencFd[i]) {
            maxfd = VencFd[i];
        }

        s32Ret = HI_MPI_VENC_GetStreamBufInfo(VencChn, &stStreamBufInfo[i]);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("HI_MPI_VENC_GetStreamBufInfo failed with %#x!\n", s32Ret);
            return (void *)HI_FAILURE;
        }
    }

    while (HI_TRUE == pstPara->bThreadStart) {
        // FD_ZERO(&read_fds);
        // for (i = 0; i < s32ChnTotal; i++) {
        //     FD_SET(VencFd[i], &read_fds);
        // }

        // TimeoutVal.tv_sec = 2; /* 2 s */
        // TimeoutVal.tv_usec = 0;
        // s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);
        // if (s32Ret < 0) {
        //     SAMPLE_PRT("select failed!\n");
        //     break;
        // } 
        // else if (s32Ret == 0) {
        //     SAMPLE_PRT("get venc stream time out, exit thread\n");
        //     continue;
        // } 
        // else {
            for (i = 0; i < s32ChnTotal; i++) {
                // if (FD_ISSET(VencFd[i], &read_fds)) {

                    (HI_VOID)memset_s(&stStream, sizeof(stStream), 0, sizeof(stStream));
                    VencChn = pstPara->VeChn[i];

                    /* step 1 : query how many packs in one-frame stream. */
                    s32Ret = HI_MPI_VENC_QueryStatus(VencChn, &stStat);
                    if (s32Ret != HI_SUCCESS) {
                        SAMPLE_PRT("HI_MPI_VENC_QueryStatus chn[%d] failed with %#x!\n", VencChn, s32Ret);
                        break;
                    }
                    if (stStat.u32CurPacks == 0) {
                        continue;
                    }

                    /* step 2 : malloc corresponding number of pack nodes. */
                    stStream.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
                    if (stStream.pstPack == NULL) {
                        SAMPLE_PRT("malloc stream pack failed!\n");
                        break;
                    }

                    stStream.u32PackCount = stStat.u32CurPacks;
                    s32Ret = HI_MPI_VENC_GetStream(VencChn, &stStream, HI_TRUE);
                    if (s32Ret != HI_SUCCESS) {
                        free(stStream.pstPack);
                        stStream.pstPack = NULL;
                        SAMPLE_PRT("HI_MPI_VENC_GetStream failed with %#x!\n", s32Ret);
                        break;
                    }

                    /* step 3 : put stream to rtsp. */
                    ffmpeg_send_frame(&stStream);

                    s32Ret = HI_MPI_VENC_ReleaseStream(VencChn, &stStream);
                    if (s32Ret != HI_SUCCESS) {
                        SAMPLE_PRT("HI_MPI_VENC_ReleaseStream failed!\n");
                        free(stStream.pstPack);
                        stStream.pstPack = NULL;
                        break;
                    }

                    free(stStream.pstPack);
                    stStream.pstPack = NULL;
                    u32PictureCnt[i]++;
                // }
            }
            usleep(20*1000);
        // }
    }

    return NULL;
}


HI_S32 RTSP_COMM_VENC_StartGetStream(VENC_CHN VeChn[], HI_S32 s32Cnt)
{
    HI_S32 i;

    if (VeChn == NULL) {
        SAMPLE_PRT("null pointer\n");
        return HI_FAILURE;
    }
    gs_stPara.bThreadStart = HI_TRUE;
    gs_stPara.s32Cnt = 1;
    gs_stPara.VeChn[0] = 0;
    // for (i = 0; i < s32Cnt && i < VENC_MAX_CHN_NUM; i++) {
    //     gs_stPara.VeChn[i] = VeChn[i];
    // }
    ffmpeg_init_rtsp();
    return pthread_create(&gs_VencPid, 0, RTSP_COMM_VENC_GetVencStreamProc, (HI_VOID *)&gs_stPara);
}

HI_S32 RTSP_COMM_VENC_StopGetStream(void)
{
    if (HI_TRUE == gs_stPara.bThreadStart) {
        gs_stPara.bThreadStart = HI_FALSE;
        pthread_join(gs_VencPid, 0);
    }
    ffmpeg_deinit_rtsp();
    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
