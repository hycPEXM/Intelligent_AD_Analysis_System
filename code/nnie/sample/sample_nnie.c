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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <math.h>
#include <time.h>
#include <termios.h>

#include<sys/types.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<arpa/inet.h>

#include "hi_common.h"
#include "hi_comm_sys.h"
#include "hi_comm_svp.h"
#include "sample_comm.h"
#include "sample_comm_svp.h"
#include "sample_comm_nnie.h"
#include "sample_nnie_main.h"
#include "sample_svp_nnie_software.h"
#include "sample_comm_ive.h"
#include "vgs_img.h"
#include "misc_util.h"
#include "osd_img.h"
#include "base_interface.h"

#include "hi_debug.h"
#include "hi_comm_ive.h"

#include "model.h"

#define THRESH_MIN         0.4
#define ARRAY_SUBSCRIPT_OFFSET_1    1
#define ARRAY_SUBSCRIPT_OFFSET_2    2
#define ARRAY_SUBSCRIPT_OFFSET_3    3
#define DETECT_OBJ_MAX     32
#define DRAW_RETC_THICK    2    // Draw the width of the line
#define ARRAY_SUBSCRIPT_0     0
#define ARRAY_SUBSCRIPT_1     1
#define ARRAY_SUBSCRIPT_2     2
#define ARRAY_SUBSCRIPT_3     3
#define ARRAY_SUBSCRIPT_4     4
#define ARRAY_SUBSCRIPT_5     5
#define ARRAY_SUBSCRIPT_6     6
#define ARRAY_SUBSCRIPT_7     7
#define ARRAY_SUBSCRIPT_8     8
#define ARRAY_SUBSCRIPT_9     9

#define ARGB1555_YELLOW2    0xFF00 // 1 11111 11111 00000
#define ARGB1555_BLUE       0x801F // 1 00000 00000 11111
#define ARGB1555_BLACK      0x8000 // 1 00000 00000 00000
#define ARGB1555_RED        0xFC00 // 1 11111 00000 00000
#define TXT_BEGX            20
#define TXT_BEGY            20

#define IMG_FULL_CHN    3 // Full channel / three channel, for YUV444, RGB888

static DetectObjInfo objs[DETECT_OBJ_MAX] = {0};
static RectBox boxs[DETECT_OBJ_MAX] = {0};
static RectBox remainingBoxs[DETECT_OBJ_MAX] = {0};
static VIDEO_FRAME_INFO_S *dstFrm;
static RectBox BoxsIDprint[DETECT_OBJ_MAX] = {0};

static hi_bool g_stop_signal = HI_FALSE;
/* cnn para */
static SAMPLE_SVP_NNIE_MODEL_S s_stCnnModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stCnnNnieParam = { 0 };
static SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S s_stCnnSoftwareParam = { 0 };
/* segment para */
static SAMPLE_SVP_NNIE_MODEL_S s_stSegnetModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stSegnetNnieParam = { 0 };
/* fasterrcnn para */
static SAMPLE_SVP_NNIE_MODEL_S s_stFasterRcnnModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stFasterRcnnNnieParam = { 0 };
static SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S s_stFasterRcnnSoftwareParam = { 0 };
static SAMPLE_SVP_NNIE_NET_TYPE_E s_enNetType;
/* rfcn para */
static SAMPLE_SVP_NNIE_MODEL_S s_stRfcnModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stRfcnNnieParam = { 0 };
static SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S s_stRfcnSoftwareParam = { 0 };
static SAMPLE_IVE_SWITCH_S s_stRfcnSwitch = { HI_FALSE, HI_FALSE };
static HI_BOOL s_bNnieStopSignal = HI_FALSE;
static pthread_t s_hNnieThread = 0;
static SAMPLE_VI_CONFIG_S s_stViConfig = { 0 };

/* ssd para */
static SAMPLE_SVP_NNIE_MODEL_S s_stSsdModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stSsdNnieParam = { 0 };
static SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S s_stSsdSoftwareParam = { 0 };
/* yolov1 para */
static SAMPLE_SVP_NNIE_MODEL_S s_stYolov1Model = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stYolov1NnieParam = { 0 };
static SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S s_stYolov1SoftwareParam = { 0 };
/* yolov2 para */
static SAMPLE_SVP_NNIE_MODEL_S s_stYolov2Model = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stYolov2NnieParam = { 0 };
static SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S s_stYolov2SoftwareParam = { 0 };
/* yolov3 para */
static SAMPLE_SVP_NNIE_MODEL_S s_stYolov3Model = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stYolov3NnieParam = { 0 };
static SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S s_stYolov3SoftwareParam = { 0 };
/* lstm para */
static SAMPLE_SVP_NNIE_MODEL_S s_stLstmModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stLstmNnieParam = { 0 };
/* pvanet para */
static SAMPLE_SVP_NNIE_MODEL_S s_stPvanetModel = { 0 };
static SAMPLE_SVP_NNIE_PARAM_S s_stPvanetNnieParam = { 0 };
static SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S s_stPvanetSoftwareParam = { 0 };

/*yolov3 para*/
static SAMPLE_IVE_SWITCH_S s_stYolov3Switch = {HI_FALSE,HI_FALSE};

/*yolov2 para*/
static SAMPLE_IVE_SWITCH_S s_stYolov2Switch = {HI_FALSE,HI_FALSE};
/*print on screen(OSD)*/
static OsdSet* g_osdsPerson = NULL;
static HI_S32 g_osd0Person = -1;
static HI_S32 g_osd_Person_ID[DETECT_OBJ_MAX];
//memset_s(g_osd_Person_ID, sizeof(HI_S32)*DETECT_OBJ_MAX, -1, sizeof(HI_S32)*DETECT_OBJ_MAX); //或用0xFF
/*track*/
static HI_U32 ID_num_total=0; //被跟踪的总人数
static HI_FLOAT focused_time_total=0.0;//人群注意力总时长
// static HI_BOOL accumulate_first_flag=HI_FALSE;
static HI_BOOL g_uart_switch_flag = HI_FALSE;


int uartFdOur = 0;
#define UART_BUFF_SIZE           32
/*
 * 串口设置
 * Serial port settings
 */
// int Uart1Config(int fd)
// {
//     struct termios newtio = {0}, oldtio = {0};
//     /*
//      * 获取原有串口配置
//      * Get the original serial port configuration
//      */
//     if (tcgetattr(fd, &oldtio) != 0) {
//         perror("SetupSerial 1");
//         return -1;
//     }
//     (void)memset_s(&newtio, sizeof(newtio), 0, sizeof(newtio));
//     /*
//      * CREAD开启串行数据接收，CLOCAL打开本地连接模式
//      * CREAD opens serial data reception, CLOCAL opens local connection mode
//      */
//     newtio.c_cflag  |=  CLOCAL | CREAD;

//     /*
//      * 设置数据位
//      * Set data bit
//      */
//     newtio.c_cflag &= ~CSIZE;
//     newtio.c_cflag |= CS8;
//     /*
//      * 设置奇偶校验位
//      * Set parity bit
//      */
//     newtio.c_cflag &= ~PARENB; // 无奇偶校验
//     /*
//      * 设置波特率115200
//      * Set baud rate 115200
//      */
//     cfsetispeed(&newtio, B115200);
//     cfsetospeed(&newtio, B115200);

//     /*
//      * 设置停止位
//      * Set stop bit
//      */
//     newtio.c_cflag &=  ~CSTOPB; /* 默认为一位停止位 */
//     /*
//      * 设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时
//      *
//      * Set the minimum characters and waiting time,
//      * when there are no special requirements for receiving characters and waiting time
//      */
//     newtio.c_cc[VTIME]  = 0; /* 非规范模式读取时的超时时间 */
//     newtio.c_cc[VMIN] = 0; /* 非规范模式读取时的最小字符数 */
//     /*
//      * tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来
//      *
//      * tcflush clears the unfinished input/output requests and data of the terminal;
//      * TCIFLUSH means clearing the data being received and not reading it out
//      */
//     tcflush(fd, TCIFLUSH);
//     if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
//         perror("com set error");
//         return -1;
//     }
//     return 0;
// }

// /*
//  * @berf uart 发送数据
//  * @param int fd: 串口文件描述符
//  * @param void *buf:发送数据buffer
//  * @param int len:数据缓冲长度
//  *
//  * @berf uart send
//  * @param int fd: uart file descriptor
//  * @param void *buf:send data buf
//  * @param int len:data buf len
//  */
// int UartSend(int fd, char *buf, int len)
// {
//     int ret = 0;
//     int count = 0;
//     char *sendBuf = buf;
//     int sendLen = len;

//     tcflush(fd, TCIFLUSH);

//     while (sendLen > 0) {
//         ret = write(fd, (char*)sendBuf + count, sendLen);
//         if (ret < 1) {
//             printf("write data below 1 byte % d\r\n", ret);
//             break;
//         }
//         count += ret;
//         sendLen = sendLen - ret;
//     }

//     return count;
// }

// /*
//  * @berf uart 读取数据
//  * @param int uart_fd: 串口文件描述符
//  * @param void *buf: 读取数据buffer
//  * @param int len: 数据缓冲区长度
//  * @param int timeoutMs: 读取数据时间
//  *
//  * @berf uart read
//  * @param int uart_fd: uart file descriptor
//  * @param void *buf: read data buf
//  * @param int len: data buf len
//  * @param int timeoutMs: read data time
//  */
// int UartRead(int fd, char *buf, int len, int timeoutMs)
// {
//     int ret = 0;
//     size_t  rsum = 0;
//     ret = 0;
//     fd_set rset;
//     struct timeval time;
//     int timeout = timeoutMs;
//     char *readBuf = buf;
//     int readLen = len;

//     while (rsum < readLen) {
//         time.tv_sec = timeout / 1000; /* 1000:转换成秒 */
//         time.tv_usec = (timeout - time.tv_sec * 1000) * 1000; /* 1000, 1000:转换为微秒 */
//         FD_ZERO(&rset);
//         FD_SET(fd, &rset);
//         ret = select(fd + 1, &rset, NULL, NULL, &time); // 非阻塞式读取数据
//         if (ret <= 0) {
//             if (ret == 0) {
//                 printf("time over!\r\n");
//                 return -1;
//             }
//             if (ret == -1) {
//                 printf("select error!\r\n");
//                 continue; // 信号中断
//             }
//             return -1;
//         } else {
//             ret = read(fd, (char *)readBuf + rsum, readLen - rsum);
//             if (ret < 0) {
//                 printf("read data failed\r\n");
//                 return ret;
//             } else {
//                 rsum += ret;
//             }
//         }
//     }
//     return rsum;
// }

// unsigned int UartOpenInit(void)
// {
//     int fd;
//     char *uart1 = "/dev/ttyAMA1";

//     if ((fd = open(uart1, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
//         printf("open %s is failed", uart1);
//         return -1;
//     } else {
//         Uart1Config(fd);
//     }
//     return fd;
// }

void UartClose(int fd)
{
    close(fd);
}

/* function : NNIE Forward */
/*
 * 获取结果
 * Fetch result
 */
static void Yolo2FetchRes(SVP_BLOB_S *pstDstScore, SVP_BLOB_S *pstDstRoi, SVP_BLOB_S *pstClassRoiNum,
    DetectObjInfo resBuf[], int resSize, int* resLen)
{
    HI_U32 i;
    HI_U32 j;
    HI_U32 u32RoiNumBias = 0;
    HI_U32 u32ScoreBias;
    HI_U32 u32BboxBias;
    HI_FLOAT f32Score;
    HI_S32* ps32Score = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstDstScore->u64VirAddr);
    HI_S32* ps32Roi = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstDstRoi->u64VirAddr);
    HI_S32* ps32ClassRoiNum = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstClassRoiNum->u64VirAddr);
    HI_U32 u32ClassNum = pstClassRoiNum->unShape.stWhc.u32Width;

    HI_ASSERT(u32ClassNum == 3); // 2: the number of class
    HI_ASSERT(resSize > 0);
    int current_cls =1;
    int resId = 0;
    *resLen = 0;
    memset_s(resBuf, resSize * sizeof(resBuf[0]), 0x00, resSize * sizeof(resBuf[0]));

    u32RoiNumBias += ps32ClassRoiNum[0];
    for (i = 1; i < u32ClassNum; i++) {
        current_cls = i;
        u32ScoreBias = u32RoiNumBias;
        u32BboxBias = u32RoiNumBias * SAMPLE_SVP_NNIE_COORDI_NUM;
        /*
         * 如果置信度分数大于结果阈值，则打印结果
         * If the confidence score greater than result threshold, the result will be printed
         */
        if ((HI_FLOAT)ps32Score[u32ScoreBias] / SAMPLE_SVP_NNIE_QUANT_BASE >=
            THRESH_MIN && ps32ClassRoiNum[i] != 0) {
        }
        for (j = 0; j < (HI_U32)ps32ClassRoiNum[i]; j++) {
            f32Score = (HI_FLOAT)ps32Score[u32ScoreBias + j] / SAMPLE_SVP_NNIE_QUANT_BASE;
            if (f32Score < THRESH_MIN) {
                SAMPLE_PRT("f32Score:%.2f\n", f32Score);
                break;
            }
            if (resId >= resSize) {
                SAMPLE_PRT("yolo2 resBuf full\n");
                break;
            }
            resBuf[resId].cls = current_cls; // class 1
            resBuf[resId].score = f32Score;

            RectBox *box = &resBuf[resId].box;
            box->xmin = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM];
            box->ymin = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + ARRAY_SUBSCRIPT_OFFSET_1];
            box->xmax = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + ARRAY_SUBSCRIPT_OFFSET_2];
            box->ymax = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + ARRAY_SUBSCRIPT_OFFSET_3];
            if (box->xmin >= box->xmax || box->ymin >= box->ymax) {
                // SAMPLE_PRT("yolo1_orig: {%d, %d, %d, %d}, discard for coord ERR\n",
                //     box->xmin, box->ymin, box->xmax, box->ymax);
            } else {
                resId++;
            }
        }
        u32RoiNumBias += ps32ClassRoiNum[i];
    }

    *resLen = resId;
}

static HI_S32 SAMPLE_SVP_NNIE_Forward(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S *pstInputDataIdx, SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S *pstProcSegIdx,
    HI_BOOL bInstant)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 i, j;
    HI_BOOL bFinish = HI_FALSE;
    SVP_NNIE_HANDLE hSvpNnieHandle = 0;
    HI_U32 u32TotalStepNum = 0;

    SAMPLE_SVP_CHECK_EXPR_RET(pstProcSegIdx->u32SegIdx >= pstNnieParam->pstModel->u32NetSegNum ||
        pstInputDataIdx->u32SegIdx >= pstNnieParam->pstModel->u32NetSegNum ||
        pstNnieParam->pstModel->u32NetSegNum > SVP_NNIE_MAX_NET_SEG_NUM,
        HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstProcSegIdx->u32SegIdx(%u) and pstInputDataIdx->u32SegIdx(%u) "
        "should be less than %u, pstNnieParam->pstModel->u32NetSegNum(%u) can't be greater than %u!\n",
        pstProcSegIdx->u32SegIdx, pstInputDataIdx->u32SegIdx, pstNnieParam->pstModel->u32NetSegNum,
        pstNnieParam->pstModel->u32NetSegNum, SVP_NNIE_MAX_NET_SEG_NUM);

    SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx].stTskBuf.u64PhyAddr,
        SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
        pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx].stTskBuf.u64VirAddr),
        pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx].stTskBuf.u32Size);

    for (i = 0; i < pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx].u32DstNum; i++) {
        if (pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].enType == SVP_BLOB_TYPE_SEQ_S32) {
            for (j = 0; j < pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num; j++) {
                u32TotalStepNum += *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32,
                    pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stSeq.u64VirAddrStep) +
                    j);
            }
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                u32TotalStepNum * pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        } else {
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Chn *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Height *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        }
    }

    /* set input blob according to node name */
    if (pstInputDataIdx->u32SegIdx != pstProcSegIdx->u32SegIdx) {
        for (i = 0; i < pstNnieParam->pstModel->astSeg[pstProcSegIdx->u32SegIdx].u16SrcNum; i++) {
            for (j = 0; j < pstNnieParam->pstModel->astSeg[pstInputDataIdx->u32SegIdx].u16DstNum; j++) {
                if (strncmp(pstNnieParam->pstModel->astSeg[pstInputDataIdx->u32SegIdx].astDstNode[j].szName,
                    pstNnieParam->pstModel->astSeg[pstProcSegIdx->u32SegIdx].astSrcNode[i].szName,
                    SVP_NNIE_NODE_NAME_LEN) == 0) {
                    pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astSrc[i] =
                        pstNnieParam->astSegData[pstInputDataIdx->u32SegIdx].astDst[j];
                    break;
                }
            }
            SAMPLE_SVP_CHECK_EXPR_RET((j == pstNnieParam->pstModel->astSeg[pstInputDataIdx->u32SegIdx].u16DstNum),
                HI_FAILURE, SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,can't find %d-th seg's %d-th src blob!\n",
                pstProcSegIdx->u32SegIdx, i);
        }
    }

    /* NNIE_Forward */
    s32Ret = HI_MPI_SVP_NNIE_Forward(&hSvpNnieHandle, pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astSrc,
        pstNnieParam->pstModel, pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst,
        &pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx], bInstant);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,HI_MPI_SVP_NNIE_Forward failed!\n");

    if (bInstant) {
        /* Wait NNIE finish */
        while (HI_ERR_SVP_NNIE_QUERY_TIMEOUT == (s32Ret = HI_MPI_SVP_NNIE_Query(
            pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx].enNnieId, hSvpNnieHandle, &bFinish, HI_TRUE))) {
            usleep(100); /* sleep 100 micro_seconds */
            SAMPLE_SVP_TRACE(SAMPLE_SVP_ERR_LEVEL_INFO, "HI_MPI_SVP_NNIE_Query Query timeout!\n");
        }
    }
    u32TotalStepNum = 0;
    for (i = 0; i < pstNnieParam->astForwardCtrl[pstProcSegIdx->u32SegIdx].u32DstNum; i++) {
        if (SVP_BLOB_TYPE_SEQ_S32 == pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].enType) {
            for (j = 0; j < pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num; j++) {
                u32TotalStepNum += *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32,
                    pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stSeq.u64VirAddrStep) +
                    j);
            }
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                u32TotalStepNum * pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        } else {
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Chn *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Height *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        }
    }

    return s32Ret;
}

/* function : NNIE ForwardWithBbox */
static HI_S32 SAMPLE_SVP_NNIE_ForwardWithBbox(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S *pstInputDataIdx, SVP_SRC_BLOB_S astBbox[],
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S *pstProcSegIdx, HI_BOOL bInstant)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_BOOL bFinish = HI_FALSE;
    SVP_NNIE_HANDLE hSvpNnieHandle = 0;
    HI_U32 u32TotalStepNum = 0;
    HI_U32 i, j;

    SAMPLE_SVP_CHECK_EXPR_RET(pstProcSegIdx->u32SegIdx >= pstNnieParam->pstModel->u32NetSegNum ||
        pstInputDataIdx->u32SegIdx >= pstNnieParam->pstModel->u32NetSegNum ||
        pstNnieParam->pstModel->u32NetSegNum > SVP_NNIE_MAX_NET_SEG_NUM,
        HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstProcSegIdx->u32SegIdx(%u) and pstInputDataIdx->u32SegIdx(%u) "
        "should be less than %u, pstNnieParam->pstModel->u32NetSegNum(%u) should be less than %u!\n",
        pstProcSegIdx->u32SegIdx, pstInputDataIdx->u32SegIdx, pstNnieParam->pstModel->u32NetSegNum,
        pstNnieParam->pstModel->u32NetSegNum, SVP_NNIE_MAX_NET_SEG_NUM);
    SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx].stTskBuf.u64PhyAddr,
        SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
        pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx].stTskBuf.u64VirAddr),
        pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx].stTskBuf.u32Size);

    for (i = 0; i < pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx].u32DstNum; i++) {
        if (SVP_BLOB_TYPE_SEQ_S32 == pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].enType) {
            for (j = 0; j < pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num; j++) {
                u32TotalStepNum += *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32,
                    pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stSeq.u64VirAddrStep) +
                    j);
            }
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                u32TotalStepNum * pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        } else {
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Chn *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Height *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        }
    }

    /* set input blob according to node name */
    if (pstInputDataIdx->u32SegIdx != pstProcSegIdx->u32SegIdx) {
        for (i = 0; i < pstNnieParam->pstModel->astSeg[pstProcSegIdx->u32SegIdx].u16SrcNum; i++) {
            for (j = 0; j < pstNnieParam->pstModel->astSeg[pstInputDataIdx->u32SegIdx].u16DstNum; j++) {
                if (strncmp(pstNnieParam->pstModel->astSeg[pstInputDataIdx->u32SegIdx].astDstNode[j].szName,
                    pstNnieParam->pstModel->astSeg[pstProcSegIdx->u32SegIdx].astSrcNode[i].szName,
                    SVP_NNIE_NODE_NAME_LEN) == 0) {
                    pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astSrc[i] =
                        pstNnieParam->astSegData[pstInputDataIdx->u32SegIdx].astDst[j];
                    break;
                }
            }
            SAMPLE_SVP_CHECK_EXPR_RET((j == pstNnieParam->pstModel->astSeg[pstInputDataIdx->u32SegIdx].u16DstNum),
                HI_FAILURE, SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,can't find %d-th seg's %d-th src blob!\n",
                pstProcSegIdx->u32SegIdx, i);
        }
    }
    /* NNIE_ForwardWithBbox */
    s32Ret = HI_MPI_SVP_NNIE_ForwardWithBbox(&hSvpNnieHandle, pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astSrc,
        astBbox, pstNnieParam->pstModel, pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst,
        &pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx], bInstant);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,HI_MPI_SVP_NNIE_ForwardWithBbox failed!\n");

    if (bInstant) {
        /* Wait NNIE finish */
        while (HI_ERR_SVP_NNIE_QUERY_TIMEOUT ==
            (s32Ret = HI_MPI_SVP_NNIE_Query(pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx].enNnieId,
            hSvpNnieHandle, &bFinish, HI_TRUE))) {
            usleep(100); /* sleep 100 micro_seconds */
            SAMPLE_SVP_TRACE(SAMPLE_SVP_ERR_LEVEL_INFO, "HI_MPI_SVP_NNIE_Query Query timeout!\n");
        }
    }
    u32TotalStepNum = 0;

    for (i = 0; i < pstNnieParam->astForwardWithBboxCtrl[pstProcSegIdx->u32SegIdx].u32DstNum; i++) {
        if (SVP_BLOB_TYPE_SEQ_S32 == pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].enType) {
            for (j = 0; j < pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num; j++) {
                u32TotalStepNum += *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32,
                    pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stSeq.u64VirAddrStep) +
                    j);
            }
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                u32TotalStepNum * pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        } else {
            SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64PhyAddr,
                SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u64VirAddr),
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Num *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Chn *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].unShape.stWhc.u32Height *
                pstNnieParam->astSegData[pstProcSegIdx->u32SegIdx].astDst[i].u32Stride);
        }
    }

    return s32Ret;
}

/* function : Fill Src Data */
static HI_S32 SAMPLE_SVP_NNIE_FillSrcData(SAMPLE_SVP_NNIE_CFG_S *pstNnieCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S *pstInputDataIdx)
{
    FILE *fp = NULL;
    HI_U32 i = 0, j = 0, n = 0;
    HI_U32 u32Height = 0, u32Width = 0, u32Chn = 0, u32Stride = 0, u32Dim = 0;
    HI_U32 u32VarSize = 0;
    HI_U8 *pu8PicAddr = NULL;
    HI_U32 *pu32StepAddr = NULL;
    HI_U32 u32SegIdx = pstInputDataIdx->u32SegIdx;
    HI_U32 u32NodeIdx = pstInputDataIdx->u32NodeIdx;
    HI_U32 u32TotalStepNum = 0;
    HI_ULONG ulSize;
    HI_CHAR path[PATH_MAX] = {0};

    /* open file */
    SAMPLE_SVP_CHECK_EXPR_RET(pstNnieCfg->pszPic == HI_NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstNnieCfg->pszPic is null!\n");
    SAMPLE_SVP_CHECK_EXPR_RET(pstInputDataIdx->u32SegIdx >= SVP_NNIE_MAX_NET_SEG_NUM, HI_INVALID_VALUE,
        SAMPLE_SVP_ERR_LEVEL_ERROR, "Error, u32SegIdx should be less than %u!\n", SVP_NNIE_MAX_NET_SEG_NUM);
    SAMPLE_SVP_CHECK_EXPR_RET(pstInputDataIdx->u32NodeIdx >= SVP_NNIE_MAX_INPUT_NUM, HI_INVALID_VALUE,
        SAMPLE_SVP_ERR_LEVEL_ERROR, "Error, u32NodeIdx should be less than %u!\n", SVP_NNIE_MAX_INPUT_NUM);

    SAMPLE_SVP_CHECK_EXPR_RET((strlen(pstNnieCfg->pszPic) > PATH_MAX) ||
        (realpath(pstNnieCfg->pszPic, path) == HI_NULL),
        HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR, "Error, file_name is invalid!\n");
    fp = fopen(path, "rb");
    SAMPLE_SVP_CHECK_EXPR_RET(fp == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR, "Error, open file failed!\n");

    /* get data size */
    if (SVP_BLOB_TYPE_U8 <= pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].enType &&
        SVP_BLOB_TYPE_YVU422SP >= pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].enType) {
        u32VarSize = sizeof(HI_U8);
    } else {
        u32VarSize = sizeof(HI_U32);
    }

    /* fill src data */
    if (SVP_BLOB_TYPE_SEQ_S32 == pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].enType) {
        u32Dim = pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].unShape.stSeq.u32Dim;
        u32Stride = pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Stride;
        pu32StepAddr = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32,
            pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].unShape.stSeq.u64VirAddrStep);
        pu8PicAddr = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U8,
            pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u64VirAddr);
        for (n = 0; n < pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Num; n++) {
            for (i = 0; i < *(pu32StepAddr + n); i++) {
                ulSize = fread(pu8PicAddr, u32Dim * u32VarSize, 1, fp);
                SAMPLE_SVP_CHECK_EXPR_GOTO(ulSize != 1, FAIL, SAMPLE_SVP_ERR_LEVEL_ERROR,
                    "Error,Read image file failed!\n");
                pu8PicAddr += u32Stride;
            }
            u32TotalStepNum += *(pu32StepAddr + n);
        }
        SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u64PhyAddr,
            SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
            pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u64VirAddr),
            u32TotalStepNum * u32Stride);
    } else {
        u32Height = pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].unShape.stWhc.u32Height;
        u32Width = pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].unShape.stWhc.u32Width;
        u32Chn = pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].unShape.stWhc.u32Chn;
        u32Stride = pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Stride;
        pu8PicAddr = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U8,
            pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u64VirAddr);
        if (SVP_BLOB_TYPE_YVU420SP == pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].enType) {
            for (n = 0; n < pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Num; n++) {
                for (i = 0; i < u32Chn * u32Height / 2; i++) { /* Brightness: 1 height, Chroma: 1/2 height */
                    ulSize = fread(pu8PicAddr, u32Width * u32VarSize, 1, fp);
                    SAMPLE_SVP_CHECK_EXPR_GOTO(ulSize != 1, FAIL, SAMPLE_SVP_ERR_LEVEL_ERROR,
                        "Error,Read image file failed!\n");
                    pu8PicAddr += u32Stride;
                }
            }
        } else if (SVP_BLOB_TYPE_YVU422SP == pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].enType) {
            for (n = 0; n < pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Num; n++) {
                for (i = 0; i < u32Height * 2; i++) { /* Brightness: 1 height, Chroma: 1 height */
                    ulSize = fread(pu8PicAddr, u32Width * u32VarSize, 1, fp);
                    SAMPLE_SVP_CHECK_EXPR_GOTO(ulSize != 1, FAIL, SAMPLE_SVP_ERR_LEVEL_ERROR,
                        "Error,Read image file failed!\n");
                    pu8PicAddr += u32Stride;
                }
            }
        } else {
            for (n = 0; n < pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Num; n++) {
                for (i = 0; i < u32Chn; i++) {
                    for (j = 0; j < u32Height; j++) {
                        ulSize = fread(pu8PicAddr, u32Width * u32VarSize, 1, fp);
                        SAMPLE_SVP_CHECK_EXPR_GOTO(ulSize != 1, FAIL, SAMPLE_SVP_ERR_LEVEL_ERROR,
                            "Error,Read image file failed!\n");
                        pu8PicAddr += u32Stride;
                    }
                }
            }
        }
        SAMPLE_COMM_SVP_FlushCache(pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u64PhyAddr,
            SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_VOID,
            pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u64VirAddr),
            pstNnieParam->astSegData[u32SegIdx].astSrc[u32NodeIdx].u32Num * u32Chn * u32Height * u32Stride);
    }

    (HI_VOID)fclose(fp);
    return HI_SUCCESS;
FAIL:

    (HI_VOID)fclose(fp);
    return HI_FAILURE;
}

/* function : print report result */
static HI_S32 SAMPLE_SVP_NNIE_PrintReportResult(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam)
{
    HI_U32 u32SegNum = pstNnieParam->pstModel->u32NetSegNum;
    HI_U32 i = 0, j = 0, k = 0, n = 0;
    HI_U32 u32SegIdx = 0, u32NodeIdx = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_CHAR acReportFileName[SAMPLE_SVP_NNIE_REPORT_NAME_LENGTH] = {'\0'};
    FILE *fp = NULL;
    HI_U32 *pu32StepAddr = NULL;
    HI_S32 *ps32ResultAddr = NULL;
    HI_U32 u32Height = 0, u32Width = 0, u32Chn = 0, u32Stride = 0, u32Dim = 0;

    for (u32SegIdx = 0; u32SegIdx < u32SegNum; u32SegIdx++) {
        for (u32NodeIdx = 0; u32NodeIdx < pstNnieParam->pstModel->astSeg[u32SegIdx].u16DstNum; u32NodeIdx++) {
            s32Ret = snprintf_s(acReportFileName, SAMPLE_SVP_NNIE_REPORT_NAME_LENGTH,
                SAMPLE_SVP_NNIE_REPORT_NAME_LENGTH - 1, "seg%d_layer%d_output%d_inst.linear.hex", u32SegIdx,
                pstNnieParam->pstModel->astSeg[u32SegIdx].astDstNode[u32NodeIdx].u32NodeId, 0);
            SAMPLE_SVP_CHECK_EXPR_RET(s32Ret < 0, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
                "Error,create file name failed!\n");

            fp = fopen(acReportFileName, "w");
            SAMPLE_SVP_CHECK_EXPR_RET(fp == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
                "Error,open file failed!\n");

            if (SVP_BLOB_TYPE_SEQ_S32 == pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].enType) {
                u32Dim = pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].unShape.stSeq.u32Dim;
                u32Stride = pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].u32Stride;
                pu32StepAddr = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32,
                    pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].unShape.stSeq.u64VirAddrStep);
                ps32ResultAddr = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,
                    pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].u64VirAddr);

                for (n = 0; n < pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].u32Num; n++) {
                    for (i = 0; i < *(pu32StepAddr + n); i++) {
                        for (j = 0; j < u32Dim; j++) {
                            s32Ret = fprintf(fp, "%08x\n", *(ps32ResultAddr + j));
                            SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret < 0, PRINT_FAIL, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                "Error,write report result file failed!\n");
                        }
                        ps32ResultAddr += u32Stride / sizeof(HI_U32);
                    }
                }
            } else {
                u32Height = pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].unShape.stWhc.u32Height;
                u32Width = pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].unShape.stWhc.u32Width;
                u32Chn = pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].unShape.stWhc.u32Chn;
                u32Stride = pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].u32Stride;
                ps32ResultAddr = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,
                    pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].u64VirAddr);
                for (n = 0; n < pstNnieParam->astSegData[u32SegIdx].astDst[u32NodeIdx].u32Num; n++) {
                    for (i = 0; i < u32Chn; i++) {
                        for (j = 0; j < u32Height; j++) {
                            for (k = 0; k < u32Width; k++) {
                                s32Ret = fprintf(fp, "%08x\n", *(ps32ResultAddr + k));
                                SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret < 0, PRINT_FAIL, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Error,write report result file failed!\n");
                            }
                            ps32ResultAddr += u32Stride / sizeof(HI_U32);
                        }
                    }
                }
            }
            (HI_VOID)fclose(fp);
        }
    }
    return HI_SUCCESS;

PRINT_FAIL:
    (HI_VOID)fclose(fp);
    return HI_FAILURE;
}

/* function : Cnn software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Cnn_SoftwareDeinit(SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S *pstCnnSoftWarePara)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstCnnSoftWarePara == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstCnnSoftWarePara can't be NULL!\n");
    if ((pstCnnSoftWarePara->stGetTopN.u64PhyAddr != 0) && (pstCnnSoftWarePara->stGetTopN.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstCnnSoftWarePara->stGetTopN.u64PhyAddr, pstCnnSoftWarePara->stGetTopN.u64VirAddr);
        pstCnnSoftWarePara->stGetTopN.u64PhyAddr = 0;
        pstCnnSoftWarePara->stGetTopN.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Cnn Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Cnn_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware para deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software para deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Cnn_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Cnn_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Cnn software para init */
static HI_S32 SAMPLE_SVP_NNIE_Cnn_SoftwareParaInit(SAMPLE_SVP_NNIE_CFG_S *pstNnieCfg,
    SAMPLE_SVP_NNIE_PARAM_S *pstCnnPara, SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S *pstCnnSoftWarePara)
{
    HI_U32 u32GetTopNMemSize = 0;
    HI_U32 u32GetTopNAssistBufSize = 0;
    HI_U32 u32GetTopNPerFrameSize = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32ClassNum = pstCnnPara->pstModel->astSeg[0].astDstNode[0].unShape.stWhc.u32Width;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U64 u64Tmp;

    /* get mem size */
    u64Tmp = (HI_U64)pstCnnSoftWarePara->u32TopN * sizeof(SAMPLE_SVP_NNIE_CNN_GETTOPN_UNIT_S);
    SAMPLE_SVP_CHECK_EXPR_RET(u64Tmp > SAMPLE_SVP_NNIE_MAX_MEM, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,u64Tmp should be less than %u!\n", SAMPLE_SVP_NNIE_MAX_MEM);
    u32GetTopNPerFrameSize = (HI_U32)u64Tmp;

    u64Tmp = SAMPLE_SVP_NNIE_ALIGN16(u32GetTopNPerFrameSize) * (HI_U64)pstNnieCfg->u32MaxInputNum;
    SAMPLE_SVP_CHECK_EXPR_RET(u64Tmp > SAMPLE_SVP_NNIE_MAX_MEM, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,u64Tmp should be less than %u!\n", SAMPLE_SVP_NNIE_MAX_MEM);
    u32GetTopNMemSize = (HI_U32)u64Tmp;

    u64Tmp = (HI_U64)u32ClassNum * sizeof(SAMPLE_SVP_NNIE_CNN_GETTOPN_UNIT_S);
    SAMPLE_SVP_CHECK_EXPR_RET(u64Tmp > SAMPLE_SVP_NNIE_MAX_MEM, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,u64Tmp should be less than %u!\n", SAMPLE_SVP_NNIE_MAX_MEM);
    u32GetTopNAssistBufSize = (HI_U32)u64Tmp;

    SAMPLE_SVP_CHECK_EXPR_RET((HI_U64)u32GetTopNMemSize + u32GetTopNAssistBufSize > SAMPLE_SVP_NNIE_MAX_MEM,
        HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,total size should be less than %u!\n",
        SAMPLE_SVP_NNIE_MAX_MEM);
    u32TotalSize = u32GetTopNMemSize + u32GetTopNAssistBufSize;

    /* malloc mem */
    s32Ret =
        SAMPLE_COMM_SVP_MallocMem("SAMPLE_CNN_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr, u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);

    /* init GetTopn */
    pstCnnSoftWarePara->stGetTopN.u32Num = pstNnieCfg->u32MaxInputNum;
    pstCnnSoftWarePara->stGetTopN.unShape.stWhc.u32Chn = 1;
    pstCnnSoftWarePara->stGetTopN.unShape.stWhc.u32Height = 1;
    pstCnnSoftWarePara->stGetTopN.unShape.stWhc.u32Width = u32GetTopNPerFrameSize / sizeof(HI_U32);
    pstCnnSoftWarePara->stGetTopN.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32GetTopNPerFrameSize);
    pstCnnSoftWarePara->stGetTopN.u64PhyAddr = u64PhyAddr;
    pstCnnSoftWarePara->stGetTopN.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr);

    /* init AssistBuf */
    pstCnnSoftWarePara->stAssistBuf.u32Size = u32GetTopNAssistBufSize;
    pstCnnSoftWarePara->stAssistBuf.u64PhyAddr = u64PhyAddr + u32GetTopNMemSize;
    pstCnnSoftWarePara->stAssistBuf.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) + u32GetTopNMemSize;

    return s32Ret;
}

/* function : Cnn init */
static HI_S32 SAMPLE_SVP_NNIE_Cnn_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstNnieCfg, SAMPLE_SVP_NNIE_PARAM_S *pstCnnPara,
    SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S *pstCnnSoftWarePara)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstNnieCfg, pstCnnPara);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software para */
    if (pstCnnSoftWarePara != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Cnn_SoftwareParaInit(pstNnieCfg, pstCnnPara, pstCnnSoftWarePara);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error(%#x),SAMPLE_SVP_NNIE_Cnn_SoftwareParaInit failed!\n", s32Ret);
    }

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Cnn_Deinit(pstCnnPara, pstCnnSoftWarePara, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Cnn_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}

/* function : Cnn process */
static HI_S32 SAMPLE_SVP_NNIE_Cnn_PrintResult(SVP_BLOB_S *pstGetTopN, HI_U32 u32TopN)
{
    HI_U32 i, j;
    HI_U32 *pu32Tmp = NULL;
    HI_U32 u32Stride;
    SAMPLE_SVP_CHECK_EXPR_RET(pstGetTopN == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,pstGetTopN can't be NULL!\n");

    u32Stride = pstGetTopN->u32Stride;
    for (j = 0; j < pstGetTopN->u32Num; j++) {
        SAMPLE_SVP_TRACE_INFO("==== The %dth image info====\n", j);
        pu32Tmp = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, ((HI_UL)pstGetTopN->u64VirAddr + j * u32Stride));
        for (i = 0; i < u32TopN * 2; i += 2) { /* leave a space */
            SAMPLE_SVP_TRACE_INFO("%d:%d\n", pu32Tmp[i], pu32Tmp[i + 1]);
        }
    }
    return HI_SUCCESS;
}
static hi_void SAMPLE_SVP_NNIE_Cnn_Stop(void)
{
    SAMPLE_SVP_NNIE_Cnn_Deinit(&s_stCnnNnieParam, &s_stCnnSoftwareParam, &s_stCnnModel);
    (HI_VOID)memset_s(&s_stCnnNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stCnnSoftwareParam, sizeof(SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_CNN_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stCnnModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show Cnn sample(image 28x28 U8_C1) */
void SAMPLE_SVP_NNIE_Cnn(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/y/0_28x28.y";
    const HI_CHAR *pcModelName = "./data/nnie_model/classification/inst_mnist_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core
    s_stCnnSoftwareParam.u32TopN = 5; /* to get 5 highest results */
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");

    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Cnn_Stop();
        return;
    }

    /* CNN Load model */
    SAMPLE_SVP_TRACE_INFO("Cnn Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stCnnModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* CNN parameter initialization */
    /* Cnn software parameters are set in SAMPLE_SVP_NNIE_Cnn_SoftwareParaInit,
     if user has changed net struct, please make sure the parameter settings in
     SAMPLE_SVP_NNIE_Cnn_SoftwareParaInit function are correct */
    SAMPLE_SVP_TRACE_INFO("Cnn parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Cnn_Stop();
        return;
    }
    s_stCnnNnieParam.pstModel = &s_stCnnModel.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Cnn_ParamInit(&stNnieCfg, &s_stCnnNnieParam, &s_stCnnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Cnn_ParamInit failed!\n");

    /* record tskBuf */
    s32Ret = HI_MPI_SVP_NNIE_AddTskBuf(&(s_stCnnNnieParam.astForwardCtrl[0].stTskBuf));
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,HI_MPI_SVP_NNIE_AddTskBuf failed!\n");

    /* Fill src data */
    SAMPLE_SVP_TRACE_INFO("Cnn start!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Cnn_Stop();
        return;
    }
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stCnnNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_1, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process(process the 0-th segment) */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Cnn_Stop();
        return;
    }
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stCnnNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_1, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* Software process */
    /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Cnn_GetTopN
     function's input data are correct */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Cnn_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Cnn_GetTopN(&s_stCnnNnieParam, &s_stCnnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_1, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_CnnGetTopN failed!\n");

    /* Print result */
    SAMPLE_SVP_TRACE_INFO("Cnn result:\n");
    s32Ret = SAMPLE_SVP_NNIE_Cnn_PrintResult(&(s_stCnnSoftwareParam.stGetTopN), s_stCnnSoftwareParam.u32TopN);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_1, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Cnn_PrintResult failed!\n");

CNN_FAIL_1:
    /* Remove TskBuf */
    s32Ret = HI_MPI_SVP_NNIE_RemoveTskBuf(&(s_stCnnNnieParam.astForwardCtrl[0].stTskBuf));
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, CNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,HI_MPI_SVP_NNIE_RemoveTskBuf failed!\n");
CNN_FAIL_0:
    SAMPLE_SVP_NNIE_Cnn_Deinit(&s_stCnnNnieParam, &s_stCnnSoftwareParam, &s_stCnnModel);
    SAMPLE_COMM_SVP_CheckSysExit();
}

hi_void SAMPLE_SVP_NNIE_Cnn_HandleSig(hi_void)
{
    g_stop_signal = HI_TRUE;
}
static void SAMPLE_SVP_NNIE_Segnet_Stop(void)
{
    SAMPLE_SVP_NNIE_Cnn_Deinit(&s_stSegnetNnieParam, NULL, &s_stSegnetModel);
    (HI_VOID)memset_s(&s_stSegnetNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stSegnetModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show Segnet sample(image 224x224 U8_C3) */
void SAMPLE_SVP_NNIE_Segnet(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/segnet_image_224x224.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/segmentation/inst_segnet_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0;
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Segnet_Stop();
        return;
    }
    /* Segnet Load model */
    SAMPLE_SVP_TRACE_INFO("Segnet Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stSegnetModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SEGNET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* Segnet parameter initialization */
    SAMPLE_SVP_TRACE_INFO("Segnet parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Segnet_Stop();
        return;
    }
    s_stSegnetNnieParam.pstModel = &s_stSegnetModel.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Cnn_ParamInit(&stNnieCfg, &s_stSegnetNnieParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SEGNET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Cnn_ParamInit failed!\n");

    /* Fill src data */
    SAMPLE_SVP_TRACE_INFO("Segnet start!\n");
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Segnet_Stop();
    }
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stSegnetNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SEGNET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process(process the 0-th segment) */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Segnet_Stop();
        return;
    }
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stSegnetNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SEGNET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* print report result */
    s32Ret = SAMPLE_SVP_NNIE_PrintReportResult(&s_stSegnetNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SEGNET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_PrintReportResult failed!\n");

    SAMPLE_SVP_TRACE_INFO("Segnet is successfully processed!\n");

SEGNET_FAIL_0:
    SAMPLE_SVP_NNIE_Cnn_Deinit(&s_stSegnetNnieParam, NULL, &s_stSegnetModel);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : Segnet sample signal handle */
hi_void SAMPLE_SVP_NNIE_Segnet_HandleSig(hi_void)
{
    g_stop_signal = HI_TRUE;
}

/* function : print detection result */
static HI_S32 SAMPLE_SVP_NNIE_Detection_PrintResult(SVP_BLOB_S *pstDstScore, SVP_BLOB_S *pstDstRoi,
    SVP_BLOB_S *pstClassRoiNum, HI_FLOAT f32PrintResultThresh)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RoiNumBias = 0;
    HI_U32 u32ScoreBias = 0;
    HI_U32 u32BboxBias = 0;
    HI_FLOAT f32Score = 0.0f;
    HI_S32 *ps32Score = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstDstScore->u64VirAddr);
    HI_S32 *ps32Roi = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstDstRoi->u64VirAddr);
    HI_S32 *ps32ClassRoiNum = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstClassRoiNum->u64VirAddr);
    HI_U32 u32ClassNum = pstClassRoiNum->unShape.stWhc.u32Width;
    HI_S32 s32XMin = 0, s32YMin = 0, s32XMax = 0, s32YMax = 0;

    u32RoiNumBias += ps32ClassRoiNum[0];
    for (i = 1; i < u32ClassNum; i++) {
        u32ScoreBias = u32RoiNumBias;
        u32BboxBias = u32RoiNumBias * SAMPLE_SVP_NNIE_COORDI_NUM;
        /* if the confidence score greater than result threshold, the result will be printed */
        if ((HI_FLOAT)ps32Score[u32ScoreBias] / SAMPLE_SVP_NNIE_QUANT_BASE >= f32PrintResultThresh &&
            ps32ClassRoiNum[i] != 0) {
            SAMPLE_SVP_TRACE_INFO("==== The %dth class box info====\n", i);
        }
        for (j = 0; j < (HI_U32)ps32ClassRoiNum[i]; j++) {
            f32Score = (HI_FLOAT)ps32Score[u32ScoreBias + j] / SAMPLE_SVP_NNIE_QUANT_BASE;
            if (f32Score < f32PrintResultThresh) {
                break;
            }
            s32XMin = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM];
            s32YMin = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + 1]; /* to get next element of this array */
            s32XMax = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + 2]; /* to get next element of this array */
            s32YMax = ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + 3]; /* to get next element of this array */
            SAMPLE_SVP_TRACE_INFO("%d %d %d %d %f\n", s32XMin, s32YMin, s32XMax, s32YMax, f32Score);
        }
        u32RoiNumBias += ps32ClassRoiNum[i];
    }
    return HI_SUCCESS;
}

/* function : FasterRcnn software deinit */
static HI_S32 SAMPLE_SVP_NNIE_FasterRcnn_SoftwareDeinit(SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if ((pstSoftWareParam->stRpnTmpBuf.u64PhyAddr != 0) && (pstSoftWareParam->stRpnTmpBuf.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stRpnTmpBuf.u64PhyAddr, pstSoftWareParam->stRpnTmpBuf.u64VirAddr);
        pstSoftWareParam->stRpnTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stRpnTmpBuf.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : FasterRcnn Deinit */
static HI_S32 SAMPLE_SVP_NNIE_FasterRcnn_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_FasterRcnn_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : FasterRcnn software para init */
static HI_S32 SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg,
    SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RpnTmpBufSize = 0;
    HI_U32 u32RpnBboxBufSize = 0;
    HI_U32 u32GetResultTmpBufSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    /* RPN parameter init */
    /* The values of the following parameters are related to algorithm principles.
        For details, see related algorithms. */
    pstSoftWareParam->u32MaxRoiNum = pstCfg->u32MaxRoiNum;
    if (SAMPLE_SVP_NNIE_VGG16_FASTER_RCNN == s_enNetType) {
        pstSoftWareParam->u32ClassNum = 4;
        pstSoftWareParam->u32NumRatioAnchors = 3;
        pstSoftWareParam->u32NumScaleAnchors = 3;
        pstSoftWareParam->au32Scales[0] = 8 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[1] = 16 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[2] = 32 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Ratios[0] = 0.5 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Ratios[1] = 1 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Ratios[2] = 2 * SAMPLE_SVP_QUANT_BASE;
    } else {
        pstSoftWareParam->u32ClassNum = 2;
        pstSoftWareParam->u32NumRatioAnchors = 1;
        pstSoftWareParam->u32NumScaleAnchors = 9;
        pstSoftWareParam->au32Scales[0] = 1.5 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[1] = 2.1 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[2] = 2.9 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[3] = 4.1 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[4] = 5.8 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[5] = 8.0 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[6] = 11.3 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[7] = 15.8 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Scales[8] = 22.1 * SAMPLE_SVP_QUANT_BASE;
        pstSoftWareParam->au32Ratios[0] = 2.44 * SAMPLE_SVP_QUANT_BASE;
    }

    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;
    pstSoftWareParam->u32MinSize = 16;
    pstSoftWareParam->u32FilterThresh = 16;
    pstSoftWareParam->u32SpatialScale = (HI_U32)(0.0625 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.7 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->u32FilterThresh = 0;
    pstSoftWareParam->u32NumBeforeNms = 6000;
    for (i = 0; i < pstSoftWareParam->u32ClassNum; i++) {
        pstSoftWareParam->au32ConfThresh[i] = 1;
    }
    pstSoftWareParam->u32ValidNmsThresh = (HI_U32)(0.3 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->stRpnBbox.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Height = pstCfg->u32MaxRoiNum;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Width = SAMPLE_SVP_COORDI_NUM;
    pstSoftWareParam->stRpnBbox.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(SAMPLE_SVP_COORDI_NUM * sizeof(HI_U32));
    pstSoftWareParam->stRpnBbox.u32Num = 1;
    for (i = 0; i < SAMPLE_SVP_NNIE_SEGMENT_NUM; i++) {
        for (j = 0; j < pstNnieParam->pstModel->astSeg[0].u16DstNum; j++) {
            if (strncmp(pstNnieParam->pstModel->astSeg[0].astDstNode[j].szName,
                pstSoftWareParam->apcRpnDataLayerName[i], SVP_NNIE_NODE_NAME_LEN) == 0) {
                pstSoftWareParam->aps32Conv[i] =
                    SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstNnieParam->astSegData[0].astDst[j].u64VirAddr);
                pstSoftWareParam->au32ConvHeight[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Height;
                pstSoftWareParam->au32ConvWidth[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Width;
                pstSoftWareParam->au32ConvChannel[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Chn;
                break;
            }
        }
        SAMPLE_SVP_CHECK_EXPR_RET((j == pstNnieParam->pstModel->astSeg[0].u16DstNum), HI_FAILURE,
            SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,failed to find report node %s!\n",
            pstSoftWareParam->apcRpnDataLayerName[i]);
        if (i == 0) {
            pstSoftWareParam->u32ConvStride = pstNnieParam->astSegData[0].astDst[j].u32Stride;
        }
    }

    /* calculate software mem size */
    u32ClassNum = pstSoftWareParam->u32ClassNum;
    u32RpnTmpBufSize = SAMPLE_SVP_NNIE_RpnTmpBufSize(pstSoftWareParam->u32NumRatioAnchors,
        pstSoftWareParam->u32NumScaleAnchors, pstSoftWareParam->au32ConvHeight[0], pstSoftWareParam->au32ConvWidth[0]);
    SAMPLE_SVP_CHECK_EXPR_RET(u32RpnTmpBufSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_RpnTmpBufSize failed!\n");
    u32RpnTmpBufSize = SAMPLE_SVP_NNIE_ALIGN16(u32RpnTmpBufSize);
    u32RpnBboxBufSize = pstSoftWareParam->stRpnBbox.u32Num * pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Height *
        pstSoftWareParam->stRpnBbox.u32Stride;
    u32GetResultTmpBufSize = SAMPLE_SVP_NNIE_FasterRcnn_GetResultTmpBufSize(pstCfg->u32MaxRoiNum, u32ClassNum);
    SAMPLE_SVP_CHECK_EXPR_RET(u32GetResultTmpBufSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FasterRcnn_GetResultTmpBufSize failed!\n");
    u32GetResultTmpBufSize = SAMPLE_SVP_NNIE_ALIGN16(u32GetResultTmpBufSize);
    u32DstRoiSize =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstCfg->u32MaxRoiNum * sizeof(HI_U32) * SAMPLE_SVP_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstCfg->u32MaxRoiNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize +
        u32ClassRoiNumSize;

    /* malloc mem */
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_RCNN_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    /* set addr */
    pstSoftWareParam->stRpnTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stRpnTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);
    pstSoftWareParam->stRpnTmpBuf.u32Size = u32RpnTmpBufSize;

    pstSoftWareParam->stRpnBbox.u64PhyAddr = u64PhyAddr + u32RpnTmpBufSize;
    pstSoftWareParam->stRpnBbox.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr) + u32RpnTmpBufSize;

    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnBboxBufSize + u32RpnTmpBufSize);
    pstSoftWareParam->stGetResultTmpBuf.u32Size = u32GetResultTmpBufSize;

    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize;
    pstSoftWareParam->stDstRoi.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize);
    pstSoftWareParam->stDstRoi.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32) * SAMPLE_SVP_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width =
        u32ClassNum * pstSoftWareParam->u32MaxRoiNum * SAMPLE_SVP_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr =
        u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) +
        u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * pstSoftWareParam->u32MaxRoiNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr =
        u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) +
        u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

/* function : FasterRcnn parameter initialization */
static HI_S32 SAMPLE_SVP_NNIE_FasterRcnn_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstFasterRcnnCfg,
    SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware parameter */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstFasterRcnnCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software parameter */
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit(pstFasterRcnnCfg, pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_FasterRcnn_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}
static hi_void SAMPLE_SVP_NNIE_FasterRcnn_Stop(hi_void)
{
    SAMPLE_SVP_NNIE_FasterRcnn_Deinit(&s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam, &s_stFasterRcnnModel);
    (HI_VOID)memset_s(&s_stFasterRcnnNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stFasterRcnnSoftwareParam, sizeof(SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stFasterRcnnModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show fasterRcnn sample(image 1240x375 U8_C3) */
void SAMPLE_SVP_NNIE_FasterRcnn(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/single_person_1240x375.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_alexnet_frcnn_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 i = 0;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };
    g_stop_signal = HI_FALSE;

    /* Set configuration parameter */
    s_enNetType = SAMPLE_SVP_NNIE_ALEXNET_FASTER_RCNN;
    f32PrintResultThresh = 0.8f;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 300; // set maxmum 300 ROIs
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core for 0-th Seg
    stNnieCfg.aenNnieCoreId[1] = SVP_NNIE_ID_0; // set NNIE core for 1-th Seg

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    /* FasterRcnn Load model */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stFasterRcnnModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* FasterRcnn para init */
    /* apcRpnDataLayerName is used to set RPN data layer name
      and search RPN input data,if user has changed network struct, please
      make sure the data layer names are correct */
    /* FasterRcnn parameters are set in SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit,
     if user has changed network struct, please make sure the parameter settings in
     SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    s_stFasterRcnnNnieParam.pstModel = &s_stFasterRcnnModel.stModel;
    s_stFasterRcnnSoftwareParam.apcRpnDataLayerName[0] = "rpn_cls_score";
    s_stFasterRcnnSoftwareParam.apcRpnDataLayerName[1] = "rpn_bbox_pred";
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_ParamInit(&stNnieCfg, &s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FasterRcnn_ParamInit failed!\n");

    /* Fill 0-th input node of 0-th seg */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn start!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stFasterRcnnNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process 0-th seg */
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stFasterRcnnNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* RPN */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_Rpn(&s_stFasterRcnnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FasterRcnn_Rpn failed!\n");

    if (s_stFasterRcnnSoftwareParam.stRpnBbox.unShape.stWhc.u32Height != 0) {
        /* NNIE process 1-th seg, the input conv data comes from 0-th seg's 0-th report node,
         the input roi comes from RPN results */
        stInputDataIdx.u32SegIdx = 0;
        stInputDataIdx.u32NodeIdx = 0;
        stProcSegIdx.u32SegIdx = 1;
        if (g_stop_signal == HI_TRUE) {
            SAMPLE_SVP_NNIE_FasterRcnn_Stop();
            return;
        }
        s32Ret = SAMPLE_SVP_NNIE_ForwardWithBbox(&s_stFasterRcnnNnieParam, &stInputDataIdx,
            &s_stFasterRcnnSoftwareParam.stRpnBbox, &stProcSegIdx, HI_TRUE);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

        /* GetResult */
        /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_FasterRcnn_GetResult
         function's input data are correct */
        if (g_stop_signal == HI_TRUE) {
            SAMPLE_SVP_NNIE_FasterRcnn_Stop();
            return;
        }
        s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_GetResult(&s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_FasterRcnn_GetResult failed!\n");
    } else {
        for (i = 0; i < s_stFasterRcnnSoftwareParam.stClassRoiNum.unShape.stWhc.u32Width; i++) {
            *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, s_stFasterRcnnSoftwareParam.stClassRoiNum.u64VirAddr) + i) = 0;
        }
    }
    /* print result, Alexnet_FasterRcnn has 2 classes:
     class 0:background     class 1:pedestrian */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    SAMPLE_SVP_TRACE_INFO("FasterRcnn result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stFasterRcnnSoftwareParam.stDstScore,
        &s_stFasterRcnnSoftwareParam.stDstRoi, &s_stFasterRcnnSoftwareParam.stClassRoiNum, f32PrintResultThresh);

FRCNN_FAIL_0:
    SAMPLE_SVP_NNIE_FasterRcnn_Deinit(&s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam, &s_stFasterRcnnModel);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function :show fasterrcnn double_roipooling sample(image 224x224 U8_C3) */
void SAMPLE_SVP_NNIE_FasterRcnn_DoubleRoiPooling(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/double_roipooling_224_224.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_fasterrcnn_double_roipooling_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 i = 0;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    s_enNetType = SAMPLE_SVP_NNIE_VGG16_FASTER_RCNN;
    f32PrintResultThresh = 0.8f;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 300; // set maxmum 300 ROIs
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core for 0-th Seg
    stNnieCfg.aenNnieCoreId[1] = SVP_NNIE_ID_0; // set NNIE core for 1-th Seg
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    /* FasterRcnn Load model */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stFasterRcnnModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* FasterRcnn para init */
    /* apcRpnDataLayerName is used to set RPN data layer name
      and search RPN input data,if user has changed network struct, please
      make sure the data layer names are correct */
    /* FasterRcnn parameters are set in SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit,
     if user has changed network struct, please make sure the parameter settings in
     SAMPLE_SVP_NNIE_FaasterRcnn_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    s_stFasterRcnnNnieParam.pstModel = &s_stFasterRcnnModel.stModel;
    s_stFasterRcnnSoftwareParam.apcRpnDataLayerName[0] = "rpn_cls_score";
    s_stFasterRcnnSoftwareParam.apcRpnDataLayerName[1] = "rpn_bbox_pred";
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_ParamInit(&stNnieCfg, &s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FasterRcnn_ParamInit failed!\n");

    /* Fill 0-th input node of 0-th seg */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn start!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stFasterRcnnNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process 0-th seg */
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stFasterRcnnNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* RPN */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_FasterRcnn_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_Rpn(&s_stFasterRcnnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FasterRcnn_Rpn failed!\n");
    if (s_stFasterRcnnSoftwareParam.stRpnBbox.unShape.stWhc.u32Height != 0) {
        /* NNIE process 1-st seg, the input conv data comes from 0-th seg's 0-th and
          1-st report node,the input roi comes from RPN results */
        stInputDataIdx.u32SegIdx = 0;
        stInputDataIdx.u32NodeIdx = 0;
        stProcSegIdx.u32SegIdx = 1;
        s32Ret = SAMPLE_SVP_NNIE_ForwardWithBbox(&s_stFasterRcnnNnieParam, &stInputDataIdx,
            &s_stFasterRcnnSoftwareParam.stRpnBbox, &stProcSegIdx, HI_TRUE);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

        /* GetResult */
        /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_FasterRcnn_GetResult
         function's input data are correct */
        s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_GetResult(&s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, FRCNN_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_FasterRcnn_GetResult failed!\n");
    } else {
        for (i = 0; i < s_stFasterRcnnSoftwareParam.stClassRoiNum.unShape.stWhc.u32Width; i++) {
            *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, s_stFasterRcnnSoftwareParam.stClassRoiNum.u64VirAddr) + i) = 0;
        }
    }
    /* print result, FasterRcnn has 4 classes:
     class 0:background  class 1:person  class 2:people  class 3:person sitting */
    SAMPLE_SVP_TRACE_INFO("FasterRcnn result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stFasterRcnnSoftwareParam.stDstScore,
        &s_stFasterRcnnSoftwareParam.stDstRoi, &s_stFasterRcnnSoftwareParam.stClassRoiNum, f32PrintResultThresh);
FRCNN_FAIL_0:
    SAMPLE_SVP_NNIE_FasterRcnn_Deinit(&s_stFasterRcnnNnieParam, &s_stFasterRcnnSoftwareParam, &s_stFasterRcnnModel);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : fasterRcnn sample signal handle */
hi_void SAMPLE_SVP_NNIE_FasterRcnn_HandleSig(hi_void)
{
    g_stop_signal = HI_TRUE;
}

/* function : Rfcn software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Rfcn_SoftwareDeinit(SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if (pstSoftWareParam->stRpnTmpBuf.u64PhyAddr != 0 && pstSoftWareParam->stRpnTmpBuf.u64VirAddr != 0) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stRpnTmpBuf.u64PhyAddr, pstSoftWareParam->stRpnTmpBuf.u64VirAddr);
        pstSoftWareParam->stRpnTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stRpnTmpBuf.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Rfcn Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Rfcn_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Rfcn_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Rfcn_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Rfcn software para init */
static HI_S32 SAMPLE_SVP_NNIE_Rfcn_SoftwareInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RpnTmpBufSize = 0;
    HI_U32 u32RpnBboxBufSize = 0;
    HI_U32 u32GetResultTmpBufSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    /* init Rpn para */
    /* The values of the following parameters are related to algorithm principles.
        For details, see related algorithms. */
    pstSoftWareParam->u32MaxRoiNum = pstCfg->u32MaxRoiNum;
    pstSoftWareParam->u32ClassNum = 21;
    pstSoftWareParam->u32NumRatioAnchors = 3;
    pstSoftWareParam->u32NumScaleAnchors = 3;
    pstSoftWareParam->au32Scales[0] = 8 * SAMPLE_SVP_NNIE_QUANT_BASE;
    pstSoftWareParam->au32Scales[1] = 16 * SAMPLE_SVP_NNIE_QUANT_BASE;
    pstSoftWareParam->au32Scales[2] = 32 * SAMPLE_SVP_NNIE_QUANT_BASE;
    pstSoftWareParam->au32Ratios[0] = 0.5 * SAMPLE_SVP_NNIE_QUANT_BASE;
    pstSoftWareParam->au32Ratios[1] = 1 * SAMPLE_SVP_NNIE_QUANT_BASE;
    pstSoftWareParam->au32Ratios[2] = 2 * SAMPLE_SVP_NNIE_QUANT_BASE;
    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;
    pstSoftWareParam->u32MinSize = 16;
    pstSoftWareParam->u32FilterThresh = 0;
    pstSoftWareParam->u32SpatialScale = (HI_U32)(0.0625 * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.7 * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32FilterThresh = 0;
    pstSoftWareParam->u32NumBeforeNms = 6000;
    for (i = 0; i < pstSoftWareParam->u32ClassNum; i++) {
        pstSoftWareParam->au32ConfThresh[i] = 1;
        pstSoftWareParam->af32ScoreThr[i] = 0.8f;
    }
    pstSoftWareParam->u32ValidNmsThresh = (HI_U32)(0.3 * 4096);

    /* set rpn input data info, the input info is set according to RPN data layers' name */
    for (i = 0; i < SAMPLE_SVP_NNIE_SEGMENT_NUM; i++) {
        for (j = 0; j < pstNnieParam->pstModel->astSeg[0].u16DstNum; j++) {
            if (strncmp(pstNnieParam->pstModel->astSeg[0].astDstNode[j].szName,
                pstSoftWareParam->apcRpnDataLayerName[i], SVP_NNIE_NODE_NAME_LEN) == 0) {
                pstSoftWareParam->aps32Conv[i] =
                    SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstNnieParam->astSegData[0].astDst[j].u64VirAddr);
                pstSoftWareParam->au32ConvHeight[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Height;
                pstSoftWareParam->au32ConvWidth[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Width;
                pstSoftWareParam->au32ConvChannel[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Chn;
                break;
            }
        }
        SAMPLE_SVP_CHECK_EXPR_RET((j == pstNnieParam->pstModel->astSeg[0].u16DstNum), HI_FAILURE,
            SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,failed to find report node %s!\n",
            pstSoftWareParam->apcRpnDataLayerName[i]);
        if (i == 0) {
            pstSoftWareParam->u32ConvStride = pstNnieParam->astSegData[0].astDst[j].u32Stride;
        }
    }

    pstSoftWareParam->stRpnBbox.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Height = pstCfg->u32MaxRoiNum;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Width = SAMPLE_SVP_COORDI_NUM;
    pstSoftWareParam->stRpnBbox.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(SAMPLE_SVP_COORDI_NUM * sizeof(HI_U32));
    pstSoftWareParam->stRpnBbox.u32Num = 1;

    /* malloc software mem */
    u32RpnTmpBufSize = SAMPLE_SVP_NNIE_RpnTmpBufSize(pstSoftWareParam->u32NumRatioAnchors,
        pstSoftWareParam->u32NumScaleAnchors, pstSoftWareParam->au32ConvHeight[0], pstSoftWareParam->au32ConvWidth[0]);
    SAMPLE_SVP_CHECK_EXPR_RET(u32RpnTmpBufSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_RpnTmpBufSize failed!\n");
    u32RpnTmpBufSize = SAMPLE_SVP_NNIE_ALIGN16(u32RpnTmpBufSize);
    u32RpnBboxBufSize = pstSoftWareParam->stRpnBbox.u32Num * pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Height *
        pstSoftWareParam->stRpnBbox.u32Stride;
    u32GetResultTmpBufSize = SAMPLE_SVP_NNIE_Rfcn_GetResultTmpBuf(pstCfg->u32MaxRoiNum, pstSoftWareParam->u32ClassNum);
    SAMPLE_SVP_CHECK_EXPR_RET(u32GetResultTmpBufSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Rfcn_GetResultTmpBuf failed!\n");
    u32GetResultTmpBufSize = SAMPLE_SVP_NNIE_ALIGN16(u32GetResultTmpBufSize);
    u32ClassNum = pstSoftWareParam->u32ClassNum;
    u32DstRoiSize =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstCfg->u32MaxRoiNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstCfg->u32MaxRoiNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize +
        u32ClassRoiNumSize;

    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_RFCN_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    pstSoftWareParam->stRpnTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stRpnTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);
    pstSoftWareParam->stRpnTmpBuf.u32Size = u32RpnTmpBufSize;

    pstSoftWareParam->stRpnBbox.u64PhyAddr = u64PhyAddr + u32RpnTmpBufSize;
    pstSoftWareParam->stRpnBbox.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr) + u32RpnTmpBufSize;

    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr + u32RpnTmpBufSize + u32RpnBboxBufSize;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnTmpBufSize + u32RpnBboxBufSize);
    pstSoftWareParam->stGetResultTmpBuf.u32Size = u32GetResultTmpBufSize;

    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize;
    pstSoftWareParam->stDstRoi.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize);
    pstSoftWareParam->stDstRoi.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum *
        sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width =
        u32ClassNum * pstSoftWareParam->u32MaxRoiNum * SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr =
        u64PhyAddr + u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) +
        u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * pstSoftWareParam->u32MaxRoiNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr =
        u64PhyAddr + u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) +
        u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;
    return s32Ret;
}

/* function : Rfcn init */
static HI_S32 SAMPLE_SVP_NNIE_Rfcn_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);SAMPLE_SVP_NNIE_MAX_CLASS_NUM;

    /* init software para */
    s32Ret = SAMPLE_SVP_NNIE_Rfcn_SoftwareInit(pstCfg, pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Rfcn_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Rfcn_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Rfcn_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}

/* function : roi to rect */
static HI_S32 SAMPLE_SVP_NNIE_RoiToRect(SVP_BLOB_S *pstDstScore, SVP_BLOB_S *pstDstRoi, SVP_BLOB_S *pstClassRoiNum,
    HI_FLOAT *paf32ScoreThr, HI_BOOL bRmBg, SAMPLE_SVP_NNIE_RECT_ARRAY_S *pstRect, HI_U32 u32SrcWidth,
    HI_U32 u32SrcHeight, HI_U32 u32DstWidth, HI_U32 u32DstHeight)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RoiNumBias = 0;
    HI_U32 u32ScoreBias = 0;
    HI_U32 u32BboxBias = 0;
    HI_FLOAT f32Score = 0.0f;
    HI_S32 *ps32Score = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstDstScore->u64VirAddr);
    HI_S32 *ps32Roi = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstDstRoi->u64VirAddr);
    HI_S32 *ps32ClassRoiNum = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstClassRoiNum->u64VirAddr);
    HI_U32 u32ClassNum = pstClassRoiNum->unShape.stWhc.u32Width;
    HI_U32 u32RoiNumTmp = 0;

    SAMPLE_SVP_CHECK_EXPR_RET(u32ClassNum > SAMPLE_SVP_NNIE_MAX_CLASS_NUM, HI_ERR_SVP_NNIE_ILLEGAL_PARAM,
        SAMPLE_SVP_ERR_LEVEL_ERROR, "Error(%#x),u32ClassNum(%u) must be less than or equal %u to!\n",
        HI_ERR_SVP_NNIE_ILLEGAL_PARAM, u32ClassNum, SAMPLE_SVP_NNIE_MAX_CLASS_NUM);
    pstRect->u32TotalNum = 0;
    pstRect->u32ClsNum = u32ClassNum;
    if (bRmBg) {
        pstRect->au32RoiNum[0] = 0;
        u32RoiNumBias += ps32ClassRoiNum[0];
        for (i = 1; i < u32ClassNum; i++) {
            u32ScoreBias = u32RoiNumBias;
            u32BboxBias = u32RoiNumBias * SAMPLE_SVP_NNIE_COORDI_NUM;
            u32RoiNumTmp = 0;
            /* if the confidence score greater than result thresh, the result will be drew */
            if (((HI_FLOAT)ps32Score[u32ScoreBias] / SAMPLE_SVP_NNIE_QUANT_BASE >= paf32ScoreThr[i]) &&
                (ps32ClassRoiNum[i] != 0)) {
                for (j = 0; j < (HI_U32)ps32ClassRoiNum[i]; j++) {
                    /* Score is descend order */
                    f32Score = (HI_FLOAT)ps32Score[u32ScoreBias + j] / SAMPLE_SVP_NNIE_QUANT_BASE;
                    if ((f32Score < paf32ScoreThr[i]) || (u32RoiNumTmp >= SAMPLE_SVP_NNIE_MAX_ROI_NUM_OF_CLASS)) {
                        break;
                    }

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32X =
                        (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM] /
                        (HI_FLOAT)u32SrcWidth * (HI_FLOAT)u32DstWidth) & (~1);
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32Y =
                        (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + \
                        SAMPLE_SVP_NNIE_Y_MIN_OFFSET] / (HI_FLOAT)u32SrcHeight * (HI_FLOAT)u32DstHeight) & (~1);
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32X =
                        (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + \
                        SAMPLE_SVP_NNIE_X_MAX_OFFSET] / (HI_FLOAT)u32SrcWidth * (HI_FLOAT)u32DstWidth) & (~1);
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32Y =
                        pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32Y;

                    /* get the third point coordinate */
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32X =
                        pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32X;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32Y =
                        (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j * SAMPLE_SVP_NNIE_COORDI_NUM + \
                        SAMPLE_SVP_NNIE_Y_MAX_OFFSET] / (HI_FLOAT)u32SrcHeight * (HI_FLOAT)u32DstHeight) & (~1);

                    /* get the fourth point coordinate */
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[3].s32X =
                        pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32X;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[3].s32Y =
                        pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32Y;

                    u32RoiNumTmp++;
                }
            }

            pstRect->au32RoiNum[i] = u32RoiNumTmp;
            pstRect->u32TotalNum += u32RoiNumTmp;
            u32RoiNumBias += ps32ClassRoiNum[i];
        }
    }
    return HI_SUCCESS;
}

/* function : Rfcn Proc */
static HI_S32 SAMPLE_SVP_NNIE_Rfcn_Proc(SAMPLE_SVP_NNIE_PARAM_S *pstParam,
    SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSwParam)
{
    HI_S32 s32Ret = HI_FAILURE;
    HI_U32 i = 0;
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;

    /* NNIE process 0-th seg */
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(pstParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* RPN */
    s32Ret = SAMPLE_SVP_NNIE_Rfcn_Rpn(pstSwParam);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_RFCN_Rpn failed!\n");

    if (pstSwParam->stRpnBbox.unShape.stWhc.u32Height != 0) {
        /* NNIE process 1-th seg, the input data comes from 3-rd report node of 0-th seg,
          the input roi comes from RPN results */
        stInputDataIdx.u32SegIdx = 0;
        stInputDataIdx.u32NodeIdx = 3;
        stProcSegIdx.u32SegIdx = 1;
        s32Ret =
            SAMPLE_SVP_NNIE_ForwardWithBbox(pstParam, &stInputDataIdx, &pstSwParam->stRpnBbox, &stProcSegIdx, HI_TRUE);
        SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

        /* NNIE process 2-nd seg, the input data comes from 4-th report node of 0-th seg
          the input roi comes from RPN results */
        stInputDataIdx.u32SegIdx = 0;
        stInputDataIdx.u32NodeIdx = 4;
        stProcSegIdx.u32SegIdx = 2;
        s32Ret =
            SAMPLE_SVP_NNIE_ForwardWithBbox(pstParam, &stInputDataIdx, &pstSwParam->stRpnBbox, &stProcSegIdx, HI_TRUE);
        SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

        /* GetResult */
        /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Rfcn_GetResult
         function's input data are correct */

        s32Ret = SAMPLE_SVP_NNIE_Rfcn_GetResult(pstParam, pstSwParam);
        SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Rfcn_GetResult failed!\n");
    } else {
        for (i = 0; i < pstSwParam->stClassRoiNum.unShape.stWhc.u32Width; i++) {
            *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, pstSwParam->stClassRoiNum.u64VirAddr) + i) = 0;
        }
    }
    return s32Ret;
}

/* function : Rfcn Proc */
static HI_S32 SAMPLE_SVP_NNIE_Rfcn_Proc_ViToVo(SAMPLE_SVP_NNIE_PARAM_S *pstParam,
    SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSwParam, VIDEO_FRAME_INFO_S *pstExtFrmInfo, HI_U32 u32BaseWidth,
    HI_U32 u32BaseHeight)
{
    HI_S32 s32Ret = HI_FAILURE;
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };

    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    /* SP420 */
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u64VirAddr =
        pstExtFrmInfo->stVFrame.u64VirAddr[0];
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u64PhyAddr =
        pstExtFrmInfo->stVFrame.u64PhyAddr[0];
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u32Stride =
        pstExtFrmInfo->stVFrame.u32Stride[0];

    s32Ret = SAMPLE_SVP_NNIE_Rfcn_Proc(pstParam, pstSwParam);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Rfcn_Proc failed!\n");
    /* draw result, this sample has 21 classes:
     class 0:background     class 1:plane           class 2:bicycle
     class 3:bird           class 4:boat            class 5:bottle
     class 6:bus            class 7:car             class 8:cat
     class 9:chair          class10:cow             class11:diningtable
     class 12:dog           class13:horse           class14:motorbike
     class 15:person        class16:pottedplant     class17:sheep
     class 18:sofa          class19:train           class20:tvmonitor */
    s32Ret = SAMPLE_SVP_NNIE_RoiToRect(&(pstSwParam->stDstScore), &(pstSwParam->stDstRoi), &(pstSwParam->stClassRoiNum),
        pstSwParam->af32ScoreThr, HI_TRUE, &(pstSwParam->stRect), pstExtFrmInfo->stVFrame.u32Width,
        pstExtFrmInfo->stVFrame.u32Height, u32BaseWidth, u32BaseHeight);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_RoiToRect failed!\n", s32Ret);

    return s32Ret;
}

static HI_VOID SAMPLE_SVP_NNIE_Rfcn_Stop(hi_void)
{
    s_bNnieStopSignal = HI_TRUE;
    if (s_hNnieThread != 0) {
        pthread_join(s_hNnieThread, HI_NULL);
        s_hNnieThread = 0;
    }

    SAMPLE_SVP_NNIE_Rfcn_Deinit(&s_stRfcnNnieParam, &s_stRfcnSoftwareParam, &s_stRfcnModel);
    (HI_VOID)memset_s(&s_stRfcnNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stRfcnSoftwareParam, sizeof(SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stRfcnModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));

    SAMPLE_COMM_IVE_StopViVpssVencVo(&s_stViConfig, &s_stRfcnSwitch);
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

static HI_S32 SAMPLE_SVP_NNIE_Rfcn_Pause(hi_void)
{
    printf("---------------press Enter key to exit!---------------\n");
    if (s_bNnieStopSignal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop();
        return HI_FAILURE;
    }
    (hi_void)getchar();
    if (s_bNnieStopSignal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop();
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

/* function : Rfcn vi to vo thread entry */
static HI_VOID *SAMPLE_SVP_NNIE_Rfcn_ViToVo(HI_VOID *pArgs)
{
    HI_S32 s32Ret;
    SAMPLE_SVP_NNIE_PARAM_S *pstParam = NULL;
    SAMPLE_SVP_NNIE_RFCN_SOFTWARE_PARAM_S *pstSwParam = NULL;
    VIDEO_FRAME_INFO_S stBaseFrmInfo;
    VIDEO_FRAME_INFO_S stExtFrmInfo;
    const HI_S32 s32MilliSec = 20000; /* 20000ms timeout */
    const VO_LAYER voLayer = 0;
    const VO_CHN voChn = 0;
    const HI_S32 s32VpssGrp = 0;
    const HI_S32 as32VpssChn[] = {VPSS_CHN0, VPSS_CHN1};

    hi_unused(pArgs);
    pstParam = &s_stRfcnNnieParam;
    pstSwParam = &s_stRfcnSoftwareParam;

    struct timespec start_time1,end_time1;
    float use_time1 =0;

    while (HI_FALSE == s_bNnieStopSignal) {
        clock_gettime(CLOCK_REALTIME, &start_time1);
        s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[1], &stExtFrmInfo, s32MilliSec);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n", s32Ret, s32VpssGrp,
                as32VpssChn[1]);
            continue;
        }

        s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[0], &stBaseFrmInfo, s32MilliSec);
        SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, EXT_RELEASE,
            "Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n", s32Ret, s32VpssGrp,
            as32VpssChn[0]);

        s32Ret = SAMPLE_SVP_NNIE_Rfcn_Proc_ViToVo(pstParam, pstSwParam, &stExtFrmInfo, stBaseFrmInfo.stVFrame.u32Width,
            stBaseFrmInfo.stVFrame.u32Height);
        SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, BASE_RELEASE, "Error(%#x),SAMPLE_SVP_NNIE_Rfcn_Proc failed!\n",
            s32Ret);

        // Draw rect
        s32Ret = SAMPLE_COMM_SVP_NNIE_FillRect(&stBaseFrmInfo, &(pstSwParam->stRect), 0x0000FF00);
        SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, BASE_RELEASE,
            "SAMPLE_COMM_SVP_NNIE_FillRect failed, Error(%#x)!\n", s32Ret);

        s32Ret = HI_MPI_VO_SendFrame(voLayer, voChn, &stBaseFrmInfo, s32MilliSec);
        SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, BASE_RELEASE, "HI_MPI_VO_SendFrame failed, Error(%#x)!\n", s32Ret);

        clock_gettime(CLOCK_REALTIME, &end_time1);
        use_time1 = (float)(end_time1.tv_sec - start_time1.tv_sec)*1000 + (end_time1.tv_nsec - start_time1.tv_nsec) / 1000000.0;
        SAMPLE_SVP_TRACE_INFO("The use time is %f  \n", use_time1);

    BASE_RELEASE:
        s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp, as32VpssChn[0], &stBaseFrmInfo);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n", s32Ret, s32VpssGrp,
                as32VpssChn[0]);
        }

    EXT_RELEASE:
        s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp, as32VpssChn[1], &stExtFrmInfo);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n", s32Ret, s32VpssGrp,
                as32VpssChn[1]);
        }
    }

    return HI_NULL;
}

/* function : Rfcn Vi->VO */
void SAMPLE_SVP_NNIE_Rfcn(void)
{
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_rfcn_resnet50_cycle_352x288.wk";
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SIZE_S stSize;
    PIC_SIZE_E enSize = PIC_CIF;
    HI_S32 s32Ret = HI_SUCCESS;

    (HI_VOID)memset_s(&s_stRfcnModel, sizeof(s_stRfcnModel), 0, sizeof(s_stRfcnModel));
    (HI_VOID)memset_s(&s_stRfcnNnieParam, sizeof(s_stRfcnNnieParam), 0, sizeof(s_stRfcnNnieParam));
    (HI_VOID)memset_s(&s_stRfcnSoftwareParam, sizeof(s_stRfcnSoftwareParam), 0, sizeof(s_stRfcnSoftwareParam));

    /* step 1: start vi vpss vo */
    s_stRfcnSwitch.bVenc = HI_FALSE;
    s_stRfcnSwitch.bVo = HI_TRUE;
    s32Ret = SAMPLE_COMM_IVE_StartViVpssVencVo(&s_stViConfig, &s_stRfcnSwitch, &enSize);
    SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, "Error(%#x),SAMPLE_COMM_IVE_StartViVpssVencVo failed!\n",
        s32Ret);

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enSize, &stSize);
    SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, "Error(%#x),SAMPLE_COMM_SYS_GetPicSize failed!\n", s32Ret);

    /* step 2: init NNIE param */
    stNnieCfg.pszPic = NULL;
    stNnieCfg.u32MaxInputNum = 1; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 300; // set maxmum 300 ROIs
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core for 0-th Seg
    stNnieCfg.aenNnieCoreId[1] = SVP_NNIE_ID_0; // set NNIE core for 1-th Seg
    stNnieCfg.aenNnieCoreId[2] = SVP_NNIE_ID_0; // set NNIE core for 2-th Seg

    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stRfcnModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* apcRpnDataLayerName is used to set RPN data layer name
      and search RPN input data,if user has changed network struct, please
      make sure the data layer names are correct */
    s_stRfcnNnieParam.pstModel = &s_stRfcnModel.stModel;
    s_stRfcnSoftwareParam.apcRpnDataLayerName[0] = "rpn_cls_score";
    s_stRfcnSoftwareParam.apcRpnDataLayerName[1] = "rpn_bbox_pred";
    s32Ret = SAMPLE_SVP_NNIE_Rfcn_ParamInit(&stNnieCfg, &s_stRfcnNnieParam, &s_stRfcnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_1, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Rfcn_ParamInit failed!\n");

    s_bNnieStopSignal = HI_FALSE;

    /* step 3: Create work thread */
    s32Ret = prctl(PR_SET_NAME, "NNIE_ViToVo", 0, 0, 0);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_1, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "thread set name failed!\n");
    s32Ret = pthread_create(&s_hNnieThread, 0, SAMPLE_SVP_NNIE_Rfcn_ViToVo, NULL);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_1, SAMPLE_SVP_ERR_LEVEL_ERROR, "thread create failed!\n");

    s32Ret = SAMPLE_SVP_NNIE_Rfcn_Pause();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR, "vi_rfcn_vo exit!\n");

    s_bNnieStopSignal = HI_TRUE;
    pthread_join(s_hNnieThread, HI_NULL);
    s_hNnieThread = 0;
END_RFCN_1:

    SAMPLE_SVP_NNIE_Rfcn_Deinit(&s_stRfcnNnieParam, &s_stRfcnSoftwareParam, &s_stRfcnModel);
END_RFCN_0:
    SAMPLE_COMM_IVE_StopViVpssVencVo(&s_stViConfig, &s_stRfcnSwitch);
    return;
}

/* function : rfcn sample signal handle */
void SAMPLE_SVP_NNIE_Rfcn_HandleSig(void)
{
    s_bNnieStopSignal = HI_TRUE;
}

/* function : rfcn sample signal handle */
static hi_void SAMPLE_SVP_NNIE_Rfcn_Stop_File(hi_void)
{
    SAMPLE_SVP_NNIE_Rfcn_Deinit(&s_stRfcnNnieParam, &s_stRfcnSoftwareParam, &s_stRfcnModel);
    (HI_VOID)memset_s(&s_stRfcnModel, sizeof(s_stRfcnModel), 0, sizeof(s_stRfcnModel));
    (HI_VOID)memset_s(&s_stRfcnNnieParam, sizeof(s_stRfcnNnieParam), 0, sizeof(s_stRfcnNnieParam));
    (HI_VOID)memset_s(&s_stRfcnSoftwareParam, sizeof(s_stRfcnSoftwareParam), 0, sizeof(s_stRfcnSoftwareParam));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

void SAMPLE_SVP_NNIE_Rfcn_HandleSig_File(void)
{
    g_stop_signal = HI_TRUE;
}

/* function : Rfcn Read file */
void SAMPLE_SVP_NNIE_Rfcn_File(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/hoser_dog_car_person_800x600.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_rfcn_resnet50_cycle.wk";
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;

    (HI_VOID)memset_s(&s_stRfcnModel, sizeof(s_stRfcnModel), 0, sizeof(s_stRfcnModel));
    (HI_VOID)memset_s(&s_stRfcnNnieParam, sizeof(s_stRfcnNnieParam), 0, sizeof(s_stRfcnNnieParam));
    (HI_VOID)memset_s(&s_stRfcnSoftwareParam, sizeof(s_stRfcnSoftwareParam), 0, sizeof(s_stRfcnSoftwareParam));

    stNnieCfg.pszPic = pcSrcFile;
    f32PrintResultThresh = 0.8f;
    stNnieCfg.u32MaxInputNum = 1; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 300; // set maxmum 300 ROIs
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core for 0-th Seg
    stNnieCfg.aenNnieCoreId[1] = SVP_NNIE_ID_0; // set NNIE core for 1-th Seg
    stNnieCfg.aenNnieCoreId[2] = SVP_NNIE_ID_0; // set NNIE core for 2-th Seg
    g_stop_signal = HI_FALSE;
    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop_File();
        return;
    }

    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stRfcnModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* apcRpnDataLayerName is used to set RPN data layer name
      and search RPN input data,if user has changed network struct, please
      make sure the data layer names are correct */
    s_stRfcnNnieParam.pstModel = &s_stRfcnModel.stModel;
    s_stRfcnSoftwareParam.apcRpnDataLayerName[0] = "rpn_cls_score";
    s_stRfcnSoftwareParam.apcRpnDataLayerName[1] = "rpn_bbox_pred";
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop_File();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Rfcn_ParamInit(&stNnieCfg, &s_stRfcnNnieParam, &s_stRfcnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Rfcn_ParamInit failed!\n");

    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop_File();
        return;
    }
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stRfcnNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop_File();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Rfcn_Proc(&s_stRfcnNnieParam, &s_stRfcnSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, END_RFCN_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Rfcn_Proc failed!\n");

    /* print result, this sample has 21 classes:
     class 0:background     class 1:plane           class 2:bicycle
     class 3:bird           class 4:boat            class 5:bottle
     class 6:bus            class 7:car             class 8:cat
     class 9:chair          class10:cow             class11:diningtable
     class 12:dog           class13:horse           class14:motorbike
     class 15:person        class16:pottedplant     class17:sheep
     class 18:sofa          class19:train           class20:tvmonitor */

    SAMPLE_SVP_TRACE_INFO("Rfcn result:\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Rfcn_Stop_File();
        return;
    }
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stRfcnSoftwareParam.stDstScore, &s_stRfcnSoftwareParam.stDstRoi,
        &s_stRfcnSoftwareParam.stClassRoiNum, f32PrintResultThresh);

END_RFCN_0:

    SAMPLE_SVP_NNIE_Rfcn_Deinit(&s_stRfcnNnieParam, &s_stRfcnSoftwareParam, &s_stRfcnModel);
    (HI_VOID)memset_s(&s_stRfcnModel, sizeof(s_stRfcnModel), 0, sizeof(s_stRfcnModel));
    (HI_VOID)memset_s(&s_stRfcnNnieParam, sizeof(s_stRfcnNnieParam), 0, sizeof(s_stRfcnNnieParam));
    (HI_VOID)memset_s(&s_stRfcnSoftwareParam, sizeof(s_stRfcnSoftwareParam), 0, sizeof(s_stRfcnSoftwareParam));
    SAMPLE_COMM_SVP_CheckSysExit();

    return;
}

/* function : SSD software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Ssd_SoftwareDeinit(SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if ((pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr != 0) && (pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr,
            pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr);
        pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Ssd Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Ssd_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Ssd_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Ssd_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Ssd software para init */
static HI_S32 SAMPLE_SVP_NNIE_Ssd_SoftwareInit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_U32 i = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32TmpBufTotalSize = 0;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    /* Set Conv Parameters */
    /* the SSD sample report resule is after permute operation,
     conv result is (C, H, W), after permute, the report node's
     (C1, H1, W1) is (H, W, C), the stride of report result is aligned according to C dim */
    for (i = 0; i < SAMPLE_SVP_NNIE_SSD_REPORT_NODE_NUM; i++) {
        pstSoftWareParam->au32ConvHeight[i] = pstNnieParam->pstModel->astSeg[0].astDstNode[i].unShape.stWhc.u32Chn;
        pstSoftWareParam->au32ConvWidth[i] = pstNnieParam->pstModel->astSeg[0].astDstNode[i].unShape.stWhc.u32Height;
        pstSoftWareParam->au32ConvChannel[i] = pstNnieParam->pstModel->astSeg[0].astDstNode[i].unShape.stWhc.u32Width;
        if (i % 2 == 1) { // only need to set the odd index parameter
            pstSoftWareParam->au32ConvStride[i / 2] =
                SAMPLE_SVP_NNIE_ALIGN16(pstSoftWareParam->au32ConvChannel[i] * sizeof(HI_U32)) / sizeof(HI_U32);
        }
    }

    /* Set PriorBox Parameters */
    /* The values of the following parameters are related to algorithm principles.
        For details, see related algorithms. */
    pstSoftWareParam->au32PriorBoxWidth[0] = 38;
    pstSoftWareParam->au32PriorBoxWidth[1] = 19;
    pstSoftWareParam->au32PriorBoxWidth[2] = 10;
    pstSoftWareParam->au32PriorBoxWidth[3] = 5;
    pstSoftWareParam->au32PriorBoxWidth[4] = 3;
    pstSoftWareParam->au32PriorBoxWidth[5] = 1;

    pstSoftWareParam->au32PriorBoxHeight[0] = 38;
    pstSoftWareParam->au32PriorBoxHeight[1] = 19;
    pstSoftWareParam->au32PriorBoxHeight[2] = 10;
    pstSoftWareParam->au32PriorBoxHeight[3] = 5;
    pstSoftWareParam->au32PriorBoxHeight[4] = 3;
    pstSoftWareParam->au32PriorBoxHeight[5] = 1;

    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;

    pstSoftWareParam->af32PriorBoxMinSize[0][0] = 30.0f;
    pstSoftWareParam->af32PriorBoxMinSize[1][0] = 60.0f;
    pstSoftWareParam->af32PriorBoxMinSize[2][0] = 111.0f;
    pstSoftWareParam->af32PriorBoxMinSize[3][0] = 162.0f;
    pstSoftWareParam->af32PriorBoxMinSize[4][0] = 213.0f;
    pstSoftWareParam->af32PriorBoxMinSize[5][0] = 264.0f;

    pstSoftWareParam->af32PriorBoxMaxSize[0][0] = 60.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[1][0] = 111.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[2][0] = 162.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[3][0] = 213.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[4][0] = 264.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[5][0] = 315.0f;

    pstSoftWareParam->u32MinSizeNum = 1;
    pstSoftWareParam->u32MaxSizeNum = 1;
    pstSoftWareParam->bFlip = HI_TRUE;
    pstSoftWareParam->bClip = HI_FALSE;

    pstSoftWareParam->au32InputAspectRatioNum[0] = 1;
    pstSoftWareParam->au32InputAspectRatioNum[1] = 2;
    pstSoftWareParam->au32InputAspectRatioNum[2] = 2;
    pstSoftWareParam->au32InputAspectRatioNum[3] = 2;
    pstSoftWareParam->au32InputAspectRatioNum[4] = 1;
    pstSoftWareParam->au32InputAspectRatioNum[5] = 1;

    pstSoftWareParam->af32PriorBoxAspectRatio[0][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[0][1] = 0;
    pstSoftWareParam->af32PriorBoxAspectRatio[1][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[1][1] = 3;
    pstSoftWareParam->af32PriorBoxAspectRatio[2][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[2][1] = 3;
    pstSoftWareParam->af32PriorBoxAspectRatio[3][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[3][1] = 3;
    pstSoftWareParam->af32PriorBoxAspectRatio[4][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[4][1] = 0;
    pstSoftWareParam->af32PriorBoxAspectRatio[5][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[5][1] = 0;

    pstSoftWareParam->af32PriorBoxStepWidth[0] = 8;
    pstSoftWareParam->af32PriorBoxStepWidth[1] = 16;
    pstSoftWareParam->af32PriorBoxStepWidth[2] = 32;
    pstSoftWareParam->af32PriorBoxStepWidth[3] = 64;
    pstSoftWareParam->af32PriorBoxStepWidth[4] = 100;
    pstSoftWareParam->af32PriorBoxStepWidth[5] = 300;

    pstSoftWareParam->af32PriorBoxStepHeight[0] = 8;
    pstSoftWareParam->af32PriorBoxStepHeight[1] = 16;
    pstSoftWareParam->af32PriorBoxStepHeight[2] = 32;
    pstSoftWareParam->af32PriorBoxStepHeight[3] = 64;
    pstSoftWareParam->af32PriorBoxStepHeight[4] = 100;
    pstSoftWareParam->af32PriorBoxStepHeight[5] = 300;

    pstSoftWareParam->f32Offset = 0.5f;

    pstSoftWareParam->as32PriorBoxVar[0] = (HI_S32)(0.1f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->as32PriorBoxVar[1] = (HI_S32)(0.1f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->as32PriorBoxVar[2] = (HI_S32)(0.2f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->as32PriorBoxVar[3] = (HI_S32)(0.2f * SAMPLE_SVP_NNIE_QUANT_BASE);

    /* Set Softmax Parameters */
    pstSoftWareParam->u32SoftMaxInHeight = 21;
    pstSoftWareParam->au32SoftMaxInChn[0] = 121296;
    pstSoftWareParam->au32SoftMaxInChn[1] = 45486;
    pstSoftWareParam->au32SoftMaxInChn[2] = 12600;
    pstSoftWareParam->au32SoftMaxInChn[3] = 3150;
    pstSoftWareParam->au32SoftMaxInChn[4] = 756;
    pstSoftWareParam->au32SoftMaxInChn[5] = 84;

    pstSoftWareParam->u32ConcatNum = 6;
    pstSoftWareParam->u32SoftMaxOutWidth = 1;
    pstSoftWareParam->u32SoftMaxOutHeight = 21;
    pstSoftWareParam->u32SoftMaxOutChn = 8732;

    /* Set DetectionOut Parameters */
    pstSoftWareParam->u32ClassNum = 21;
    pstSoftWareParam->u32TopK = 400;
    pstSoftWareParam->u32KeepTopK = 200;
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.3f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32ConfThresh = (HI_U32)(0.000245f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->au32DetectInputChn[0] = 23104;
    pstSoftWareParam->au32DetectInputChn[1] = 8664;
    pstSoftWareParam->au32DetectInputChn[2] = 2400;
    pstSoftWareParam->au32DetectInputChn[3] = 600;
    pstSoftWareParam->au32DetectInputChn[4] = 144;
    pstSoftWareParam->au32DetectInputChn[5] = 16;

    /* Malloc assist buffer memory */
    u32ClassNum = pstSoftWareParam->u32ClassNum;
    u32TotalSize = SAMPLE_SVP_NNIE_Ssd_GetResultTmpBuf(pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_RET(u32TotalSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, SAMPLE_SVP_NNIE_Ssd_GetResultTmpBuf failed!\n");
    u32DstRoiSize =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32TopK * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32TopK * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32TotalSize + u32DstRoiSize + u32DstScoreSize + u32ClassRoiNumSize;
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_SSD_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    /* set each tmp buffer addr */
    pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);

    pstSoftWareParam->stSoftMaxTmpBuf.u64PhyAddr = u64PhyAddr + pstSoftWareParam->stPriorBoxTmpBuf.u32Size;
    pstSoftWareParam->stSoftMaxTmpBuf.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + pstSoftWareParam->stPriorBoxTmpBuf.u32Size);

    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr =
        u64PhyAddr + pstSoftWareParam->stPriorBoxTmpBuf.u32Size + pstSoftWareParam->stSoftMaxTmpBuf.u32Size;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr +
        pstSoftWareParam->stPriorBoxTmpBuf.u32Size + pstSoftWareParam->stSoftMaxTmpBuf.u32Size);

    u32TmpBufTotalSize = pstSoftWareParam->stPriorBoxTmpBuf.u32Size + pstSoftWareParam->stSoftMaxTmpBuf.u32Size +
        pstSoftWareParam->stGetResultTmpBuf.u32Size;

    /* set result blob */
    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize;
    pstSoftWareParam->stDstRoi.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32TmpBufTotalSize);
    pstSoftWareParam->stDstRoi.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32TopK * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width =
        u32ClassNum * pstSoftWareParam->u32TopK * SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32TopK * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * pstSoftWareParam->u32TopK;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

/* function : Ssd init */
static HI_S32 SAMPLE_SVP_NNIE_Ssd_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software para */
    s32Ret = SAMPLE_SVP_NNIE_Ssd_SoftwareInit(pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Ssd_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Ssd_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Ssd_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}

static void SAMPLE_SVP_NNIE_Ssd_Stop(void)
{
    SAMPLE_SVP_NNIE_Ssd_Deinit(&s_stSsdNnieParam, &s_stSsdSoftwareParam, &s_stSsdModel);
    (HI_VOID)memset_s(&s_stSsdNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stSsdSoftwareParam, sizeof(SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stSsdModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show SSD sample(image 300x300 U8_C3) */
void SAMPLE_SVP_NNIE_Ssd(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/dog_bike_car_300x300.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_ssd_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    f32PrintResultThresh = 0.8f;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Ssd_Stop();
        return;
    }
    /* Ssd Load model */
    SAMPLE_SVP_TRACE_INFO("Ssd Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stSsdModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SSD_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* Ssd parameter initialization */
    /* Ssd parameters are set in SAMPLE_SVP_NNIE_Ssd_SoftwareInit,
      if user has changed net struct, please make sure the parameter settings in
      SAMPLE_SVP_NNIE_Ssd_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("Ssd parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Ssd_Stop();
        return;
    }
    s_stSsdNnieParam.pstModel = &s_stSsdModel.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Ssd_ParamInit(&stNnieCfg, &s_stSsdNnieParam, &s_stSsdSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SSD_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Ssd_ParamInit failed!\n");

    /* Fill src data */
    SAMPLE_SVP_TRACE_INFO("Ssd start!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Ssd_Stop();
        return;
    }

    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stSsdNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SSD_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process(process the 0-th segment) */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Ssd_Stop();
        return;
    }
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stSsdNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SSD_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");
    /* software process */
    /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Ssd_GetResult
     function's input data are correct */

    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Ssd_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Ssd_GetResult(&s_stSsdNnieParam, &s_stSsdSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, SSD_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Ssd_GetResult failed!\n");

    /* print result, this sample has 21 classes:
     class 0:background     class 1:plane           class 2:bicycle
     class 3:bird           class 4:boat            class 5:bottle
     class 6:bus            class 7:car             class 8:cat
     class 9:chair          class10:cow             class11:diningtable
     class 12:dog           class13:horse           class14:motorbike
     class 15:person        class16:pottedplant     class17:sheep
     class 18:sofa          class19:train           class20:tvmonitor */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Ssd_Stop();
        return;
    }
    SAMPLE_SVP_TRACE_INFO("Ssd result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stSsdSoftwareParam.stDstScore, &s_stSsdSoftwareParam.stDstRoi,
        &s_stSsdSoftwareParam.stClassRoiNum, f32PrintResultThresh);

SSD_FAIL_0:
    SAMPLE_SVP_NNIE_Ssd_Deinit(&s_stSsdNnieParam, &s_stSsdSoftwareParam, &s_stSsdModel);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : SSD sample signal handle */
void SAMPLE_SVP_NNIE_Ssd_HandleSig(void)
{
    g_stop_signal = HI_TRUE;
}

/* function : Yolov1 software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Yolov1_SoftwareDeinit(SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if ((pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr != 0) &&
        (pstSoftWareParam->stGetResultTmpBuf.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr,
            pstSoftWareParam->stGetResultTmpBuf.u64VirAddr);
        pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = 0;
        pstSoftWareParam->stDstRoi.u64PhyAddr = 0;
        pstSoftWareParam->stDstRoi.u64VirAddr = 0;
        pstSoftWareParam->stDstScore.u64PhyAddr = 0;
        pstSoftWareParam->stDstScore.u64VirAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64PhyAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Yolov1 Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Yolov1_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Yolov1_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Yolov1_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Yolov1 software para init */
static HI_S32 SAMPLE_SVP_NNIE_Yolov1_SoftwareInit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32BboxNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32TmpBufTotalSize = 0;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;
    pstSoftWareParam->u32BboxNumEachGrid = 2;
    pstSoftWareParam->u32ClassNum = 20;
    pstSoftWareParam->u32GridNumHeight = 7;
    pstSoftWareParam->u32GridNumWidth = 7;
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.5f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32ConfThresh = (HI_U32)(0.2f * SAMPLE_SVP_NNIE_QUANT_BASE);

    /* Malloc assist buffer memory */
    u32ClassNum = pstSoftWareParam->u32ClassNum + 1;
    u32BboxNum =
        pstSoftWareParam->u32BboxNumEachGrid * pstSoftWareParam->u32GridNumHeight * pstSoftWareParam->u32GridNumWidth;
    u32TmpBufTotalSize = SAMPLE_SVP_NNIE_Yolov1_GetResultTmpBuf(pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_RET(u32TmpBufTotalSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, SAMPLE_SVP_NNIE_Yolov1_GetResultTmpBuf failed!\n");
    u32DstRoiSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32TotalSize + u32DstRoiSize + u32DstScoreSize + u32ClassRoiNumSize + u32TmpBufTotalSize;
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_YOLOV1_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    /* set each tmp buffer addr */
    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);

    /* set result blob */
    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize;
    pstSoftWareParam->stDstRoi.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32TmpBufTotalSize);
    pstSoftWareParam->stDstRoi.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width = u32ClassNum * u32BboxNum * SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32TmpBufTotalSize + u32DstRoiSize);
    pstSoftWareParam->stDstScore.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * u32BboxNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

/* function : Yolov1 init */
static HI_S32 SAMPLE_SVP_NNIE_Yolov1_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software para */
    s32Ret = SAMPLE_SVP_NNIE_Yolov1_SoftwareInit(pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov1_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Yolov1_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov1_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}

static hi_void SAMPLE_SVP_NNIE_Yolov1_Stop(hi_void)
{
    SAMPLE_SVP_NNIE_Yolov1_Deinit(&s_stYolov1NnieParam, &s_stYolov1SoftwareParam, &s_stYolov1Model);
    (HI_VOID)memset_s(&s_stYolov1NnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stYolov1SoftwareParam, sizeof(SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_YOLOV1_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stYolov1Model, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show YOLOV1 sample(image 448x448 U8_C3) */
void SAMPLE_SVP_NNIE_Yolov1(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/dog_bike_car_448x448.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_yolov1_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    f32PrintResultThresh = 0.3f;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov1_Stop();
        return;
    }

    /* Yolov1 Load model */
    SAMPLE_SVP_TRACE_INFO("Yolov1 Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stYolov1Model);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV1_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* Yolov1 parameter initialization */
    /* Yolov1 software parameters are set in SAMPLE_SVP_NNIE_Yolov1_SoftwareInit,
      if user has changed net struct, please make sure the parameter settings in
      SAMPLE_SVP_NNIE_Yolov1_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("Yolov1 parameter initialization!\n");
    s_stYolov1NnieParam.pstModel = &s_stYolov1Model.stModel;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov1_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Yolov1_ParamInit(&stNnieCfg, &s_stYolov1NnieParam, &s_stYolov1SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV1_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov1_ParamInit failed!\n");

    /* Fill src data */
    SAMPLE_SVP_TRACE_INFO("Yolov1 start!\n");

    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov1_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stYolov1NnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV1_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process(process the 0-th segment) */
    stProcSegIdx.u32SegIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov1_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stYolov1NnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV1_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");
    /* software process */
    /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Yolov1_GetResult
     function input data are correct */

    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov1_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Yolov1_GetResult(&s_stYolov1NnieParam, &s_stYolov1SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV1_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov1_GetResult failed!\n");

    /* print result, this sample has 21 classes:
     class 0:background     class 1:plane           class 2:bicycle
     class 3:bird           class 4:boat            class 5:bottle
     class 6:bus            class 7:car             class 8:cat
     class 9:chair          class10:cow             class11:diningtable
     class 12:dog           class13:horse           class14:motorbike
     class 15:person        class16:pottedplant     class17:sheep
     class 18:sofa          class19:train           class20:tvmonitor */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov1_Stop();
        return;
    }
    SAMPLE_SVP_TRACE_INFO("Yolov1 result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stYolov1SoftwareParam.stDstScore, &s_stYolov1SoftwareParam.stDstRoi,
        &s_stYolov1SoftwareParam.stClassRoiNum, f32PrintResultThresh);

YOLOV1_FAIL_0:
    SAMPLE_SVP_NNIE_Yolov1_Deinit(&s_stYolov1NnieParam, &s_stYolov1SoftwareParam, &s_stYolov1Model);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : Yolov1 sample signal handle */
void SAMPLE_SVP_NNIE_Yolov1_HandleSig(void)
{
    g_stop_signal = HI_TRUE;
}

/*----------------------------------*/
/* function : Yolov2 software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Yolov2_SoftwareDeinit(SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if ((pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr != 0) &&
        (pstSoftWareParam->stGetResultTmpBuf.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr,
            pstSoftWareParam->stGetResultTmpBuf.u64VirAddr);
        pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = 0;
        pstSoftWareParam->stDstRoi.u64PhyAddr = 0;
        pstSoftWareParam->stDstRoi.u64VirAddr = 0;
        pstSoftWareParam->stDstScore.u64PhyAddr = 0;
        pstSoftWareParam->stDstScore.u64VirAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64PhyAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Yolov2 Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Yolov2_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Yolov2_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Yolov2_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Yolov2 software para init */
static HI_S32 SAMPLE_SVP_NNIE_Yolov2_SoftwareInit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32BboxNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32TmpBufTotalSize = 0;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    /* The values of the following parameters are related to algorithm principles.
        For details, see related algorithms. */
    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;
    pstSoftWareParam->u32BboxNumEachGrid = 5;
    pstSoftWareParam->u32ClassNum = 2;
    pstSoftWareParam->u32GridNumHeight = 13;
    pstSoftWareParam->u32GridNumWidth = 13;
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.3f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32ConfThresh = (HI_U32)(0.25f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32MaxRoiNum = 10;
    pstSoftWareParam->af32Bias[0] = 1.08;
    pstSoftWareParam->af32Bias[1] = 1.19;
    pstSoftWareParam->af32Bias[2] = 3.42;
    pstSoftWareParam->af32Bias[3] = 4.41;
    pstSoftWareParam->af32Bias[4] = 6.63;
    pstSoftWareParam->af32Bias[5] = 11.38;
    pstSoftWareParam->af32Bias[6] = 9.42;
    pstSoftWareParam->af32Bias[7] = 5.11;
    pstSoftWareParam->af32Bias[8] = 16.62;
    pstSoftWareParam->af32Bias[9] = 10.52;

    /* Malloc assist buffer memory */
    u32ClassNum = pstSoftWareParam->u32ClassNum + 1;
    u32BboxNum =
        pstSoftWareParam->u32BboxNumEachGrid * pstSoftWareParam->u32GridNumHeight * pstSoftWareParam->u32GridNumWidth;
    u32TmpBufTotalSize = SAMPLE_SVP_NNIE_Yolov2_GetResultTmpBuf(pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_RET(u32TmpBufTotalSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, SAMPLE_SVP_NNIE_Yolov2_GetResultTmpBuf failed!\n");
    u32DstRoiSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32TotalSize + u32DstRoiSize + u32DstScoreSize + u32ClassRoiNumSize + u32TmpBufTotalSize;
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_YOLOV2_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    /* set each tmp buffer addr */
    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);

    /* set result blob */
    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize;
    pstSoftWareParam->stDstRoi.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32TmpBufTotalSize);
    pstSoftWareParam->stDstRoi.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width = u32ClassNum * u32BboxNum * SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * u32BboxNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr) + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

/* function : Yolov1 init */
static HI_S32 SAMPLE_SVP_NNIE_Yolov2_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software para */
    s32Ret = SAMPLE_SVP_NNIE_Yolov2_SoftwareInit(pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov2_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Yolov2_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov2_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}
/*----------------------------------*/

/*=================================*/
static HI_S32 SampleSvpNnieYolov2SoftwareInit(SAMPLE_SVP_NNIE_CFG_S* pstCfg,
    SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S* pstSoftWareParam)
{
    HI_S32 s32Ret;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32BboxNum;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32DstRoiSize;
    HI_U32 u32DstScoreSize;
    HI_U32 u32ClassRoiNumSize;
    HI_U32 u32TmpBufTotalSize;
    HI_U64 u64PhyAddr = 0;
    HI_U8* pu8VirAddr = NULL;

    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;
    pstSoftWareParam->u32BboxNumEachGrid = 5; // 5: 2BboxNumEachGrid
    pstSoftWareParam->u32ClassNum = 2; // 5: class number
    pstSoftWareParam->u32GridNumHeight = 12; // 12: GridNumHeight
    pstSoftWareParam->u32GridNumWidth = 20; // 20: GridNumWidth
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.45f*SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32ConfThresh = (HI_U32)(0.4f*SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32MaxRoiNum = 10;  // 10: MaxRoiNum
    pstSoftWareParam->af32Bias[0] = 0.52; // 0.52: af32Bias[0] value
    pstSoftWareParam->af32Bias[1] = 0.61; // 0.61: af32Bias[1] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_2] = 1.05; // 1.05: af32Bias[ARRAY_SUBSCRIPT_2] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_3] = 1.12; // 1.12: af32Bias[ARRAY_SUBSCRIPT_3] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_4] = 1.85; // 1.85: af32Bias[ARRAY_SUBSCRIPT_4] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_5] = 2.05; // 2.05: af32Bias[ARRAY_SUBSCRIPT_5] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_6] = 4.63; // 4.63: af32Bias[ARRAY_SUBSCRIPT_6] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_7] = 4.49; // 4.49: af32Bias[ARRAY_SUBSCRIPT_7] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_8] = 7.15; // 7.15: af32Bias[ARRAY_SUBSCRIPT_8] value
    pstSoftWareParam->af32Bias[ARRAY_SUBSCRIPT_9] = 7.56; // 7.56: af32Bias[ARRAY_SUBSCRIPT_9] value

    /*
     * 申请辅助内存空间
     * Malloc assist buffer memory
     */
    u32ClassNum = pstSoftWareParam->u32ClassNum + 1;
    u32BboxNum = pstSoftWareParam->u32BboxNumEachGrid*pstSoftWareParam->u32GridNumHeight*
        pstSoftWareParam->u32GridNumWidth;
    u32TmpBufTotalSize = SAMPLE_SVP_NNIE_Yolov2_GetResultTmpBuf(pstSoftWareParam);
    u32DstRoiSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32TotalSize + u32DstRoiSize + u32DstScoreSize + u32ClassRoiNumSize + u32TmpBufTotalSize;
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_YOLOV2_INIT", NULL, (HI_U64*)&u64PhyAddr,
        (void**)&pu8VirAddr, u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void*)pu8VirAddr, u32TotalSize);

    /*
     * 设置每个tmp buffer地址
     * Set each tmp buffer addr
     */
    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = (HI_U64)((HI_UL)pu8VirAddr);

    /*
     * 设置结果blob
     * Set result blob
     */
    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize;
    pstSoftWareParam->stDstRoi.u64VirAddr = (HI_U64)((HI_UL)pu8VirAddr + u32TmpBufTotalSize);
    pstSoftWareParam->stDstRoi.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum *
        u32BboxNum * sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width = u32ClassNum *
        u32BboxNum*SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr = (HI_U64)((HI_UL)pu8VirAddr + u32TmpBufTotalSize + u32DstRoiSize);
    pstSoftWareParam->stDstScore.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * u32BboxNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum*u32BboxNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize +
        u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr = (HI_U64)((HI_UL)pu8VirAddr + u32TmpBufTotalSize +
        u32DstRoiSize + u32DstScoreSize);
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

static HI_S32 SampleSvpNnieYolov2SoftwareDeinit(SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S* pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if (pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr != 0 && pstSoftWareParam->stGetResultTmpBuf.u64VirAddr != 0) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr,
            pstSoftWareParam->stGetResultTmpBuf.u64VirAddr);
        pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = 0;
        pstSoftWareParam->stDstRoi.u64PhyAddr = 0;
        pstSoftWareParam->stDstRoi.u64VirAddr = 0;
        pstSoftWareParam->stDstScore.u64PhyAddr = 0;
        pstSoftWareParam->stDstScore.u64VirAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64PhyAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64VirAddr = 0;
    }
    return s32Ret;
}

static HI_S32 SampleSvpNnieYolov2Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S* pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /*
     * SAMPLE_SVP_NNIE_PARAM_S参数去初始化
     * Hardware param deinit
     */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(HI_SUCCESS != s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /*
     * SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S参数去初始化
     * Software deinit
     */
    if (pstSoftWareParam != NULL) {
        s32Ret = SampleSvpNnieYolov2SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(HI_SUCCESS != s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SampleSvpNnieYolov2SoftwareDeinit failed!\n");
    }

    /*
     * SAMPLE_SVP_NNIE_MODEL_S参数去初始化
     * Model deinit
     */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(HI_SUCCESS != s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

static HI_S32 SampleSvpNnieYolov2ParamInit(SAMPLE_SVP_NNIE_CFG_S* pstCfg,
    SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S* pstSoftWareParam)
{
    HI_S32 s32Ret;
    /*
     * 初始化hardware参数
     * Init hardware param
     */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /*
     * 初始化software参数
     * Init software param
     */
    s32Ret = SampleSvpNnieYolov2SoftwareInit(pstCfg, pstNnieParam,
        pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov1_SoftwareInit failed!\n", s32Ret);
    return s32Ret;
INIT_FAIL_0:
    s32Ret = SampleSvpNnieYolov2Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov1_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}
/*=================================*/

static void SAMPLE_SVP_NNIE_Yolov2_Stop(void)
{
    SAMPLE_SVP_NNIE_Yolov2_Deinit(&s_stYolov2NnieParam, &s_stYolov2SoftwareParam, &s_stYolov2Model);
    (HI_VOID)memset_s(&s_stYolov2NnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stYolov2SoftwareParam, sizeof(SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stYolov2Model, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show YOLOV2 sample(image 416x416 U8_C3) */
void SAMPLE_SVP_NNIE_Yolov2(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/2008_001538_640x384.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/resnet18_best_inst.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    f32PrintResultThresh = 0.2f;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov2_Stop();
        return;
    }

    /* Yolov2 Load model */
    SAMPLE_SVP_TRACE_INFO("Yolov2 Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stYolov2Model);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV2_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* Yolov2 parameter initialization */
    /* Yolov2 software parameters are set in SAMPLE_SVP_NNIE_Yolov2_SoftwareInit,
      if user has changed net struct, please make sure the parameter settings in
      SAMPLE_SVP_NNIE_Yolov2_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("Yolov2 parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov2_Stop();
        return;
    }
    s_stYolov2NnieParam.pstModel = &s_stYolov2Model.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Yolov2_ParamInit(&stNnieCfg, &s_stYolov2NnieParam, &s_stYolov2SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV2_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov2_ParamInit failed!\n");

    /* Fill src data */
    SAMPLE_SVP_TRACE_INFO("Yolov2 start!\n");
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov2_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stYolov2NnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV2_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process(process the 0-th segment) */
    stProcSegIdx.u32SegIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov2_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stYolov2NnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV2_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* Software process */
    /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Yolov2_GetResult
     function input data are correct */

    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov2_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Yolov2_GetResult(&s_stYolov2NnieParam, &s_stYolov2SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV2_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov2_GetResult failed!\n");

    /* print result, this sample has 6 classes:
     class 0:background     class 1:Carclass           class 2:Vanclass
     class 3:Truckclass     class 4:Pedestrianclass    class 5:Cyclist */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov2_Stop();
        return;
    }
    SAMPLE_SVP_TRACE_INFO("Yolov2 result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stYolov2SoftwareParam.stDstScore, &s_stYolov2SoftwareParam.stDstRoi,
        &s_stYolov2SoftwareParam.stClassRoiNum, f32PrintResultThresh);

YOLOV2_FAIL_0:
    SAMPLE_SVP_NNIE_Yolov2_Deinit(&s_stYolov2NnieParam, &s_stYolov2SoftwareParam, &s_stYolov2Model);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : Yolov2 sample signal handle */
void SAMPLE_SVP_NNIE_Yolov2_HandleSig(void)
{
    g_stop_signal = HI_TRUE;
}

/* function : Yolov3 software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Yolov3_SoftwareDeinit(SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if ((pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr != 0) &&
        (pstSoftWareParam->stGetResultTmpBuf.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr,
            pstSoftWareParam->stGetResultTmpBuf.u64VirAddr);
        pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = 0;
        pstSoftWareParam->stDstRoi.u64PhyAddr = 0;
        pstSoftWareParam->stDstRoi.u64VirAddr = 0;
        pstSoftWareParam->stDstScore.u64PhyAddr = 0;
        pstSoftWareParam->stDstScore.u64VirAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64PhyAddr = 0;
        pstSoftWareParam->stClassRoiNum.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Yolov3 Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Yolov3_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Yolov3_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_Yolov3_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Yolov3 software para init */
static HI_S32 SAMPLE_SVP_NNIE_Yolov3_SoftwareInit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32TmpBufTotalSize = 0;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    /* The values of the following parameters are related to algorithm principles.
        For details, see related algorithms. */
    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;
    pstSoftWareParam->u32BboxNumEachGrid = 3;
    pstSoftWareParam->u32ClassNum = 80;
    pstSoftWareParam->au32GridNumHeight[0] = 13;
    pstSoftWareParam->au32GridNumHeight[1] = 26;
    pstSoftWareParam->au32GridNumHeight[2] = 52;
    pstSoftWareParam->au32GridNumWidth[0] = 13;
    pstSoftWareParam->au32GridNumWidth[1] = 26;
    pstSoftWareParam->au32GridNumWidth[2] = 52;
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.3f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32ConfThresh = (HI_U32)(0.5f * SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32MaxRoiNum = 10;
    pstSoftWareParam->af32Bias[0][0] = 116;
    pstSoftWareParam->af32Bias[0][1] = 90;
    pstSoftWareParam->af32Bias[0][2] = 156;
    pstSoftWareParam->af32Bias[0][3] = 198;
    pstSoftWareParam->af32Bias[0][4] = 373;
    pstSoftWareParam->af32Bias[0][5] = 326;
    pstSoftWareParam->af32Bias[1][0] = 30;
    pstSoftWareParam->af32Bias[1][1] = 61;
    pstSoftWareParam->af32Bias[1][2] = 62;
    pstSoftWareParam->af32Bias[1][3] = 45;
    pstSoftWareParam->af32Bias[1][4] = 59;
    pstSoftWareParam->af32Bias[1][5] = 119;
    pstSoftWareParam->af32Bias[2][0] = 10;
    pstSoftWareParam->af32Bias[2][1] = 13;
    pstSoftWareParam->af32Bias[2][2] = 16;
    pstSoftWareParam->af32Bias[2][3] = 30;
    pstSoftWareParam->af32Bias[2][4] = 33;
    pstSoftWareParam->af32Bias[2][5] = 23;

    /* Malloc assist buffer memory */
    u32ClassNum = pstSoftWareParam->u32ClassNum + 1;

    SAMPLE_SVP_CHECK_EXPR_RET(SAMPLE_SVP_NNIE_YOLOV3_REPORT_BLOB_NUM != pstNnieParam->pstModel->astSeg[0].u16DstNum,
        HI_FAILURE, SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,pstNnieParam->pstModel->astSeg[0].u16DstNum(%d) should be %d!\n",
        pstNnieParam->pstModel->astSeg[0].u16DstNum, SAMPLE_SVP_NNIE_YOLOV3_REPORT_BLOB_NUM);
    u32TmpBufTotalSize = SAMPLE_SVP_NNIE_Yolov3_GetResultTmpBuf(pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_RET(u32TmpBufTotalSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, SAMPLE_SVP_NNIE_Yolov3_GetResultTmpBuf failed!\n");
    u32DstRoiSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32) *
        SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32TotalSize + u32DstRoiSize + u32DstScoreSize + u32ClassRoiNumSize + u32TmpBufTotalSize;
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_YOLOV3_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    /* set each tmp buffer addr */
    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr);

    /* set result blob */
    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize;
    pstSoftWareParam->stDstRoi.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr + u32TmpBufTotalSize);
    pstSoftWareParam->stDstRoi.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum *
        sizeof(HI_U32) * SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width =
        u32ClassNum * pstSoftWareParam->u32MaxRoiNum * SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr + u32TmpBufTotalSize + u32DstRoiSize);
    pstSoftWareParam->stDstScore.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * pstSoftWareParam->u32MaxRoiNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr = u64PhyAddr + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr =
        SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64, pu8VirAddr + u32TmpBufTotalSize + u32DstRoiSize + u32DstScoreSize);
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

/* function : Yolov3 init */
static HI_S32 SAMPLE_SVP_NNIE_Yolov3_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software para */
    s32Ret = SAMPLE_SVP_NNIE_Yolov3_SoftwareInit(pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov3_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Yolov3_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Yolov3_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}

static hi_void SAMPLE_SVP_NNIE_Yolov3_Stop(hi_void)
{
    SAMPLE_SVP_NNIE_Yolov3_Deinit(&s_stYolov3NnieParam, &s_stYolov3SoftwareParam, &s_stYolov3Model);
    (HI_VOID)memset_s(&s_stYolov3NnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stYolov3SoftwareParam, sizeof(SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stYolov3Model, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show YOLOV3 sample(image 416x416 U8_C3) */
void SAMPLE_SVP_NNIE_Yolov3(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/dog_bike_car_416x416.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_yolov3_cycle.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };//初始化结构体
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    f32PrintResultThresh = 0.8f;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.u32MaxInputNum = u32PicNum; // max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov3_Stop();
        return;
    }

    /* Yolov3 Load model */
    SAMPLE_SVP_TRACE_INFO("Yolov3 Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stYolov3Model);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV3_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    
    clock_t start,end;
    double time_used;
    start = clock();
    /* Yolov3 parameter initialization */
    /* Yolov3 software parameters are set in SAMPLE_SVP_NNIE_Yolov3_SoftwareInit,
      if user has changed net struct, please make sure the parameter settings in
      SAMPLE_SVP_NNIE_Yolov3_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("Yolov3 parameter initialization!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov3_Stop();
        return;
    }
    s_stYolov3NnieParam.pstModel = &s_stYolov3Model.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Yolov3_ParamInit(&stNnieCfg, &s_stYolov3NnieParam, &s_stYolov3SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV3_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov3_ParamInit failed!\n");


    /* Fill src data */
    SAMPLE_SVP_TRACE_INFO("Yolov3 start!\n");
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov3_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stYolov3NnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV3_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");

    /* NNIE process(process the 0-th segment) */
    stProcSegIdx.u32SegIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov3_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stYolov3NnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV3_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* Software process */
    /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Yolov3_GetResult
     function input data are correct */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Yolov3_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Yolov3_GetResult(&s_stYolov3NnieParam, &s_stYolov3SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, YOLOV3_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov3_GetResult failed!\n");

    /* print result, this sample has 81 classes:
     class 0:background      class 1:person       class 2:bicycle         class 3:car            class 4:motorbike class
     5:aeroplane class 6:bus             class 7:train        class 8:truck           class 9:boat           class
     10:traffic light class 11:fire hydrant   class 12:stop sign   class 13:parking meter  class 14:bench         class
     15:bird class 16:cat            class 17:dog         class 18:horse          class 19:sheep         class 20:cow
     class 21:elephant       class 22:bear        class 23:zebra          class 24:giraffe       class 25:backpack
     class 26:umbrella       class 27:handbag     class 28:tie            class 29:suitcase      class 30:frisbee
     class 31:skis           class 32:snowboard   class 33:sports ball    class 34:kite          class 35:baseball bat
     class 36:baseball glove class 37:skateboard  class 38:surfboard      class 39:tennis racket class 40bottle
     class 41:wine glass     class 42:cup         class 43:fork           class 44:knife         class 45:spoon
     class 46:bowl           class 47:banana      class 48:apple          class 49:sandwich      class 50orange
     class 51:broccoli       class 52:carrot      class 53:hot dog        class 54:pizza         class 55:donut
     class 56:cake           class 57:chair       class 58:sofa           class 59:pottedplant   class 60bed
     class 61:diningtable    class 62:toilet      class 63:vmonitor       class 64:laptop        class 65:mouse
     class 66:remote         class 67:keyboard    class 68:cell phone     class 69:microwave     class 70:oven
     class 71:toaster        class 72:sink        class 73:refrigerator   class 74:book          class 75:clock
     class 76:vase           class 77:scissors    class 78:teddy bear     class 79:hair drier    class 80:toothbrush */
    SAMPLE_SVP_TRACE_INFO("Yolov3 result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stYolov3SoftwareParam.stDstScore, &s_stYolov3SoftwareParam.stDstRoi,
        &s_stYolov3SoftwareParam.stClassRoiNum, f32PrintResultThresh);

    end = clock();
    time_used = ((double)(end-start))/CLOCKS_PER_SEC;
    printf("process time : %f ms\n",time_used/1000);

YOLOV3_FAIL_0:
    SAMPLE_SVP_NNIE_Yolov3_Deinit(&s_stYolov3NnieParam, &s_stYolov3SoftwareParam, &s_stYolov3Model);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : Yolov3 sample signal handle */
void SAMPLE_SVP_NNIE_Yolov3_HandleSig(void)
{
    g_stop_signal = HI_TRUE;
}

/* function : Lstm Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Lstm_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParamm, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParamm != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParamm);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Lstm init */
static HI_S32 SAMPLE_SVP_NNIE_Lstm_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstNnieCfg, SAMPLE_SVP_NNIE_PARAM_S *pstLstmPara)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware para */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstNnieCfg, pstLstmPara);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);
    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Lstm_Deinit(pstLstmPara, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Lstm_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}
static hi_void SAMPLE_SVP_NNIE_Lstm_Stop(hi_void)
{
    SAMPLE_SVP_NNIE_Lstm_Deinit(&s_stLstmNnieParam, &s_stLstmModel);
    (HI_VOID)memset_s(&s_stLstmNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stLstmModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show Lstm sample(vector) */
void SAMPLE_SVP_NNIE_Lstm(void)
{
    const HI_CHAR *apcSrcFile[3] = {"./data/nnie_image/vector/Seq.SEQ_S32",
                                    "./data/nnie_image/vector/Vec1.VEC_S32",
                                    "./data/nnie_image/vector/Vec2.VEC_S32"};
    const HI_CHAR *pchModelName = "./data/nnie_model/recurrent/lstm_3_3.wk";
    HI_U8 *pu8VirAddr = NULL;
    HI_U32 u32SegNum = 0;
    HI_U32 u32Step = 0;
    HI_U32 u32Offset = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 i = 0, j = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    stNnieCfg.u32MaxInputNum = 16; // max input data num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core
    u32Step = 20;                               // time step
    g_stop_signal = HI_FALSE;

    /* Sys init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Lstm_Stop();
        return;
    }

    /* Lstm Load model */
    SAMPLE_SVP_TRACE_INFO("Lstm Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pchModelName, &s_stLstmModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, LSTM_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /* Lstm step initialization */
    u32SegNum = s_stLstmModel.stModel.u32NetSegNum;
    u32TotalSize = stNnieCfg.u32MaxInputNum * sizeof(HI_S32) * u32SegNum * 2;
    s32Ret = SAMPLE_COMM_SVP_MallocMem("SVP_NNIE_STEP", NULL, (HI_U64 *)&s_stLstmNnieParam.stStepBuf.u64PhyAddr,
        (void **)&pu8VirAddr, u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, LSTM_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    /* Get step virtual addr */
    s_stLstmNnieParam.stStepBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);
    for (i = 0; i < u32SegNum * SAMPLE_SVP_NNIE_EACH_SEG_STEP_ADDR_NUM; i++) {
        stNnieCfg.au64StepVirAddr[i] =
            s_stLstmNnieParam.stStepBuf.u64VirAddr + i * stNnieCfg.u32MaxInputNum * sizeof(HI_S32);
    }
    /* Set step value, in this sample, the step values are set to be 20,
    if user has changed input network, please set correct step
    values according to the input network */
    for (i = 0; i < u32SegNum; i++) {
        u32Offset = i * SAMPLE_SVP_NNIE_EACH_SEG_STEP_ADDR_NUM;
        for (j = 0; j < stNnieCfg.u32MaxInputNum; j++) {
            *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, stNnieCfg.au64StepVirAddr[u32Offset]) + j) =
                u32Step; // step of input x_t
            *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, stNnieCfg.au64StepVirAddr[u32Offset + 1]) + j) =
                u32Step; // step of output h_t
        }
    }

    /* Lstm parameter initialization */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Lstm_Stop();
        return;
    }
    SAMPLE_SVP_TRACE_INFO("Lstm parameter initialization!\n");
    s_stLstmNnieParam.pstModel = &(s_stLstmModel.stModel);
    s32Ret = SAMPLE_SVP_NNIE_Lstm_ParamInit(&stNnieCfg, &s_stLstmNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, LSTM_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Lstm_ParamInit failed!\n");

    /* Fill src data, in this sample, the 0-th seg is lstm network,if user has
     changed input network,please make sure the value of stInputDataIdx.u32SegIdx
     is correct */
    SAMPLE_SVP_TRACE_INFO("Lstm start!\n");
    stInputDataIdx.u32SegIdx = 0;
    for (i = 0;
        i < (s_stLstmNnieParam.pstModel->astSeg[stInputDataIdx.u32SegIdx].u16SrcNum) && (g_stop_signal == HI_FALSE);
        i++) {
        stNnieCfg.pszPic = apcSrcFile[i];
        stInputDataIdx.u32NodeIdx = i;
        s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stLstmNnieParam, &stInputDataIdx);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, LSTM_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_FillSrcData failed!\n");
    }
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Lstm_Stop();
        return;
    }

    /* NNIE process(process the 0-th segment) */
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    stProcSegIdx.u32SegIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stLstmNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, LSTM_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* print report result */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Lstm_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_PrintReportResult(&s_stLstmNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, LSTM_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_PrintReportResult failed!\n");

    SAMPLE_SVP_TRACE_INFO("Lstm is successfully processed!\n");

LSTM_FAIL_0:
    SAMPLE_SVP_NNIE_Lstm_Deinit(&s_stLstmNnieParam, &s_stLstmModel);
    SAMPLE_COMM_SVP_CheckSysExit();
}

/* function : Lstm sample signal handle */
void SAMPLE_SVP_NNIE_Lstm_HandleSig(void)
{
    g_stop_signal = HI_TRUE;
}

/* function : Pavnet software deinit */
static HI_S32 SAMPLE_SVP_NNIE_Pvanet_SoftwareDeinit(SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(pstSoftWareParam == NULL, HI_INVALID_VALUE, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error, pstSoftWareParam can't be NULL!\n");
    if ((pstSoftWareParam->stRpnTmpBuf.u64PhyAddr != 0) && (pstSoftWareParam->stRpnTmpBuf.u64VirAddr != 0)) {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stRpnTmpBuf.u64PhyAddr, pstSoftWareParam->stRpnTmpBuf.u64VirAddr);
        pstSoftWareParam->stRpnTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stRpnTmpBuf.u64VirAddr = 0;
    }
    return s32Ret;
}

/* function : Pvanet Deinit */
static HI_S32 SAMPLE_SVP_NNIE_Pvanet_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam, SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* hardware deinit */
    if (pstNnieParam != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /* software deinit */
    if (pstSoftWareParam != NULL) {
        s32Ret = SAMPLE_SVP_NNIE_Pvanet_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_FasterRcnn_SoftwareDeinit failed!\n");
    }
    /* model deinit */
    if (pstNnieModel != NULL) {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}

/* function : Pvanet software para init */
static HI_S32 SAMPLE_SVP_NNIE_Pvanet_SoftwareInit(SAMPLE_SVP_NNIE_CFG_S *pstCfg, SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
    SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RpnTmpBufSize = 0;
    HI_U32 u32RpnBboxBufSize = 0;
    HI_U32 u32GetResultTmpBufSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U64 u64PhyAddr = 0;
    HI_U8 *pu8VirAddr = NULL;

    /* RPN parameter init */
    /* The values of the following parameters are related to algorithm principles.
        For details, see related algorithms. */
    pstSoftWareParam->u32MaxRoiNum = pstCfg->u32MaxRoiNum;
    pstSoftWareParam->u32ClassNum = 21;
    pstSoftWareParam->u32NumRatioAnchors = 7;
    pstSoftWareParam->u32NumScaleAnchors = 6;
    pstSoftWareParam->au32Ratios[0] = (HI_S32)(0.333 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->au32Ratios[1] = (HI_S32)(0.5 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->au32Ratios[2] = (HI_S32)(0.667 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->au32Ratios[3] = (HI_S32)(1 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->au32Ratios[4] = (HI_S32)(1.5 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->au32Ratios[5] = (HI_S32)(2 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->au32Ratios[6] = (HI_S32)(3 * SAMPLE_SVP_QUANT_BASE);

    pstSoftWareParam->au32Scales[0] = 2 * SAMPLE_SVP_QUANT_BASE;
    pstSoftWareParam->au32Scales[1] = 3 * SAMPLE_SVP_QUANT_BASE;
    pstSoftWareParam->au32Scales[2] = 5 * SAMPLE_SVP_QUANT_BASE;
    pstSoftWareParam->au32Scales[3] = 9 * SAMPLE_SVP_QUANT_BASE;
    pstSoftWareParam->au32Scales[4] = 16 * SAMPLE_SVP_QUANT_BASE;
    pstSoftWareParam->au32Scales[5] = 32 * SAMPLE_SVP_QUANT_BASE;

    /* set origin image height & width from src[0] shape */
    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;

    pstSoftWareParam->u32MinSize = 16;
    pstSoftWareParam->u32SpatialScale = (HI_U32)(0.0625 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->u32NmsThresh = (HI_U32)(0.7 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->u32FilterThresh = 0;
    pstSoftWareParam->u32ValidNmsThresh = (HI_U32)(0.3 * SAMPLE_SVP_QUANT_BASE);
    pstSoftWareParam->u32NumBeforeNms = 12000;
    pstSoftWareParam->u32MaxRoiNum = 200;

    for (i = 0; i < pstSoftWareParam->u32ClassNum; i++) {
        pstSoftWareParam->au32ConfThresh[i] = 1;
    }

    pstSoftWareParam->stRpnBbox.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Height = pstCfg->u32MaxRoiNum;
    pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Width = SAMPLE_SVP_COORDI_NUM;
    pstSoftWareParam->stRpnBbox.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(SAMPLE_SVP_COORDI_NUM * sizeof(HI_U32));
    pstSoftWareParam->stRpnBbox.u32Num = 1;
    for (i = 0; i < SAMPLE_SVP_NNIE_SEGMENT_NUM; i++) {
        for (j = 0; j < pstNnieParam->pstModel->astSeg[0].u16DstNum; j++) {
            if (strncmp(pstNnieParam->pstModel->astSeg[0].astDstNode[j].szName,
                pstSoftWareParam->apcRpnDataLayerName[i], SVP_NNIE_NODE_NAME_LEN) == 0) {
                pstSoftWareParam->aps32Conv[i] =
                    SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32, pstNnieParam->astSegData[0].astDst[j].u64VirAddr);
                pstSoftWareParam->au32ConvHeight[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Height;
                pstSoftWareParam->au32ConvWidth[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Width;
                pstSoftWareParam->au32ConvChannel[i] =
                    pstNnieParam->pstModel->astSeg[0].astDstNode[j].unShape.stWhc.u32Chn;
                break;
            }
        }
        SAMPLE_SVP_CHECK_EXPR_RET((j == pstNnieParam->pstModel->astSeg[0].u16DstNum), HI_FAILURE,
            SAMPLE_SVP_ERR_LEVEL_ERROR, "Error,failed to find report node %s!\n",
            pstSoftWareParam->apcRpnDataLayerName[i]);
        if (i == 0) {
            pstSoftWareParam->u32ConvStride = pstNnieParam->astSegData[0].astDst[j].u32Stride;
        }
    }
    /* calculate software mem size */
    u32ClassNum = pstSoftWareParam->u32ClassNum;
    u32RpnTmpBufSize = SAMPLE_SVP_NNIE_RpnTmpBufSize(pstSoftWareParam->u32NumRatioAnchors,
        pstSoftWareParam->u32NumScaleAnchors, pstSoftWareParam->au32ConvHeight[0], pstSoftWareParam->au32ConvWidth[0]);
    SAMPLE_SVP_CHECK_EXPR_RET(u32RpnTmpBufSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_RpnTmpBufSize failed!\n");
    u32RpnTmpBufSize = SAMPLE_SVP_NNIE_ALIGN16(u32RpnTmpBufSize);
    u32RpnBboxBufSize = pstSoftWareParam->stRpnBbox.u32Num * pstSoftWareParam->stRpnBbox.unShape.stWhc.u32Height *
        pstSoftWareParam->stRpnBbox.u32Stride;
    u32GetResultTmpBufSize = SAMPLE_SVP_NNIE_Pvanet_GetResultTmpBufSize(pstCfg->u32MaxRoiNum, u32ClassNum);
    SAMPLE_SVP_CHECK_EXPR_RET(u32GetResultTmpBufSize == 0, HI_ERR_SVP_NNIE_ILLEGAL_PARAM, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Pvanet_GetResultTmpBufSize failed!\n");
    u32GetResultTmpBufSize = SAMPLE_SVP_NNIE_ALIGN16(u32GetResultTmpBufSize);
    u32DstRoiSize =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstCfg->u32MaxRoiNum * sizeof(HI_U32) * SAMPLE_SVP_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstCfg->u32MaxRoiNum * sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    u32TotalSize = u32RpnTmpBufSize + u32RpnBboxBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize +
        u32ClassRoiNumSize;

    /* malloc mem */
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_Pvanet_INIT", NULL, (HI_U64 *)&u64PhyAddr, (void **)&pu8VirAddr,
        u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,Malloc memory failed!\n");
    (HI_VOID)memset_s(pu8VirAddr, u32TotalSize, 0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr, (void *)pu8VirAddr, u32TotalSize);

    /* set addr */
    pstSoftWareParam->stRpnTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stRpnTmpBuf.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr);
    pstSoftWareParam->stRpnTmpBuf.u32Size = u32RpnTmpBufSize;

    pstSoftWareParam->stRpnBbox.u64PhyAddr = u64PhyAddr + u32RpnTmpBufSize;
    pstSoftWareParam->stRpnBbox.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr) + u32RpnTmpBufSize;

    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnBboxBufSize + u32RpnTmpBufSize);
    pstSoftWareParam->stGetResultTmpBuf.u32Size = u32GetResultTmpBufSize;

    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize;
    pstSoftWareParam->stDstRoi.u64VirAddr =
        (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize);
    pstSoftWareParam->stDstRoi.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32) * SAMPLE_SVP_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width =
        u32ClassNum * pstSoftWareParam->u32MaxRoiNum * SAMPLE_SVP_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr =
        u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr = (HI_U64)((HI_UINTPTR_T)pu8VirAddr + u32RpnBboxBufSize + u32RpnTmpBufSize +
        u32GetResultTmpBufSize + u32DstRoiSize);
    pstSoftWareParam->stDstScore.u32Stride =
        SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * pstSoftWareParam->u32MaxRoiNum * sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum * pstSoftWareParam->u32MaxRoiNum;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr =
        u64PhyAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr = SAMPLE_SVP_NNIE_CONVERT_PTR_TO_ADDR(HI_U64,
        pu8VirAddr + u32RpnBboxBufSize + u32RpnTmpBufSize + u32GetResultTmpBufSize + u32DstRoiSize + u32DstScoreSize);
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum * sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}

/* function : Pvanet parameter initialization */
static HI_S32 SAMPLE_SVP_NNIE_Pvanet_ParamInit(SAMPLE_SVP_NNIE_CFG_S *pstFasterRcnnCfg,
    SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S *pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /* init hardware parameter */
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstFasterRcnnCfg, pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n", s32Ret);

    /* init software parameter */
    s32Ret = SAMPLE_SVP_NNIE_Pvanet_SoftwareInit(pstFasterRcnnCfg, pstNnieParam, pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, INIT_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_Pvanet_SoftwareInit failed!\n", s32Ret);

    return s32Ret;
INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_FasterRcnn_Deinit(pstNnieParam, pstSoftWareParam, NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(s32Ret != HI_SUCCESS, s32Ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_FasterRcnn_Deinit failed!\n", s32Ret);
    return HI_FAILURE;
}
static hi_void SAMPLE_SVP_NNIE_Pvanet_Stop(hi_void)
{
    SAMPLE_SVP_NNIE_FasterRcnn_Deinit(&s_stPvanetNnieParam, &s_stPvanetSoftwareParam, &s_stPvanetModel);
    (HI_VOID)memset_s(&s_stPvanetNnieParam, sizeof(SAMPLE_SVP_NNIE_PARAM_S), 0, sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    (HI_VOID)memset_s(&s_stPvanetSoftwareParam, sizeof(SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S), 0,
        sizeof(SAMPLE_SVP_NNIE_FASTERRCNN_SOFTWARE_PARAM_S));
    (HI_VOID)memset_s(&s_stPvanetModel, sizeof(SAMPLE_SVP_NNIE_MODEL_S), 0, sizeof(SAMPLE_SVP_NNIE_MODEL_S));
    SAMPLE_COMM_SVP_CheckSysExit();
    printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
}

/* function : show Pvanet fasterRcnn sample(image 224x224 U8_C3) */
void SAMPLE_SVP_NNIE_Pvanet(void)
{
    const HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/horse_dog_car_person_224x224.bgr";
    const HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_fasterrcnn_pvanet_inst.wk";
    const HI_U32 u32PicNum = 1;
    HI_FLOAT f32PrintResultThresh = 0.0f;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 i = 0;
    SAMPLE_SVP_NNIE_CFG_S stNnieCfg = { 0 };
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = { 0 };
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = { 0 };

    /* Set configuration parameter */
    s_enNetType = SAMPLE_SVP_NNIE_PVANET_FASTER_RCNN;
    f32PrintResultThresh = 0.8f;
    stNnieCfg.u32MaxInputNum = u32PicNum;
    stNnieCfg.u32MaxRoiNum = 200;
    stNnieCfg.pszPic = pcSrcFile;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0;
    stNnieCfg.aenNnieCoreId[1] = SVP_NNIE_ID_0;
    g_stop_signal = HI_FALSE;

    /* Sys_init */
    s32Ret = SAMPLE_COMM_SVP_CheckSysInit();
    SAMPLE_SVP_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_CheckSysInit failed!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Pvanet_Stop();
        return;
    }

    /* FasterRcnn Load model */
    SAMPLE_SVP_TRACE_INFO("Pvanet load Model!!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName, &s_stPvanetModel);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "ERROR, SAMPLE_COMM_SVP_NNIE_LoadModel failed\n");

    /* Pvanet para init */
    /* apcRpnDataLayerName is used to set RPN data layer name
       and search RPN input data,if user has changed network struct, please
       make sure the data layer names are correct */
    /* Pvanet parameters are set in SAMPLE_SVP_NNIE_Pvanet_SoftwareInit,
     if user has changed network struct, please make sure the parameter settings in
     SAMPLE_SVP_NNIE_FasterRcnn_SoftwareInit function are correct */
    SAMPLE_SVP_TRACE_INFO("Pvanet parameter initialization!\n");
    s_stPvanetNnieParam.pstModel = &s_stPvanetModel.stModel;
    s_stPvanetSoftwareParam.apcRpnDataLayerName[0] = "rpn_cls_score";
    s_stPvanetSoftwareParam.apcRpnDataLayerName[1] = "rpn_bbox_pred";
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Pvanet_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Pvanet_ParamInit(&stNnieCfg, &s_stPvanetNnieParam, &s_stPvanetSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Pvanet_ParamInit failed!\n");

    /* Fill 0-th input node of 0-th seg */
    SAMPLE_SVP_TRACE_INFO("Pvanet start!\n");
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Pvanet_Stop();
        return;
    }
    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    s32Ret = SAMPLE_SVP_NNIE_FillSrcData(&stNnieCfg, &s_stPvanetNnieParam, &stInputDataIdx);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "ERROR, SAMPLE_SVP_NNIE_FillSrcData Failed!!\n");

    /* NNIE process 0-th seg */
    stProcSegIdx.u32SegIdx = 0;
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Pvanet_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stPvanetNnieParam, &stInputDataIdx, &stProcSegIdx, HI_TRUE);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Forward failed!\n");

    /* Do RPN */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Pvanet_Stop();
        return;
    }
    s32Ret = SAMPLE_SVP_NNIE_Pvanet_Rpn(&s_stPvanetSoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Pvanet_Rpn failed!\n");
    if (s_stPvanetSoftwareParam.stRpnBbox.unShape.stWhc.u32Height != 0 && g_stop_signal == HI_FALSE) {
        /* NNIE process 1-th seg, the input conv data comes from 0-th seg's 0-th report node,
          the input roi comes from RPN results */
        stInputDataIdx.u32NodeIdx = 0;
        stInputDataIdx.u32SegIdx = 0;
        stProcSegIdx.u32SegIdx = 1;
        s32Ret = SAMPLE_SVP_NNIE_ForwardWithBbox(&s_stPvanetNnieParam, &stInputDataIdx,
            &s_stPvanetSoftwareParam.stRpnBbox, &stProcSegIdx, HI_TRUE);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error,SAMPLE_SVP_NNIE_ForwardWithBbox failed!\n");

        /* GetResult */
        /* if user has changed net struct, please make sure SAMPLE_SVP_NNIE_FasterRcnn_GetResult
         function's input data are correct */
        s32Ret = SAMPLE_SVP_NNIE_Pvanet_GetResult(&s_stPvanetNnieParam, &s_stPvanetSoftwareParam);
        SAMPLE_SVP_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, PVANET_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "ERROR, SAMPLE_SVP_NNIE_Pvanet_GetResult Failed!\n");
    } else {
        for (i = 0; i < s_stPvanetSoftwareParam.stClassRoiNum.unShape.stWhc.u32Width; i++) {
            *(SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_U32, s_stPvanetSoftwareParam.stClassRoiNum.u64VirAddr) + i) = 0;
        }
    }

    /* print result, this sample has 21 classes:
     class 0:background     class 1:plane           class 2:bicycle
     class 3:bird           class 4:boat            class 5:bottle
     class 6:bus            class 7:car             class 8:cat
     class 9:chair          class10:cow             class11:diningtable
     class 12:dog           class13:horse           class14:motorbike
     class 15:person        class16:pottedplant     class17:sheep
     class 18:sofa          class19:train           class20:tvmonitor */
    if (g_stop_signal == HI_TRUE) {
        SAMPLE_SVP_NNIE_Pvanet_Stop();
        return;
    }
    SAMPLE_SVP_TRACE_INFO("Pvanet result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stPvanetSoftwareParam.stDstScore, &s_stPvanetSoftwareParam.stDstRoi,
        &s_stPvanetSoftwareParam.stClassRoiNum, f32PrintResultThresh);

PVANET_FAIL_0:
    SAMPLE_SVP_NNIE_Pvanet_Deinit(&s_stPvanetNnieParam, &s_stPvanetSoftwareParam, &s_stPvanetModel);

    SAMPLE_COMM_SVP_CheckSysExit();
}

void SAMPLE_SVP_NNIE_Pvanet_HandleSig(void)
{
    g_stop_signal = HI_TRUE;
}

/******************************************************************************
* function : roi to rect
******************************************************************************/
HI_S32 SAMPLE_SVP_NNIE_RoiToRect_Yolov3(SVP_BLOB_S *pstDstScore,
    SVP_BLOB_S *pstDstRoi, SVP_BLOB_S *pstClassRoiNum, HI_FLOAT *paf32ScoreThr,
    HI_BOOL bRmBg,SAMPLE_SVP_NNIE_RECT_ARRAY_S *pstRect,
    HI_U32 u32SrcWidth, HI_U32 u32SrcHeight,HI_U32 u32DstWidth,HI_U32 u32DstHeight)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RoiNumBias = 0;
    HI_U32 u32ScoreBias = 0;
    HI_U32 u32BboxBias = 0;
    HI_FLOAT f32Score = 0.0f;
    HI_S32* ps32Score = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,pstDstScore->u64VirAddr);
    HI_S32* ps32Roi = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,pstDstRoi->u64VirAddr);
    HI_S32* ps32ClassRoiNum = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,pstClassRoiNum->u64VirAddr);
    HI_U32 u32ClassNum = pstClassRoiNum->unShape.stWhc.u32Width;
    HI_U32 u32RoiNumTmp = 0;

    SAMPLE_SVP_CHECK_EXPR_RET(u32ClassNum > 82 ,HI_ERR_SVP_NNIE_ILLEGAL_PARAM,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),u32ClassNum(%u) must be less than or equal %u to!\n",HI_ERR_SVP_NNIE_ILLEGAL_PARAM,u32ClassNum, 82);

   
    pstRect->u32TotalNum = 0;
    pstRect->u32ClsNum = u32ClassNum;
    if (bRmBg)
    {
        pstRect->au32RoiNum[0] = 0;
        u32RoiNumBias += ps32ClassRoiNum[0];
        for (i = 1; i < u32ClassNum; i++)
        {
            u32ScoreBias = u32RoiNumBias;
            u32BboxBias = u32RoiNumBias * SAMPLE_SVP_NNIE_COORDI_NUM;
            u32RoiNumTmp = 0;
            /*if the confidence score greater than result thresh, the result will be drawed*/
            if(((HI_FLOAT)ps32Score[u32ScoreBias] / SAMPLE_SVP_NNIE_QUANT_BASE >=
                paf32ScoreThr[i])  &&  (ps32ClassRoiNum[i] != 0))
            { 
                for (j = 0; j < (HI_U32)ps32ClassRoiNum[i]; j++)
                {
                    /*Score is descend order*/
                    f32Score = (HI_FLOAT)ps32Score[u32ScoreBias + j] / SAMPLE_SVP_NNIE_QUANT_BASE;
                    if ((f32Score < paf32ScoreThr[i]) || (u32RoiNumTmp >= SAMPLE_SVP_NNIE_MAX_ROI_NUM_OF_CLASS))
                    {
                        break;
                    }

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32X = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM] / (HI_FLOAT)u32SrcWidth * (HI_FLOAT)u32DstWidth) & (~1) ;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32Y = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM + 1] / (HI_FLOAT)u32SrcHeight * (HI_FLOAT)u32DstHeight) & (~1);

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32X = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM + 2]/ (HI_FLOAT)u32SrcWidth * (HI_FLOAT)u32DstWidth) & (~1);
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32Y = pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32Y;

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32X = pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32X;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32Y = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM + 3] / (HI_FLOAT)u32SrcHeight * (HI_FLOAT)u32DstHeight) & (~1);

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[3].s32X =  pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32X;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[3].s32Y =  pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32Y;

                    u32RoiNumTmp++;
                }

            }

            pstRect->au32RoiNum[i] = u32RoiNumTmp;
            pstRect->u32TotalNum += u32RoiNumTmp;
            u32RoiNumBias += ps32ClassRoiNum[i];
        }

    }
    return HI_SUCCESS;
}


/******************************************************************************
* function : Yolov3 Procession ViToVo
******************************************************************************/
static HI_S32 SAMPLE_SVP_NNIE_Yolov3_Proc_ViToVo(SAMPLE_SVP_NNIE_PARAM_S *pstParam,
    SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S *pstSwParam, VIDEO_FRAME_INFO_S* pstExtFrmInfo,
    HI_U32 u32BaseWidth,HI_U32 u32BaseHeight)
{
    HI_S32 s32Ret = HI_FAILURE;
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = {0};
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = {0};

    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    HI_FLOAT f32PrintResultThresh = 0.8f;
    /*SP420*/
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u64VirAddr = pstExtFrmInfo->stVFrame.u64VirAddr[0];
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u64PhyAddr = pstExtFrmInfo->stVFrame.u64PhyAddr[0];
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u32Stride  = pstExtFrmInfo->stVFrame.u32Stride[0];


    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stYolov3NnieParam,&stInputDataIdx,&stProcSegIdx,HI_TRUE);

    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov3_Proc failed!\n");
    /*draw result, this sample has 21 classes:
     class 0:background     class 1:plane           class 2:bicycle
     class 3:bird           class 4:boat            class 5:bottle
     class 6:bus            class 7:car             class 8:cat
     class 9:chair          class10:cow             class11:diningtable
     class 12:dog           class13:horse           class14:motorbike
     class 15:person        class16:pottedplant     class17:sheep
     class 18:sofa          class19:train           class20:tvmonitor*/

    /*Software process*/
    /*if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Yolov3_GetResult
     function input datas are correct*/
    s32Ret = SAMPLE_SVP_NNIE_Yolov3_GetResult(&s_stYolov3NnieParam,&s_stYolov3SoftwareParam);


  //  (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&(pstSwParam->stDstScore), &(pstSwParam->stDstRoi), &(pstSwParam->stClassRoiNum),f32PrintResultThresh);

    SAMPLE_SVP_TRACE_INFO("Yolov3 result:\n");
    (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stYolov3SoftwareParam.stDstScore, 
          &s_stYolov3SoftwareParam.stDstRoi, &s_stYolov3SoftwareParam.stClassRoiNum,f32PrintResultThresh);
	
  s32Ret = SAMPLE_SVP_NNIE_RoiToRect_Yolov3(&(pstSwParam->stDstScore),
    &(pstSwParam->stDstRoi), &(pstSwParam->stClassRoiNum), pstSwParam->af32ScoreThr,HI_TRUE,&(pstSwParam->stRect),
    pstExtFrmInfo->stVFrame.u32Width, pstExtFrmInfo->stVFrame.u32Height,u32BaseWidth,u32BaseHeight);
    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_RoiToRect failed!\n",s32Ret); 

    return s32Ret;

}
/******************************************************************************
* function : Yolov3 vi to vo thread entry
******************************************************************************/
static HI_VOID* SAMPLE_SVP_NNIE_Yolov3_ViToVo_thread(HI_VOID* pArgs)
{
    HI_S32 s32Ret;
    SAMPLE_SVP_NNIE_PARAM_S *pstParam;
    SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S *pstSwParam;
    VIDEO_FRAME_INFO_S stBaseFrmInfo;
    VIDEO_FRAME_INFO_S stExtFrmInfo;
    HI_S32 s32MilliSec = 20000;
    VO_LAYER voLayer = 0;
    VO_CHN voChn = 0;
    HI_S32 s32VpssGrp = 0;
    HI_S32 as32VpssChn[] = {VPSS_CHN0, VPSS_CHN1};

    pstParam = &s_stYolov3NnieParam;
    pstSwParam = &s_stYolov3SoftwareParam;

     struct timespec start_time,end_time;
    float use_time =0;

    while (HI_FALSE == s_bNnieStopSignal)
    {
        clock_gettime(CLOCK_REALTIME, &start_time);
            s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[1], &stExtFrmInfo, s32MilliSec);
        if(HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n",
                s32Ret,s32VpssGrp, as32VpssChn[1]);
            continue;
        }

        s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[0], &stBaseFrmInfo, s32MilliSec);
        SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, EXT_RELEASE,
            "Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n",
            s32Ret,s32VpssGrp, as32VpssChn[0]);

        s32Ret = SAMPLE_SVP_NNIE_Yolov3_Proc_ViToVo(pstParam,pstSwParam, &stExtFrmInfo,
        stBaseFrmInfo.stVFrame.u32Width,stBaseFrmInfo.stVFrame.u32Height);
        SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
            "Error(%#x),SAMPLE_SVP_NNIE_YOLOV3_Proc failed!\n", s32Ret);

       // SAMPLE_SVP_TRACE_INFO("The Width is %d\n", stExtFrmInfo.stVFrame.u32Width);
	// SAMPLE_SVP_TRACE_INFO("The Height is %d\n", stExtFrmInfo.stVFrame.u32Height);

	 
        //Draw rect
        s32Ret = SAMPLE_COMM_SVP_NNIE_FillRect(&stBaseFrmInfo, &(pstSwParam->stRect), 0x0000FF00);
        SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
            "SAMPLE_COMM_SVP_NNIE_FillRect failed, Error(%#x)!\n", s32Ret);

        s32Ret = HI_MPI_VO_SendFrame(voLayer, voChn, &stBaseFrmInfo, s32MilliSec);
        SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
            "HI_MPI_VO_SendFrame failed, Error(%#x)!\n", s32Ret);

       clock_gettime(CLOCK_REALTIME, &end_time);
       use_time = (float)(end_time.tv_sec - start_time.tv_sec)*1000 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
       SAMPLE_SVP_TRACE_INFO("The use time is %f  \n", use_time);
       
	

        BASE_RELEASE:
            s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp,as32VpssChn[0], &stBaseFrmInfo);
            if (HI_SUCCESS != s32Ret)
            {
                SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n",
                    s32Ret,s32VpssGrp,as32VpssChn[0]);
            }

        EXT_RELEASE:
            s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp,as32VpssChn[1], &stExtFrmInfo);
            if (HI_SUCCESS != s32Ret)
            {
                SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n",
                    s32Ret,s32VpssGrp,as32VpssChn[1]);
            }

    }

    return HI_NULL;
}
/******************************************************************************
* function : Yolov3 vi to vo real time detection
******************************************************************************/
void SAMPLE_SVP_NNIE_Yolov3_Vivo(void)
{
 //   HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/dog_bike_car_416x416.bgr";
    HI_CHAR *pcModelName = "./data/nnie_model/detection/inst_yolov3_cycle.wk";
    SAMPLE_SVP_NNIE_CFG_S   stNnieCfg = {0};
    SIZE_S stSize;
    PIC_SIZE_E enSize = PIC_CIF;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_CHAR acThreadName[16] = {0};




    /*Sys init*/
    SAMPLE_COMM_SVP_CheckSysInit();

    /******************************************
     step 1: start vi vpss vo
     ******************************************/
    s_stYolov3Switch.bVenc = HI_FALSE;
    s_stYolov3Switch.bVo   = HI_TRUE;
    s32Ret = SAMPLE_COMM_IVE_StartViVpssVencVo_Yolov3(&s_stViConfig,&s_stYolov3Switch,&enSize);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, YOLOV3_FAIL_1,
        "Error(%#x),SAMPLE_COMM_IVE_StartViVpssVencVo failed!\n", s32Ret);

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enSize, &stSize);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, YOLOV3_FAIL_1,
        "Error(%#x),SAMPLE_COMM_SYS_GetPicSize failed!\n", s32Ret);

    stSize.u32Width = 640;
    stSize.u32Height = 384;

    /******************************************
     step 2: init NNIE param
     ******************************************/
    stNnieCfg.pszPic= NULL;
    stNnieCfg.u32MaxInputNum = 1;
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0;//set NNIE core

    /*Yolov3 Load model*/
    SAMPLE_SVP_TRACE_INFO("Yolov3 Load model!\n");
    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName,&s_stYolov3Model);
    SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret,YOLOV3_FAIL_0,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

    /*Yolov3 parameter initialization*/
    /*Yolov3 software parameters are set in SAMPLE_SVP_NNIE_Yolov3_SoftwareInit,
      if user has changed net struct, please make sure the parameter settings in
      SAMPLE_SVP_NNIE_Yolov3_SoftwareInit function are correct*/
    SAMPLE_SVP_TRACE_INFO("Yolov3 parameter initialization!\n");
    s_stYolov3NnieParam.pstModel = &s_stYolov3Model.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Yolov3_ParamInit(&stNnieCfg,&s_stYolov3NnieParam,&s_stYolov3SoftwareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret,YOLOV3_FAIL_0,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov3_ParamInit failed!\n");

    /*Fill src data*/
    SAMPLE_SVP_TRACE_INFO("Yolov3 start!\n");


    s_bNnieStopSignal = HI_FALSE;
    /******************************************
      step 3: Create work thread
     ******************************************/
    snprintf(acThreadName, 16, "NNIE_ViToVo");
    prctl(PR_SET_NAME, (unsigned long)acThreadName, 0,0,0);
    pthread_create(&s_hNnieThread, 0, SAMPLE_SVP_NNIE_Yolov3_ViToVo_thread, NULL);	
     SAMPLE_PAUSE();

    s_bNnieStopSignal = HI_TRUE;
    pthread_join(s_hNnieThread, HI_NULL);
    s_hNnieThread = 0;



YOLOV3_FAIL_1:
    SAMPLE_SVP_NNIE_Yolov3_Deinit(&s_stYolov3NnieParam,&s_stYolov3SoftwareParam,&s_stYolov3Model);

YOLOV3_FAIL_0:
    SAMPLE_COMM_IVE_StopViVpssVencVo_Yolov3(&s_stViConfig,&s_stYolov3Switch);

}

void SAMPLE_SVP_NNIE_Yolov3_Vivo_HandleSig(void)
{
    s_bNnieStopSignal = HI_TRUE;
    if (0 != s_hNnieThread)
    {
        pthread_join(s_hNnieThread, HI_NULL);
        s_hNnieThread = 0;
    }

    SAMPLE_SVP_NNIE_Yolov3_Deinit(&s_stYolov3NnieParam,&s_stYolov3SoftwareParam,&s_stYolov3Model);
    memset(&s_stYolov3NnieParam,0,sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    memset(&s_stYolov3SoftwareParam,0,sizeof(SAMPLE_SVP_NNIE_YOLOV3_SOFTWARE_PARAM_S));
    memset(&s_stYolov3Model,0,sizeof(SAMPLE_SVP_NNIE_MODEL_S));

    SAMPLE_COMM_IVE_StopViVpssVencVo_Yolov3(&s_stViConfig,&s_stYolov3Switch);

}


#define Net_FRM_WIDTH    640
#define Net_FRM_HEIGHT   384
/*===========================================================================*/
/******************************************************************************
* function : roi to rect Yolov2
******************************************************************************/
HI_S32 SAMPLE_SVP_NNIE_RoiToRect_Yolov2(SVP_BLOB_S *pstDstScore,
    SVP_BLOB_S *pstDstRoi, SVP_BLOB_S *pstClassRoiNum, HI_FLOAT *paf32ScoreThr,
    HI_BOOL bRmBg,SAMPLE_SVP_NNIE_RECT_ARRAY_S *pstRect,
    HI_U32 u32SrcWidth, HI_U32 u32SrcHeight,HI_U32 u32DstWidth,HI_U32 u32DstHeight)
{
    HI_U32 i = 0, j = 0;
    HI_U32 u32RoiNumBias = 0;
    HI_U32 u32ScoreBias = 0;
    HI_U32 u32BboxBias = 0;
    HI_FLOAT f32Score = 0.0f;
    HI_S32* ps32Score = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,pstDstScore->u64VirAddr);
    HI_S32* ps32Roi = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,pstDstRoi->u64VirAddr);
    HI_S32* ps32ClassRoiNum = SAMPLE_SVP_NNIE_CONVERT_64BIT_ADDR(HI_S32,pstClassRoiNum->u64VirAddr);
    HI_U32 u32ClassNum = pstClassRoiNum->unShape.stWhc.u32Width;
    HI_U32 u32RoiNumTmp = 0;

    SAMPLE_SVP_CHECK_EXPR_RET(u32ClassNum > 4 ,HI_ERR_SVP_NNIE_ILLEGAL_PARAM,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),u32ClassNum(%u) must be less than or equal %u to!\n",HI_ERR_SVP_NNIE_ILLEGAL_PARAM,u32ClassNum, 4);

   
    pstRect->u32TotalNum = 0;
    pstRect->u32ClsNum = u32ClassNum;
    if (bRmBg)
    {
        pstRect->au32RoiNum[0] = 0;
        u32RoiNumBias += ps32ClassRoiNum[0];
        for (i = 1; i < u32ClassNum; i++)
        {
            u32ScoreBias = u32RoiNumBias;
            u32BboxBias = u32RoiNumBias * SAMPLE_SVP_NNIE_COORDI_NUM;
            u32RoiNumTmp = 0;
            /*if the confidence score greater than result thresh, the result will be drawed*/
            if(((HI_FLOAT)ps32Score[u32ScoreBias] / SAMPLE_SVP_NNIE_QUANT_BASE >=
                paf32ScoreThr[i])  &&  (ps32ClassRoiNum[i] != 0))
            { 
                for (j = 0; j < (HI_U32)ps32ClassRoiNum[i]; j++)
                {
                    /*Score is descend order*/
                    f32Score = (HI_FLOAT)ps32Score[u32ScoreBias + j] / SAMPLE_SVP_NNIE_QUANT_BASE;
                    if ((f32Score < paf32ScoreThr[i]) || (u32RoiNumTmp >= SAMPLE_SVP_NNIE_MAX_ROI_NUM_OF_CLASS))
                    {
                        break;
                    }

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32X = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM] / (HI_FLOAT)u32SrcWidth * (HI_FLOAT)u32DstWidth) & (~1) ;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32Y = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM + 1] / (HI_FLOAT)u32SrcHeight * (HI_FLOAT)u32DstHeight) & (~1);

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32X = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM + 2]/ (HI_FLOAT)u32SrcWidth * (HI_FLOAT)u32DstWidth) & (~1);
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32Y = pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32Y;

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32X = pstRect->astRect[i][u32RoiNumTmp].astPoint[1].s32X;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32Y = (HI_U32)((HI_FLOAT)ps32Roi[u32BboxBias + j*SAMPLE_SVP_NNIE_COORDI_NUM + 3] / (HI_FLOAT)u32SrcHeight * (HI_FLOAT)u32DstHeight) & (~1);

                    pstRect->astRect[i][u32RoiNumTmp].astPoint[3].s32X =  pstRect->astRect[i][u32RoiNumTmp].astPoint[0].s32X;
                    pstRect->astRect[i][u32RoiNumTmp].astPoint[3].s32Y =  pstRect->astRect[i][u32RoiNumTmp].astPoint[2].s32Y;

                    u32RoiNumTmp++;
                }

            }

            pstRect->au32RoiNum[i] = u32RoiNumTmp;
            pstRect->u32TotalNum += u32RoiNumTmp;
            u32RoiNumBias += ps32ClassRoiNum[i];
        }

    }
    return HI_SUCCESS;
}

#define FOCUS_LEN 25
#define FOCUS_THRESH 12
#define MISS_THRESH 3
#define TRACK_TRESH 3
#define MATCH_THRESH 0.4
#define BUF_LEN 80

typedef struct TRACK_ID { 
    // RectBox box_prev;   
    //Yolo2FetchRes得到的resBuf相当于current时刻的box，PERSON_ID中的box_curr相当于上一帧的box，所以不需要box_prev这个变量了
    RectBox box_curr;
    // HI_U32 cls_arr[FOCUS_LEN];
    int cls;
    HI_U32 box_ID;
    HI_U32 miss_hits;
    HI_BOOL matched_flag;
    HI_U32 tracked_count;
    HI_BOOL tracked_flag;
    float focused_time[FOCUS_LEN]; //循环利用
    float focused_time_total;
    float live_time;
    HI_BOOL accumulate_first_flag;
} PERSON_ID;

//这么写会重复累加，导致统计的注意力时长大于实际的
// HI_VOID Accumulate_Focused_Time(float focused_time[], int len)
// {
//     int i;
//     int focused_count=0;
//     float sum=0;
//     //static HI_U32 focused_time_total=0;
//     for(i=0;i<len;i++){
//         if(focused_time[i]>0){
//             sum+=focused_time[i];
//             focused_count++;
//         }
//     }
//     if(focused_count>=FOCUS_THRESH)
//         focused_time_total+=sum;
// }

//应该对每一个ID维护一个accumulate_first_flag的标志，否则会造成统计的注意力时长偏小
// HI_VOID Accumulate_Focused_Time(float focused_time[], int len, int frame_curr)
// {
//     int i;
//     int focused_count=0;    
//     if(accumulate_first_flag==HI_FALSE){
//         float sum=0;
//         for(i=0;i<len;i++){
//             if(focused_time[i]>0){
//                 sum+=focused_time[i];
//                 focused_count++;
//             }
//         }
//         if(focused_count>=FOCUS_THRESH)
//             focused_time_total+=sum;
//         accumulate_first_flag=HI_TRUE;
//     }
//     else{
//         for(i=0;i<len;i++)
//             if(focused_time[i]>0)
//                 focused_count++;
//         if(focused_count>=FOCUS_THRESH)
//             focused_time_total+=focused_time[frame_curr];
//     }
// }

// HI_VOID Accumulate_Focused_Time(PERSON_ID *ID, int len, int frame_curr, float live_time_tmp)
HI_VOID Accumulate_Focused_Time(PERSON_ID *ID, int len, int frame_curr)
{
    int i;
    int focused_count=0;    
    if(ID->accumulate_first_flag==HI_FALSE){
        float sum=0;
        for(i=0;i<len;i++){
            if(ID->focused_time[i]>0){
                sum+=ID->focused_time[i];
                focused_count++;
            }
        }
        if(focused_count>=FOCUS_THRESH){
            focused_time_total+=sum;
            ID->focused_time_total+=sum;
            ID->accumulate_first_flag=HI_TRUE;
        }
    }
    else{
        for(i=0;i<len;i++)
            if(ID->focused_time[i]>0)
                focused_count++;
        if(focused_count>=FOCUS_THRESH){
            focused_time_total+=ID->focused_time[frame_curr];
            ID->focused_time_total+=ID->focused_time[frame_curr];
        }
    }
    // ID->live_time += live_time_tmp;
}

HI_DOUBLE Track_Yolov2_Iou(RectBox *box1, RectBox *box2)
{
    HI_FLOAT f32InterWidth = 0.0;
    HI_FLOAT f32InterHeight = 0.0;
    HI_DOUBLE f64InterArea = 0.0;
    HI_DOUBLE f64Box1Area = 0.0;
    HI_DOUBLE f64Box2Area = 0.0;
    HI_DOUBLE f64UnionArea = 0.0;

    f32InterWidth = (HI_FLOAT)(SAMPLE_SVP_NNIE_MIN(box1->xmax,box2->xmax)-SAMPLE_SVP_NNIE_MAX(box1->xmin,box2->xmin));
    f32InterHeight = (HI_FLOAT)(SAMPLE_SVP_NNIE_MIN(box1->ymax,box2->ymax)-SAMPLE_SVP_NNIE_MAX(box1->ymin,box2->ymin));
    if (f32InterWidth <= 0 || f32InterHeight <= 0)
        return 0;

    f64InterArea = f32InterWidth * f32InterHeight;
    f64Box1Area = (HI_DOUBLE)((box1->xmax - box1->xmin) * (box1->ymax - box1->ymin));
    f64Box2Area = (HI_DOUBLE)((box2->xmax - box2->xmin) * (box2->ymax - box2->ymin));
    f64UnionArea = f64Box1Area + f64Box2Area - f64InterArea;

    return f64InterArea / f64UnionArea;
}

/*
参数初始化放在SAMPLE_SVP_NNIE_Yolov2_ViToVo_thread()函数的前面
注意：刚开始跟踪时box_prev为空
对每一帧进行跟踪时，都将matched_flag置为False

匹配成功：
1、box_prev与box_curr两两之间计算IOU，匹配最大的那个，视为同一个目标ID，置其matched_flag为True
匹配失败：
2、漏检，不更新box_curr，miss_hits+=1，连续超过3帧漏检(miss_hits>3)则丢弃该目标ID，否则将miss_hits清零
3、新增目标ID
*/

//resBuf把box信息传给ID_arr
// int Trivial_Track_Yolov2(DetectObjInfo resBuf[], int resLen, float usedTime, PERSON_ID ID_arr[], int prevBoxNum, HI_U32 frame, HI_FLOAT *AD_attractiveness_tmp)
int Trivial_Track_Yolov2(DetectObjInfo resBuf[], int resLen, float usedTime, PERSON_ID ID_arr[], int prevBoxNum, HI_U32 frame)
{
    int i,j;
    int frame_tmp=frame%FOCUS_LEN;
    int box_curr_num=0;
    // RectBox *box_tmp;
    HI_DOUBLE max_IOU;
    HI_DOUBLE IOU_tmp;
    HI_BOOL matched_flag[DETECT_OBJ_MAX]={0};
    int max_idx;
    // HI_BOOL all_matched_flag;
    //static HI_U32 ID_num_total=0; //被跟踪的总人数

    //算法刚开始运行的第一帧或前面一帧没有目标（要么一直没有目标，要么目标因为恰好消失或漏检次数超过阈值而丢弃）的情况，不需要考虑box_prev，不需要进行IOU匹配
    //前一帧prevBoxNum==0的情况说明当前帧的框都是新增的！所以根本没必要累积注意力时长
    if (prevBoxNum==0){
        for(i=0;i<resLen;i++){
            // box_tmp = &ID_arr[i].box_curr;
            // box_tmp->xmin = resBuf[i].box.xmin;
            // box_tmp->ymin = resBuf[i].box.ymin;
            // box_tmp->xmax = resBuf[i].box.xmax;
            // box_tmp->ymax = resBuf[i].box.ymax;
            ID_arr[i].box_curr=resBuf[i].box;
            // ID_arr[i].box_ID = ID_num_total;
            // ID_num_total++;
            ID_arr[i].cls=resBuf[i].cls;
            ID_arr[i].miss_hits=0;
            // if(ID_arr[i].tracked_flag==HI_FALSE)
            ID_arr[i].tracked_count=1;
            ID_arr[i].tracked_flag=HI_FALSE;
            ID_arr[i].focused_time_total=0;
            ID_arr[i].live_time=usedTime;
            ID_arr[i].accumulate_first_flag=HI_FALSE;
            if(resBuf[i].cls==2){
                ID_arr[i].focused_time[frame_tmp] = usedTime;                
            }
            // else if (resBuf[i].cls==1){
            else{
                ID_arr[i].focused_time[frame_tmp] = 0;
            }
            // printf("boxcurr(%d,%d,%d,%d),ID:%d,cls%d,hitmiss%d",ID_arr[i].box_curr.xmin,ID_arr[i].box_curr.ymin,ID_arr[i].box_curr.xmax,
            //     ID_arr[i].box_curr.ymax,ID_arr[i].box_ID,ID_arr[i].cls,ID_arr[i].miss_hits);
            // if(frame>1 && ID_arr[i].tracked_flag==HI_TRUE) //第一帧不需要累积注意力时长
            //     Accumulate_Focused_Time(ID_arr[i].focused_time,FOCUS_LEN,frame_tmp);            
        }
        prevBoxNum = resLen;
    }
    else if (prevBoxNum>0)
    {
        //matched_flag初始化为False 
        for(i=0;i<prevBoxNum;i++)
            ID_arr[i].matched_flag = HI_FALSE;

        //resLen>prevBoxNum: 丢失的目标数<新增的目标数（新进入画面或者连续超过3帧漏检）
        //resLen==prevBoxNum: 刚好匹配；丢失的目标数==新增的目标数
        //resLen<prevBoxNum: 漏检；目标离开画面；丢失的目标数>新增的目标数
        for(i=0;i<prevBoxNum;i++){
            max_IOU=0;
            for(j=0;j<resLen;j++)
            {
                if(matched_flag[j]==HI_FALSE){
                    IOU_tmp=Track_Yolov2_Iou(&resBuf[j].box,&ID_arr[i].box_curr);
                    if(IOU_tmp>max_IOU){
                        max_IOU=IOU_tmp;
                        max_idx=j;
                    }
                }
            }
            //匹配成功
            if(max_IOU>MATCH_THRESH){ 
                matched_flag[max_idx]=HI_TRUE;
                // ID_arr[i].box_prev=ID_arr[i].box_curr;
                ID_arr[i].box_curr=resBuf[max_idx].box;
                ID_arr[i].miss_hits=0;
                ID_arr[i].matched_flag=HI_TRUE;
                ID_arr[i].cls=resBuf[max_idx].cls;
                if(resBuf[max_idx].cls==2)
                    ID_arr[i].focused_time[frame_tmp] = usedTime;                
                else if(resBuf[max_idx].cls==1)
                    ID_arr[i].focused_time[frame_tmp] = 0;
                ID_arr[i].live_time += usedTime;
                if(ID_arr[i].cls == 1 || ID_arr[i].cls ==2){
                    if(ID_arr[i].tracked_flag==HI_FALSE){
                        if(ID_arr[i].tracked_count<=TRACK_TRESH){
                            ID_arr[i].tracked_count++;
                            if(ID_arr[i].tracked_count>TRACK_TRESH){
                                ID_arr[i].tracked_flag=HI_TRUE;
                                ID_arr[i].box_ID = ID_num_total;
                                ID_num_total++;
                            }
                        }
                    }
                }
            }
        } 
        //分三种情况来考虑，好像没必要
        // if(prevBoxNum>resLen){
            // for(i=0;i<resLen;i++)
            //     if(matched_flag[i]!=HI_TRUE){ //当前检测到的目标框全部匹配上
            //         all_matched_flag=HI_FALSE;
            //         break;
            //     }
        // }            
        for(i=0;i<prevBoxNum;i++){
            if(ID_arr[i].matched_flag==HI_FALSE){//上一帧的目标框和当前帧匹配失败                
                ID_arr[i].miss_hits++;
                // if(ID_arr[i].tracked_flag==HI_FALSE){ //若在漏检之前Track处于unconfirmed(untracked)的状态，可以直接把它删除
                //     for(j=i;j<prevBoxNum-1;j++)
                //         ID_arr[j]=ID_arr[j+1];
                //     (HI_VOID)memset_s(&ID_arr[prevBoxNum-1],sizeof(PERSON_ID),0,sizeof(PERSON_ID));
                //     prevBoxNum--;
                //     i--;
                // }
                // else if(ID_arr[i].miss_hits>MISS_THRESH){
                if(ID_arr[i].miss_hits>MISS_THRESH){ 
                    // if(ID_arr[i].tracked_flag==HI_TRUE)
                    //     *AD_attractiveness_tmp=*AD_attractiveness_tmp + ID_arr[i].focused_time_total/ID_arr[i].live_time;
                    for(j=i;j<prevBoxNum-1;j++)
                        ID_arr[j]=ID_arr[j+1];
                    (HI_VOID)memset_s(&ID_arr[prevBoxNum-1],sizeof(PERSON_ID),0,sizeof(PERSON_ID));
                    prevBoxNum--;
                    i--;
                }
                else{
                    box_curr_num++;
                    ID_arr[i].live_time += usedTime;
                    if(ID_arr[i].cls==2)
                        ID_arr[i].focused_time[frame_tmp]=usedTime;
                    // else if(ID_arr[i].cls==1)
                    else
                        ID_arr[i].focused_time[frame_tmp]=0;
                }
            }
            else
                box_curr_num++;
        }
        //到这一步box_curr_num应该等于prevBoxNum
        HI_ASSERT(box_curr_num==prevBoxNum);
        //Unmatched Detections -> New Unconfirmed Tracks
        for(i=0;i<resLen;i++)
            if(matched_flag[i]!=HI_TRUE){ //当前帧未匹配到的，视作新增的目标
                if(box_curr_num+1<DETECT_OBJ_MAX){
                    ID_arr[box_curr_num+1].box_curr = resBuf[i].box;
                    ID_arr[box_curr_num+1].cls = resBuf[i].cls;
                    ID_arr[box_curr_num+1].miss_hits=0;
                    ID_arr[box_curr_num+1].tracked_count=1;
                    ID_arr[box_curr_num+1].tracked_flag=HI_FALSE;
                    ID_arr[box_curr_num+1].focused_time_total=0;
                    ID_arr[box_curr_num+1].live_time=usedTime;
                    ID_arr[box_curr_num+1].accumulate_first_flag=HI_FALSE;
                    // ID_arr[box_curr_num+1].box_ID = ID_num_total;
                    // ID_num_total++;
                    box_curr_num++;
                }
            }            
        prevBoxNum=box_curr_num;
        for(i=0;i<prevBoxNum;i++)
            if(ID_arr[i].tracked_flag==HI_TRUE)
                // Accumulate_Focused_Time(ID_arr[i].focused_time,FOCUS_LEN,frame_tmp);
                // Accumulate_Focused_Time(&ID_arr[i],FOCUS_LEN,frame_tmp,usedTime);
                Accumulate_Focused_Time(&ID_arr[i],FOCUS_LEN,frame_tmp);
    }
    return prevBoxNum;
}

// HI_VOID Del_i(PERSON_ID ID_arr[], int i, int len)
// {
//     for(int j=i;j<len-1;j++)
//         ID_arr[j]=ID_arr[j+1];        
// }


/******************************************************************************
* function : Yolov2 Procession ViToVo
******************************************************************************/
static HI_S32 SAMPLE_SVP_NNIE_Yolov2_Proc_ViToVo(SAMPLE_SVP_NNIE_PARAM_S *pstParam,
    SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S *pstSwParam, VIDEO_FRAME_INFO_S* pstExtFrmInfo,
    HI_U32 u32BaseWidth,HI_U32 u32BaseHeight,DetectObjInfo resBuf[], int resSize, int* resLen)
{
    HI_S32 s32Ret = HI_FAILURE;
    SAMPLE_SVP_NNIE_INPUT_DATA_INDEX_S stInputDataIdx = {0};
    SAMPLE_SVP_NNIE_PROCESS_SEG_INDEX_S stProcSegIdx = {0};

    stInputDataIdx.u32SegIdx = 0;
    stInputDataIdx.u32NodeIdx = 0;
    HI_FLOAT f32PrintResultThresh = 0.8f;
    /*SP420*/
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u64VirAddr = pstExtFrmInfo->stVFrame.u64VirAddr[0];
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u64PhyAddr = pstExtFrmInfo->stVFrame.u64PhyAddr[0];
    pstParam->astSegData[stInputDataIdx.u32SegIdx].astSrc[stInputDataIdx.u32NodeIdx].u32Stride  = pstExtFrmInfo->stVFrame.u32Stride[0];


    s32Ret = SAMPLE_SVP_NNIE_Forward(&s_stYolov2NnieParam,&stInputDataIdx,&stProcSegIdx,HI_TRUE);

    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error,SAMPLE_SVP_NNIE_Yolov2_Proc failed!\n");
    /*draw result, this sample has 3 classes:
     class 0:background     class 1:focused           class 2:unfocused
*/


    /*Software process*/
    /*if user has changed net struct, please make sure SAMPLE_SVP_NNIE_Yolov2_GetResult
     function input datas are correct*/
    s32Ret = SAMPLE_SVP_NNIE_Yolov2_GetResult(&s_stYolov2NnieParam,&s_stYolov2SoftwareParam);
    //这一步把归一化的xywh转成s_stYolov2SoftwareParam.stDstRoi的xyxy了

    // SAMPLE_SVP_TRACE_INFO("Yolov2 result:\n");
    // (void)SAMPLE_SVP_NNIE_Detection_PrintResult(&s_stYolov2SoftwareParam.stDstScore, SAMPLE_SVP_NNIE_Yolov2_ViToVo_threads_bNnieStopSignal
    //       &s_stYolov2SoftwareParam.stDstRoi, &s_stYolov2SoftwareParam.stClassRoiNum,f32PrintResultThresh);
	
    s32Ret = SAMPLE_SVP_NNIE_RoiToRect_Yolov2(&(pstSwParam->stDstScore),
    &(pstSwParam->stDstRoi), &(pstSwParam->stClassRoiNum), pstSwParam->af32ScoreThr,HI_TRUE,&(pstSwParam->stRect),
    pstExtFrmInfo->stVFrame.u32Width, pstExtFrmInfo->stVFrame.u32Height,u32BaseWidth,u32BaseHeight);
    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),SAMPLE_SVP_NNIE_RoiToRect failed!\n",s32Ret); 
    
    //主要作用是把检测框的信息转为DetectObjInfo的格式，并且对低于置信度阈值的框进行了进一步过滤
    Yolo2FetchRes(&s_stYolov2SoftwareParam.stDstScore,
        &s_stYolov2SoftwareParam.stDstRoi, &s_stYolov2SoftwareParam.stClassRoiNum, resBuf, resSize, resLen);

    return s32Ret;

}

// struct readThreadArgs {
//     int UnfocusNum;
//     int FocusNum; 
// };

static HI_VOID* read_infrared_signal(void* args)
{
    int *curr_box_num = (int *) args;
    /*uart*/
    char *UartName1 = NULL;
    char *UartName2 = NULL;
    UartName1 = "Some_People\n"; 
    UartName2 = "None_People\n"; 
    unsigned char uartReadBuff[2048] = {0}; /* 读取buff大小为2048 */
    unsigned int RecvLen = strlen(UartName1);
    //unsigned int ret = 0;
    int readLen = 0;
    /*
        * 串口初始化定义
        * Serial port initialization definition
        */
    uartFdOur = UartOpenInit();
    if (uartFdOur < 0) {
        printf("uart1 open failed\r\n");
    } else {
        printf("uart1 open successed\r\n");
    }
    while(HI_FALSE == s_bNnieStopSignal)
    {
        readLen = UartRead(uartFdOur, uartReadBuff, RecvLen, 1000); /* 1000 :time out */
        printf("Uart read data:%s\r\n", uartReadBuff);
        if (!strcmp(uartReadBuff,"Some_People\n")){
            g_uart_switch_flag = HI_TRUE;
            // sleep(3);
        }
        else if ((!strcmp(uartReadBuff,"None_People\n")) && (*curr_box_num == 0)){
            g_uart_switch_flag = HI_FALSE;
        }
    }

    return HI_NULL;
}

/******************************************************************************
* function : Yolov2 vi to vo thread entry
******************************************************************************/
static HI_VOID* SAMPLE_SVP_NNIE_Yolov2_ViToVo_thread(HI_VOID* pArgs)
{
    HI_S32 s32Ret;
    SAMPLE_SVP_NNIE_PARAM_S *pstParam;
    SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S *pstSwParam;
    VIDEO_FRAME_INFO_S stBaseFrmInfo;
    VIDEO_FRAME_INFO_S stExtFrmInfo;
    HI_S32 s32MilliSec = 20000;
    VO_LAYER voLayer = 0;
    VO_CHN voChn = 0;
    HI_S32 s32VpssGrp = 0;
    HI_S32 as32VpssChn[] = {VPSS_CHN0, VPSS_CHN1};

    pstParam = &s_stYolov2NnieParam;
    pstSwParam = &s_stYolov2SoftwareParam;

    struct timespec start_time,end_time;
    float use_time =0;
    float fps;

    OsdSet *osds;
    osds = OsdsCreate(HI_OSD_BINDMOD_VPSS, s32VpssGrp, as32VpssChn[0]);
    s32Ret = OsdLibInit();
    g_osdsPerson = osds;
    HI_ASSERT(g_osdsPerson);
    g_osd0Person = OsdsCreateRgn(g_osdsPerson);
    HI_ASSERT(g_osd0Person >= 0);
    memset_s(g_osd_Person_ID, sizeof(HI_S32)*DETECT_OBJ_MAX, -1, sizeof(HI_S32)*DETECT_OBJ_MAX); //或用0xFF

    for(int i=0; i<15; i++){
        g_osd_Person_ID[i] = OsdsCreateRgn(g_osdsPerson);
        HI_ASSERT(g_osd_Person_ID[i]>=0);
    }

    PERSON_ID personID[DETECT_OBJ_MAX] = {0};
    int prev_box_num=0;
    // int max_ID_num=-1;
    int prev_rgn_num=0;
    //static HI_U32 focused_time_total=0;
    //static HI_U32 ID_num_total=0; //被跟踪的总人数
    HI_U32 frame=1; //从1计起
    //广告吸引度
    HI_FLOAT AD_attractiveness_total=0; //AD_attractiveness_dead_ID + 当前帧ID累计得到的吸引度
    // HI_FLOAT AD_attractiveness_dead_ID=0;//已经丢失的ID的AD_attractiveness_total
    HI_FLOAT AD_attractiveness_mean=0; //对ID总数做平均，AD_attractiveness_total/ID_num_total
    //AD_attractiveness_mean即对每个ID做平均的广告吸引度，其值域应为[0,1]

    static HI_CHAR prevOsd[BUF_LEN];
    HI_OSD_ATTR_S rgn;
    HI_OSD_ATTR_S rgn_ID[DETECT_OBJ_MAX]={0};
    int x_ID, y_ID;
    HI_S32 offset = 0;    
    HI_CHAR osdBuf[BUF_LEN] = "";
    HI_S32 offset_box=0;
    HI_CHAR osdBuf_box[BUF_LEN] = "";

    /*udp*/
    struct sockaddr_in s;
    int sock;
    char buffsend[512];
    char buffrecive[512];
    //协议：IPv4、IPv6
    //PF_INET PF_INET6
    //SOCK_DGRAM 无连接的
    //SOCK_STREAM TCP 面向连接的
    if((sock = socket(AF_INET,SOCK_DGRAM,0))==-1){
        perror("error");
    }

    memset(&s,0,sizeof(s));
    s.sin_family = AF_INET;
    s.sin_port = htons(3000);
    //pc端口
    //s.sin_addr.s_addr = inet_addr("192.168.200.10"); 
    //pc wifi
    s.sin_addr.s_addr = inet_addr("192.168.200.100"); 
    //手机端口
    //s.sin_addr.s_addr = inet_addr("192.168.12.2");
    memset(buffsend,0,sizeof(buffsend));
    /*udp*/

    static pthread_t uart_tid;
    // pthread_create(&uart_tid, NULL, read_infrared_signal, NULL);
    pthread_create(&uart_tid, NULL, read_infrared_signal, &prev_box_num);
    usleep(100000);

    while (HI_FALSE == s_bNnieStopSignal)
    {
        if(g_uart_switch_flag == HI_TRUE)
        {
            offset=0;
            offset_box=0;
            memset_s(osdBuf,BUF_LEN,0,BUF_LEN);
            memset_s(osdBuf_box,BUF_LEN,0,BUF_LEN);

            // for(int i=0; i<5; i++){
            // g_osd_Person_ID[i] = OsdsCreateRgn(g_osdsPerson);
            // HI_ASSERT(g_osd_Person_ID[i]>=0);
            // }

            clock_gettime(CLOCK_REALTIME, &start_time);
            s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[1], &stExtFrmInfo, s32MilliSec);
            if(HI_SUCCESS != s32Ret)
            {
                SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n",
                    s32Ret,s32VpssGrp, as32VpssChn[1]);
                continue;
            }

            s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[0], &stBaseFrmInfo, s32MilliSec);
            SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, EXT_RELEASE,
                "Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n",
                s32Ret,s32VpssGrp, as32VpssChn[0]);
        
            VIDEO_FRAME_INFO_S resizeFrm;
            s32Ret = MppFrmResize(&stExtFrmInfo, &resizeFrm, Net_FRM_WIDTH, Net_FRM_HEIGHT);
            int objNum;
            s32Ret = SAMPLE_SVP_NNIE_Yolov2_Proc_ViToVo(pstParam,pstSwParam, &resizeFrm,
            stBaseFrmInfo.stVFrame.u32Width,stBaseFrmInfo.stVFrame.u32Height,objs,DETECT_OBJ_MAX, &objNum);
            SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
                "Error(%#x),SAMPLE_SVP_NNIE_YOLOV2_Proc failed!\n", s32Ret);        

            HI_U32 FocuseNum = 0;
            HI_U32 UnfocuseNum = 0;
            
            //printf("!!!!!!!!!!!!!!!!!!!!!!!! objNum: %d, focused_time: %.3f !!!!!!!!!!!!!!!!!!!!!!!!!!",objNum,focused_time_total);
            // for (int i = 0; i < objNum; i++) {
            //     RectBox *box = &objs[i].box;
            //     RectBoxTran(box, Net_FRM_WIDTH, Net_FRM_HEIGHT,stBaseFrmInfo.stVFrame.u32Width,stBaseFrmInfo.stVFrame.u32Height);
            //     SAMPLE_PRT("yolo2_out: {%d, %d, %d, %d} \n score: %f \n class: %d\n", box->xmin, box->ymin, box->xmax, box->ymax, objs[i].score, objs[i].cls);
            //     //因为验证而打印的部分后续可以删了：包括SAMPLE_PRT、printf、HI_ASSERT
            //     // boxs[i] = *box;
            //     // if(objs[i].cls == 1){
            //     //     UnfocuseNum++;
            //     // }
            //     // if(objs[i].cls == 2){
            //     //     FocuseNum++;
            //     // }
            // }
            //SAMPLE_SVP_TRACE_INFO("The Width is %d\n", resizeFrm.stVFrame.u32Width);
            //SAMPLE_SVP_TRACE_INFO("The Height is %d\n", resizeFrm.stVFrame.u32Height);

            //s32Ret = HI_MPI_VO_SendFrame(voLayer, voChn, &resizeFrm, 0);
            //SAMPLE_CHECK_EXPR_GOTO(s32Ret != HI_SUCCESS, BASE_RELEASE,
            //"HI_MPI_VO_SendFrame fail, Error(%#x)\n", s32Ret);


            //后面画框用到的box信息要使用跟踪的PERSON_ID.box_curr，这样能够弥补漏检的情况
            // MppFrmDestroy(&resizeFrm);
            //  //Draw rect
            // int num = 0;
            // for (int j = 0; j < objNum; j++) {
            //     remainingBoxs[num++] = boxs[j];  
            //     if(objs[j].cls == 1){
            //         MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, objNum, RGB888_GREEN, DRAW_RETC_THICK);
            //     }
            //     else{
            //         MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, objNum, RGB888_RED, DRAW_RETC_THICK);
            //     }
            // }
            //之前不是讨论过MppFrmDrawRects是一次性画多个框的吗
            //改成下面这样？
            // for (int j = 0; j < objNum; j++) {
            //     remainingBoxs[0] = boxs[j];  
            //     if(objs[j].cls == 1){
            //         MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, 1, RGB888_GREEN, DRAW_RETC_THICK);
            //     }
            //     else{
            //         MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, 1, RGB888_RED, DRAW_RETC_THICK);
            //     }
            // }

            clock_gettime(CLOCK_REALTIME, &end_time);
            //单位是毫秒，hyc把这部分移到SendFrame前面了，因为Track要统计时间，统计的ID要显示在画面中
            use_time = (float)(end_time.tv_sec - start_time.tv_sec)*1000 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
            //SAMPLE_SVP_TRACE_INFO("The use time is %f ms\n", use_time);
            fps=1000/use_time;
            // prev_box_num = Trivial_Track_Yolov2(objs, objNum, use_time, personID, prev_box_num, frame, &AD_attractiveness_dead_ID);
            prev_box_num = Trivial_Track_Yolov2(objs, objNum, use_time, personID, prev_box_num, frame);
            frame++;

            // if(prev_box_num>max_ID_num)
            //     max_ID_num=prev_box_num;

            //实验发现会出现cls=0的情况，必须手动把它排除掉
            //Draw rects; display ID number; print info for debugging
            // AD_attractiveness_total=AD_attractiveness_dead_ID;
            AD_attractiveness_total = 0;
            // printf("dead%f\n",AD_attractiveness_total);
            for (int i = 0; i < prev_box_num; i++) {
                RectBox box = personID[i].box_curr;
                RectBoxTran(&box, Net_FRM_WIDTH, Net_FRM_HEIGHT,stBaseFrmInfo.stVFrame.u32Width,stBaseFrmInfo.stVFrame.u32Height);
                //SAMPLE_PRT("yolo2_out_track: box{%d, %d, %d, %d}  \n class: %d\n miss_hits:%d,\n box_ID:%d,\n matched_flag:%d,\n tracked_count:%d,focuesed_time[%f,%f,%f,%f,%f,%f]", 
                    //box.xmin, box.ymin, box.xmax, box.ymax, personID[i].cls, personID[i].miss_hits, personID[i].box_ID, personID[i].matched_flag, personID[i].tracked_count,personID[i].focused_time[0],personID[i].focused_time[1],personID[i].focused_time[2],personID[i].focused_time[3],personID[i].focused_time[4],personID[i].focused_time[5]);
                //因为验证而打印的部分后续可以删了：包括SAMPLE_PRT、printf、HI_ASSERT
                // boxs[i] = box;
                remainingBoxs[0] = box;
                BoxsIDprint[i] = box;
                if(personID[i].tracked_flag==HI_TRUE){
                    memset_s(osdBuf_box,BUF_LEN,0,BUF_LEN);
                    offset_box=0;
                    if(personID[i].cls == 1){
                        UnfocuseNum++;
                        MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, 1, RGB888_GREEN, DRAW_RETC_THICK); 
                        offset_box += snprintf_s(osdBuf_box + offset_box, sizeof(osdBuf_box) - offset_box, sizeof(osdBuf_box) - offset_box - 1, 
                            "%u ", personID[i].box_ID);
                        offset_box += snprintf_s(osdBuf_box + offset_box, sizeof(osdBuf_box) - offset_box, sizeof(osdBuf_box) - offset_box - 1, 
                            "Unfocused");
                        HI_ASSERT(offset_box < sizeof(osdBuf_box));
                        x_ID=SAMPLE_SVP_NNIE_MIN(SAMPLE_SVP_NNIE_MAX(((BoxsIDprint[i].xmin)/2)*2,0),stBaseFrmInfo.stVFrame.u32Width-1);
                        y_ID=SAMPLE_SVP_NNIE_MIN(SAMPLE_SVP_NNIE_MAX(((BoxsIDprint[i].ymin-40)/2)*2,0),stBaseFrmInfo.stVFrame.u32Height-1);
                        //字体大小的设置：sample\taurus\ai_sample\mpp_help\src\osd_img.c，OSD_FONT_WIDTH_DEF与OSD_FONT_HEIGHT_DEF
                        TxtRgnInit(&rgn_ID[i], osdBuf_box, x_ID, y_ID, ARGB1555_BLUE); // font width and heigt use default 40
                        OsdsSetRgn(g_osdsPerson, g_osd_Person_ID[i], &rgn_ID[i]);  
                    }
                    else if(personID[i].cls == 2){
                        FocuseNum++;
                        MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, 1, RGB888_RED, DRAW_RETC_THICK);   
                        offset_box += snprintf_s(osdBuf_box + offset_box, sizeof(osdBuf_box) - offset_box, sizeof(osdBuf_box) - offset_box - 1, 
                            "%u ", personID[i].box_ID);
                        offset_box += snprintf_s(osdBuf_box + offset_box, sizeof(osdBuf_box) - offset_box, sizeof(osdBuf_box) - offset_box - 1, 
                            "Focused");
                        HI_ASSERT(offset_box < sizeof(osdBuf_box));
                        x_ID=SAMPLE_SVP_NNIE_MIN(SAMPLE_SVP_NNIE_MAX(((BoxsIDprint[i].xmin)/2)*2,0),stBaseFrmInfo.stVFrame.u32Width-1);
                        y_ID=SAMPLE_SVP_NNIE_MIN(SAMPLE_SVP_NNIE_MAX(((BoxsIDprint[i].ymin-40)/2)*2,0),stBaseFrmInfo.stVFrame.u32Height-1);
                        //字体大小的设置：sample\taurus\ai_sample\mpp_help\src\osd_img.c，OSD_FONT_WIDTH_DEF与OSD_FONT_HEIGHT_DEF
                        TxtRgnInit(&rgn_ID[i], osdBuf_box, x_ID, y_ID, ARGB1555_BLUE); // font width and heigt use default 40
                        OsdsSetRgn(g_osdsPerson, g_osd_Person_ID[i], &rgn_ID[i]);                  
                    }
                    AD_attractiveness_total += personID[i].focused_time_total/personID[i].live_time;  
                    //printf("curlive id:%d,class:%d,totalfocustime:%f,livetime:%f\n",personID[i].box_ID,personID[i].cls,personID[i].focused_time_total,personID[i].live_time);              
                    //每个框的左上角加ID文字
                    // memset_s(osdBuf_box,BUF_LEN,0,BUF_LEN);
                    // offset_box=0;
                    // offset_box += snprintf_s(osdBuf_box + offset_box, sizeof(osdBuf_box) - offset_box, sizeof(osdBuf_box) - offset_box - 1, 
                    //     "%u", personID[i].box_ID);
                    // HI_ASSERT(offset_box < sizeof(osdBuf_box));
                    // x_ID=SAMPLE_SVP_NNIE_MIN(SAMPLE_SVP_NNIE_MAX(((BoxsIDprint[i].xmin)/2)*2,0),stBaseFrmInfo.stVFrame.u32Width);
                    // y_ID=SAMPLE_SVP_NNIE_MIN(SAMPLE_SVP_NNIE_MAX(((BoxsIDprint[i].ymin-40)/2)*2,0),stBaseFrmInfo.stVFrame.u32Height);
                    // //字体大小的设置：sample\taurus\ai_sample\mpp_help\src\osd_img.c，OSD_FONT_WIDTH_DEF与OSD_FONT_HEIGHT_DEF
                    // TxtRgnInit(&rgn_ID[i], osdBuf_box, x_ID, y_ID, ARGB1555_BLUE); // font width and heigt use default 40
                    // OsdsSetRgn(g_osdsPerson, g_osd_Person_ID[i], &rgn_ID[i]);
                }
                //printf("g_osd_person_ID[%d]:%d\n",i,g_osd_Person_ID[i]);
                //OsdsDestroyRgn(g_osdsPerson,g_osd_Person_ID[i]);
            }
            for(int i=prev_box_num;i<prev_rgn_num;i++)
            {
                TxtRgnInit(&rgn_ID[i], " ", 100, 100, ARGB1555_BLUE); // font width and heigt use default 40
                OsdsSetRgn(g_osdsPerson, g_osd_Person_ID[i], &rgn_ID[i]);
                //printf(">>>g_osd_person_ID[%d]:%d\n",i,g_osd_Person_ID[i]);
            }
            prev_rgn_num=prev_box_num;
            // for(int i=prev_box_num;i<max_ID_num;i++)
            // {
            //     TxtRgnInit(&rgn_ID[i], " ", 100, 100, ARGB1555_BLUE); // font width and heigt use default 40
            //     OsdsSetRgn(g_osdsPerson, g_osd_Person_ID[i], &rgn_ID[i]);
            //     //printf(">>>g_osd_person_ID[%d]:%d\n",i,g_osd_Person_ID[i]);
            // }
            // for(int i=0;i<5;i++)
            // {
            //     printf("g_osd_person_ID[%d]:%d\n",i,g_osd_Person_ID[i]);
            //     OsdsDestroyRgn(g_osdsPerson,g_osd_Person_ID[i]);
            // }

            //Draw rect
            // for (int j = 0; j < prev_box_num; j++) {
            //     remainingBoxs[0] = boxs[j];  
            //     if(personID[j].tracked_flag==HI_TRUE){
            //         if(objs[j].cls == 1){
            //             MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, 1, RGB888_GREEN, DRAW_RETC_THICK);
            //         }
            //         else if(objs[j].cls == 2){
            //             MppFrmDrawRects(&stBaseFrmInfo, remainingBoxs, 1, RGB888_RED, DRAW_RETC_THICK);
            //         }
            //     }
            // }
            MppFrmDestroy(&resizeFrm);

            /*OSD*/    

            offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1, "FocuseNum: %u", FocuseNum);
            HI_ASSERT(offset < sizeof(osdBuf));
            // offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1,
            //     "%u",FocuseNum);
            // HI_ASSERT(offset < sizeof(osdBuf));

            offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1, "; UnfocuseNum: %u", UnfocuseNum);
            HI_ASSERT(offset < sizeof(osdBuf));
            // offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1,
            //     "%u",UnfocuseNum);
            // HI_ASSERT(offset < sizeof(osdBuf));

            offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1, "; IDTotalNum: %u", ID_num_total);
            HI_ASSERT(offset < sizeof(osdBuf));
            // offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1,
            //     "%u",ID_num_total);
            // HI_ASSERT(offset < sizeof(osdBuf));
            // offset += snprintf_s(osdBuf + offset, sizeof(osdBuf) - offset, sizeof(osdBuf) - offset - 1, "; FPS: %.1f", fps);
            // HI_ASSERT(offset < sizeof(osdBuf));

            if (strcmp(osdBuf, prevOsd) != 0) {
                HiStrxfrm(prevOsd, osdBuf, sizeof(prevOsd)); //就是把osdBuf字符串拷贝给prevOsd
                // HI_OSD_ATTR_S rgn;
                TxtRgnInit(&rgn, osdBuf, TXT_BEGX, TXT_BEGY, ARGB1555_RED); // font width and heigt use default 40
                OsdsSetRgn(g_osdsPerson, g_osd0Person, &rgn);
            }
            // if(ID_num_total>0)
            //     AD_attractiveness_mean= AD_attractiveness_total/ID_num_total;
            // else
            //     AD_attractiveness_mean=0;
            if(prev_box_num>0)
                AD_attractiveness_mean= AD_attractiveness_total/prev_box_num;
            else
                AD_attractiveness_mean=0;
            //UdpSend
            sprintf(buffsend,"Focus:%u,UnFocus:%u,Total:%f,Attractiveness:%f",
                FocuseNum,UnfocuseNum,focused_time_total/1000, AD_attractiveness_mean);
            sendto(sock,buffsend,strlen(buffsend),0,(struct sockaddr *)&s,sizeof(s));
            //UdpRecive

            // s32Ret = HI_MPI_VO_SendFrame(voLayer, voChn, &stBaseFrmInfo, s32MilliSec);
            // SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
            //     "HI_MPI_VO_SendFrame failed, Error(%#x)!\n", s32Ret);       
            
            s32Ret =  HI_MPI_VENC_SendFrame(0, &stBaseFrmInfo, s32MilliSec);
            SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
                "HI_MPI_VENC_SendFrame failed, Error(%#x)!\n", s32Ret); 
            
            //printf("SendstBaseFrmInfo:width:%d, height:%d, entype:%d\n",stBaseFrmInfo.stVFrame.u32Width,stBaseFrmInfo.stVFrame.u32Height,stBaseFrmInfo.stVFrame.enPixelFormat);
            BASE_RELEASE:
                s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp,as32VpssChn[0], &stBaseFrmInfo);
                if (HI_SUCCESS != s32Ret)
                {
                    SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n",
                        s32Ret,s32VpssGrp,as32VpssChn[0]);
                }

            EXT_RELEASE:
                s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp,as32VpssChn[1], &stExtFrmInfo);
                if (HI_SUCCESS != s32Ret)
                {
                    SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n",
                        s32Ret,s32VpssGrp,as32VpssChn[1]);
                }
            else if(g_uart_switch_flag == HI_FALSE){
                //UdpSend
                sprintf(buffsend,"Focus:%u,UnFocus:%u,Total:%f,Attractiveness:%f",
                    0,0,focused_time_total/1000, 0);
                sendto(sock,buffsend,strlen(buffsend),0,(struct sockaddr *)&s,sizeof(s));
                usleep(40*1000);
            }
        }
    }
    
    pthread_join(uart_tid, NULL);

    return HI_NULL;
}
/******************************************************************************
* function : Yolov2 vi to vo real time detection
******************************************************************************/
void SAMPLE_SVP_NNIE_Yolov2_Vivo(void)
{
 //   HI_CHAR *pcSrcFile = "./data/nnie_image/rgb_planar/dog_bike_car_416x416.bgr";
    HI_CHAR *pcModelName = "./data/nnie_model/detection/resnet18_best.wk";
    SAMPLE_SVP_NNIE_CFG_S   stNnieCfg = {0};
    SIZE_S stSize;
    PIC_SIZE_E enSize = PIC_CIF;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_CHAR acThreadName[16] = {0};

    /*udp*/
    struct sockaddr_in s1;
    int sock1;
    char buff1[1024]="";
    char buff2[1024]="";
    //协议：IPv4、IPv6
    //PF_INET PF_INET6
    //SOCK_DGRAM 无连接的
    //SOCK_STREAM TCP 面向连接的
    if((sock1 = socket(AF_INET,SOCK_DGRAM,0))==-1){
        perror("error");
    }
    memset(&s1,0,sizeof(s1));
    s1.sin_family = AF_INET;
    s1.sin_port = htons(3000);
    s1.sin_addr.s_addr = INADDR_ANY;
    if(bind(sock1,(struct sockaddr *)&s1,sizeof(s1))==-1)
    {
        perror("bind error");
    }
    socklen_t sock_len = sizeof(s1);

    /******************************************
        step 1: start vi vpss vo
    ******************************************/
    s_stYolov2Switch.bVenc = HI_TRUE;
    s_stYolov2Switch.bVo   = HI_TRUE;
    s32Ret = SAMPLE_COMM_IVE_StartViVpssVencVo_Yolov2_test(&s_stViConfig,&s_stYolov2Switch,&enSize);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, YOLOV2_FAIL_1,
        "Error(%#x),SAMPLE_COMM_IVE_StartViVpssVencVo failed!\n", s32Ret);

    InitShowAD(0);
    StartShowAD();

    while(1){
        int recvbyte1 = recvfrom(sock1,buff1,sizeof(buff1)-1,0,(struct sockaddr *)&s1,&sock_len);
        buff1[recvbyte1]= 0;
        if(!strcmp(buff1,"start_yolov2")){
            //     /*Sys init*/
            // SAMPLE_COMM_SVP_CheckSysInit();


            //s32Ret = SAMPLE_COMM_IVE_StartViVpssVencVo_Yolov2(&s_stViConfig,&s_stYolov2Switch,&enSize);

            s32Ret = SAMPLE_COMM_SYS_GetPicSize(enSize, &stSize);
            SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, YOLOV2_FAIL_1,
                "Error(%#x),SAMPLE_COMM_SYS_GetPicSize failed!\n", s32Ret);

            stSize.u32Width = 640;
            stSize.u32Height = 384;

            /******************************************
             step 2: init NNIE param
            ******************************************/
            stNnieCfg.pszPic= NULL;
            stNnieCfg.u32MaxInputNum = 1;
            stNnieCfg.u32MaxRoiNum = 0;
            stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0;//set NNIE core

            /*YOLOV2 */
            SAMPLE_SVP_NNIE_CFG_S *self;
            HI_U32 u32PicNum = 1;
            //HI_S32 s32Ret;

            self = (SAMPLE_SVP_NNIE_CFG_S*)malloc(sizeof(*self));
            HI_ASSERT(self);
            memset_s(self, sizeof(*self), 0x00, sizeof(*self));

            /*
            * 设置配置参数
            * Set configuration parameter
            */
            self->pszPic = NULL;
            self->u32MaxInputNum = u32PicNum; // max input image num in each batch
            self->u32MaxRoiNum = 0;
            self->aenNnieCoreId[0] = SVP_NNIE_ID_0; // set NNIE core

            /*Yolov2 Load model*/
            SAMPLE_SVP_TRACE_INFO("Yolov2 Load model!\n");
            s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName,&s_stYolov2Model);
            SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret,YOLOV2_FAIL_0,SAMPLE_SVP_ERR_LEVEL_ERROR,
                "Error,SAMPLE_COMM_SVP_NNIE_LoadModel failed!\n");

            /*Yolov2 parameter initialization*/
            /*Yolov2 software parameters are set in SAMPLE_SVP_NNIE_Yolov2_SoftwareInit,
            if user has changed net struct, please make sure the parameter settings in
            SAMPLE_SVP_NNIE_Yolov2_SoftwareInit function are correct*/
            SAMPLE_SVP_TRACE_INFO("Yolov2 parameter initialization!\n");
            s_stYolov2NnieParam.pstModel = &s_stYolov2Model.stModel;
            // s32Ret = SAMPLE_SVP_NNIE_Yolov2_ParamInit(&stNnieCfg,&s_stYolov2NnieParam,&s_stYolov2SoftwareParam);
            // SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret,YOLOV2_FAIL_0,SAMPLE_SVP_ERR_LEVEL_ERROR,
            //     "Error,SAMPLE_SVP_NNIE_Yolov2_ParamInit failed!\n");
            s32Ret = SampleSvpNnieYolov2ParamInit(self, &s_stYolov2NnieParam, &s_stYolov2SoftwareParam);
            SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, YOLOV2_FAIL_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                "Error,SampleSvpNnieYolov2ParamInit failed!\n");

            /******************************************
             step 3: Create work thread
            ******************************************/
            SAMPLE_SVP_TRACE_INFO("Yolov2 start!\n");
            snprintf(acThreadName, 16, "NNIE_ViToVo");
            prctl(PR_SET_NAME, (unsigned long)acThreadName, 0,0,0);
            s_bNnieStopSignal = HI_FALSE;
            /******************************************
            step 3: Create work thread
            ******************************************/
            pthread_create(&s_hNnieThread, 0, SAMPLE_SVP_NNIE_Yolov2_ViToVo_thread, NULL);	
            while(strcmp(buff1,"stop_yolov2")){
                int recvbyte1 = recvfrom(sock1,buff1,sizeof(buff1)-1,0,(struct sockaddr *)&s1,&sock_len);
                buff1[recvbyte1]= 0;                
            }
            //SAMPLE_PAUSE();                       
            s_bNnieStopSignal = HI_TRUE;
            // pthread_join(s_hNnieThread, HI_NULL);
            // s_hNnieThread = 0;
            usleep(200000);
            OsdsClear(g_osdsPerson);  
            if (0 != s_hNnieThread)
            {
                pthread_join(s_hNnieThread, HI_NULL);
                s_hNnieThread = 0;
            }

            SAMPLE_SVP_NNIE_Yolov2_Deinit(&s_stYolov2NnieParam,&s_stYolov2SoftwareParam,&s_stYolov2Model);
            memset(&s_stYolov2NnieParam,0,sizeof(SAMPLE_SVP_NNIE_PARAM_S));
            memset(&s_stYolov2SoftwareParam,0,sizeof(SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S));
            memset(&s_stYolov2Model,0,sizeof(SAMPLE_SVP_NNIE_MODEL_S));

            //SAMPLE_COMM_IVE_StopViVpssVencVo_Yolov2(&s_stViConfig,&s_stYolov2Switch);
            close(uartFdOur);
            uartFdOur = 0;   
            ID_num_total=0; 
            focused_time_total=0;        
        }    
    }
    SAMPLE_COMM_IVE_StopViVpssVencVo_Yolov2_test(&s_stViConfig,&s_stYolov2Switch);
    DeInitShowAD();
    // s_bNnieStopSignal = HI_FALSE;
    // /******************************************
    //   step 3: Create work thread
    //  ******************************************/
    // snprintf(acThreadName, 16, "NNIE_ViToVo");
    // prctl(PR_SET_NAME, (unsigned long)acThreadName, 0,0,0);
    // pthread_create(&s_hNnieThread, 0, SAMPLE_SVP_NNIE_Yolov2_ViToVo_thread, NULL);	
    //  SAMPLE_PAUSE();

    // s_bNnieStopSignal = HI_TRUE;
    // pthread_join(s_hNnieThread, HI_NULL);
    // s_hNnieThread = 0;
    // OsdsClear(g_osdsPerson);


YOLOV2_FAIL_1:
    SAMPLE_SVP_NNIE_Yolov2_Deinit(&s_stYolov2NnieParam,&s_stYolov2SoftwareParam,&s_stYolov2Model);

YOLOV2_FAIL_0:
    SAMPLE_COMM_IVE_StopViVpssVencVo_Yolov2(&s_stViConfig,&s_stYolov2Switch);

}

void SAMPLE_SVP_NNIE_Yolov2_Vivo_HandleSig(void)
{
    s_bNnieStopSignal = HI_TRUE;
    if (0 != s_hNnieThread)
    {
        pthread_join(s_hNnieThread, HI_NULL);
        s_hNnieThread = 0;
    }

    SAMPLE_SVP_NNIE_Yolov2_Deinit(&s_stYolov2NnieParam,&s_stYolov2SoftwareParam,&s_stYolov2Model);
    memset(&s_stYolov2NnieParam,0,sizeof(SAMPLE_SVP_NNIE_PARAM_S));
    memset(&s_stYolov2SoftwareParam,0,sizeof(SAMPLE_SVP_NNIE_YOLOV2_SOFTWARE_PARAM_S));
    memset(&s_stYolov2Model,0,sizeof(SAMPLE_SVP_NNIE_MODEL_S));

    SAMPLE_COMM_IVE_StopViVpssVencVo_Yolov2(&s_stViConfig,&s_stYolov2Switch);

}