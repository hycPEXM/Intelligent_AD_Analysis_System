/* ****************************************************** */
/* Author       : Joey Qiu */
/* Last modified: 2023-08-12 22:57 */
/* Email        : 2650982628@qq.com */
/* Filename     : ad_show.c */
/* Description  :  */
/* ****************************************************** */

#include "hi_debug.h"
#include "hi_comm_ive.h"
#include "sample_comm_ive.h"
#include <hi_type.h>

/*
 * 常量
 * Constant
 */
#define IMG_FULL_CHN    3 // Full channel / three channel, for YUV444, RGB888
#define IMG_HALF_CHN    2 // Half channel, for YUV420/422
#define THREE_TIMES     3
#define TWO_TIMES       2

/*
 * 常用数值单位
 * Commonly used numerical units
 */
#define HI_KB               1024
#define HI_MB               (1024 * 1024)
#define HI_MS_OF_SEC        1000 // 1s in milliseconds
#define HI_NS_OF_MS         1000000 // Nanoseconds in 1ms
#define HI_BYTE_BITS        8 // Number of bits in 1 byte
#define HI_OVEN_BASE        2 // Even base
#define HI_INT8_BITS        8 // 8-bit integer number of bits
#define HI_INT16_BITS       16 // 16-bit integer number of bits
#define HI_INT32_BITS       32 // 32-bit integer number of bits
#define HI_INT64_BITS       64 // The number of bits of a 64-bit integer
#define HI_PER_BASE         100

/*
 * 调试log等级
 * Debug log level
 */
#define HI_DLEV_NONE        0 // disable
#define HI_DLEV_ERR         1 // error
#define HI_DLEV_WARN        2 // warning
#define HI_DLEV_INFO        3 // informational
#define HI_DLEV_DBG         4 // debug normal
#define HI_DLEV_VERB        5 // debug vervose
#define HI_DLEV_BUTT        6

#define LOGI(format, ...) LOG_ONLY(HI_DLEV_INFO, format, ##__VA_ARGS__)

/*
 * 打印log文件格式
 * Log with file/name
 */
#define LOG_ONLY(lev, format, ...) do { \
    if (g_hiDbgLev >= (lev)) { \
        printf(format, ##__VA_ARGS__); \
    } \
}   while (0)

/*
 * 矩形坐标结构体定义
 * Rectangular coordinate structure definition
 */
typedef struct RectBox {
    int xmin;
    int ymin;
    int xmax;
    int ymax;
} RectBox;

/*
 * 对齐类型
 * Alignment type
 */
typedef enum AlignType {
    ALIGN_TYPE_2 = 2, // Align by 2 bytes
    ALIGN_TYPE_16 = 16, // Align by 16 bytes
    ALIGN_TYPE_32 = 32, // Align by 32 bytes
} AlignType;

typedef struct HiSampleIveColorSpaceConvInfo {
    IVE_SRC_IMAGE_S stSrc;
    FILE* pFpSrc;
    FILE* pFpDst;
} SampleIveColorSpaceConvInfo;

static SampleIveColorSpaceConvInfo g_stColorSpaceInfo;

/*
 * 调试等级
 * Debug level
 */
int g_hiDbgLev = HI_DLEV_INFO;

int HiAlign16(int num)
{
    return (((num) + 16 - 1) / 16 * 16); // 16: Align16
}

int HiAlign32(int num)
{
    return (((num) + 32 - 1) / 32 * 32); // 32: Align32
}

/*
 * 取路径的文件名部分
 * Take the file name part of the path
 */
const char* HiPathName(const char* path)
{
    HI_ASSERT(path);

    const char *p = strrchr(path, '/');
    if (p) {
        return p + 1;
    }
    return path;
}

/*
 * 计算通道的步幅
 * Calculate the stride of a channel
 */
static uint32_t IveCalStride(IVE_IMAGE_TYPE_E enType, uint32_t width, AlignType align)
{
    uint32_t size = 1;

    switch (enType) {
        case IVE_IMAGE_TYPE_U8C1:
        case IVE_IMAGE_TYPE_S8C1:
        case IVE_IMAGE_TYPE_S8C2_PACKAGE:
        case IVE_IMAGE_TYPE_S8C2_PLANAR:
        case IVE_IMAGE_TYPE_U8C3_PACKAGE:
        case IVE_IMAGE_TYPE_U8C3_PLANAR:
            size = sizeof(HI_U8);
            break;
        case IVE_IMAGE_TYPE_S16C1:
        case IVE_IMAGE_TYPE_U16C1:
            size = sizeof(HI_U16);
            break;
        case IVE_IMAGE_TYPE_S32C1:
        case IVE_IMAGE_TYPE_U32C1:
            size = sizeof(uint32_t);
            break;
        case IVE_IMAGE_TYPE_S64C1:
        case IVE_IMAGE_TYPE_U64C1:
            size = sizeof(uint64_t);
            break;
        default:
            break;
    }

    if (align == ALIGN_TYPE_16) {
        return HiAlign16(width * size);
    } else if (align == ALIGN_TYPE_32) {
        return HiAlign32(width * size);
    } else {
        HI_ASSERT(0);
        return 0;
    }
}

/*
 * 根据类型和大小创建缓存
 * Create IVE image buffer based on type and size
 */
int IveImgCreate(IVE_IMAGE_S* img,
    IVE_IMAGE_TYPE_E enType, uint32_t width, uint32_t height)
{
    HI_ASSERT(img);
    uint32_t oneChnSize;
    uint32_t size;
    int ret;

    if (memset_s(img, sizeof(*img), 0, sizeof(*img)) != EOK) {
        HI_ASSERT(0);
    }
    img->enType = enType;
    img->u32Width = width;
    img->u32Height = height;
    img->au32Stride[0] = IveCalStride(img->enType, img->u32Width, ALIGN_TYPE_16);

    switch (enType) {
        case IVE_IMAGE_TYPE_U8C1:
        case IVE_IMAGE_TYPE_S8C1: // Only 1 channel
            size = img->au32Stride[0] * img->u32Height;
            ret = HI_MPI_SYS_MmzAlloc(&img->au64PhyAddr[0], (void**)&img->au64VirAddr[0], NULL, NULL, size);
            SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != ret, ret, "Error(%#x), HI_MPI_SYS_MmzAlloc!\n", ret);
            break;
        /*
         * 大小相当于像素的1.5倍(3/2), 相当于2个通道
         * The size is equivalent to 1.5 times (3/2) of the pixel, which is equivalent to 2 channels
         */
        case IVE_IMAGE_TYPE_YUV420SP:
        /*
         * 大小相当于像素的2倍，相当于2个通道
         * The size is equivalent to 2 times the pixel, which is equivalent to 2 channels
         */
        case IVE_IMAGE_TYPE_YUV422SP:
            if (enType == IVE_IMAGE_TYPE_YUV420SP) {
                size = img->au32Stride[0] * img->u32Height * THREE_TIMES / TWO_TIMES;
            } else {
                size = img->au32Stride[0] * img->u32Height * TWO_TIMES;
            }
            ret = HI_MPI_SYS_MmzAlloc(&img->au64PhyAddr[0], (void**)&img->au64VirAddr[0], NULL, NULL, size);
            SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != ret, ret, "Error(%#x), HI_MPI_SYS_MmzAlloc!\n", ret);

            /*
             * 设置通道1地址的步长，两者都需要通道1
             * Set the stride of the address of channel 1, both of which require channel 1
             */
            img->au32Stride[1] = img->au32Stride[0];
            img->au64PhyAddr[1] = img->au64PhyAddr[0] + img->au32Stride[0] * (uint64_t)img->u32Height;
            img->au64VirAddr[1] = img->au64VirAddr[0] + img->au32Stride[0] * (uint64_t)img->u32Height;
            break;

        case IVE_IMAGE_TYPE_U8C3_PLANAR: // 3 channels, often used for RGB
            oneChnSize = img->au32Stride[0] * img->u32Height;
            size = oneChnSize * 3; // 3 channels have the same size
            ret = HI_MPI_SYS_MmzAlloc(&img->au64PhyAddr[0], (void**)&img->au64VirAddr[0], NULL, NULL, size);
            SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != ret, ret, "Error(%#x), HI_MPI_SYS_MmzAlloc!\n", ret);

            /*
             * 设置通道1和通道2的地址和步长
             * Set the address and stride of channel 1 and channel 2
             */
            img->au64VirAddr[1] = img->au64VirAddr[0] + oneChnSize;
            img->au64PhyAddr[1] = img->au64PhyAddr[0] + oneChnSize;
            img->au32Stride[1] = img->au32Stride[0];
            img->au64VirAddr[2] = img->au64VirAddr[1] + oneChnSize; // 2: au64VirAddr array subscript, not out of bounds
            img->au64PhyAddr[2] = img->au64PhyAddr[1] + oneChnSize; // 2: au64VirAddr array subscript, not out of bounds
            img->au32Stride[2] = img->au32Stride[0]; // 2: au64VirAddr array subscript, not out of bounds
            break;

        /*
         * 目前如下格式不支持，主要为YVC420P, YUV422P, S8C2_PACKAGE, S8C2_PLANAR,
         * S32C1, U32C1, S64C1, U64C1, S16C1, U16C1, U8C3_PACKAGE等
         *
         * Types not currently supported: YVC420P, YUV422P, S8C2_PACKAGE, S8C2_PLANAR,
         * S32C1, U32C1, S64C1, U64C1, S16C1, U16C1, U8C3_PACKAGE,etc.
         */
        default:
            HI_ASSERT(0);
            break;
    }
    return HI_SUCCESS;
}

/*
 * 销毁IVE image
 * Destory IVE image
 */
void IveImgDestroy(IVE_IMAGE_S* img)
{
    for (int i = 0; i < IMG_FULL_CHN; i++) {
        if (img->au64PhyAddr[0] && img->au64VirAddr[0]) {
            HI_MPI_SYS_MmzFree(img->au64PhyAddr[i], (void*)((uintptr_t)img->au64VirAddr[i]));
            img->au64PhyAddr[i] = 0;
            img->au64VirAddr[i] = 0;
        }
    }
    if (memset_s(img, sizeof(*img), 0, sizeof(*img)) != EOK) {
        HI_ASSERT(0);
    }
}

/*
 * 函数：色彩转换去初始化
 * function : color convert uninit
 */
static HI_VOID SampleIveColorConvertUninit(SampleIveColorSpaceConvInfo* pstColorConvertInfo)
{
    IveImgDestroy(&pstColorConvertInfo->stSrc);

    IVE_CLOSE_FILE(pstColorConvertInfo->pFpSrc);
    /* IVE_CLOSE_FILE(pstColorConvertInfo->pFpDst); */
}


/*
 * 函数：色彩转换初始化
 * function : color convert init
 */
static HI_S32 SampleIveColorConvertInit_test(SampleIveColorSpaceConvInfo* g_stColorSpaceInfo,
    HI_CHAR* pchSrcFileName,  HI_U32 u32Width, HI_U32 u32Height)
{
    HI_S32 s32Ret;

    memset_s(g_stColorSpaceInfo, sizeof(SampleIveColorSpaceConvInfo), 0, sizeof(SampleIveColorSpaceConvInfo));

    s32Ret = SAMPLE_COMM_IVE_CreateImage(&g_stColorSpaceInfo->stSrc, IVE_IMAGE_TYPE_YUV420SP, u32Width, u32Height);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, COLOR_CONVERT_INIT_FAIL,
        "Error(%#x), Create Src Image failed!\n", s32Ret);

    s32Ret = HI_FAILURE;
    printf("%s\n",pchSrcFileName);
    g_stColorSpaceInfo->pFpSrc = fopen(pchSrcFileName, "rb");
    SAMPLE_CHECK_EXPR_GOTO(HI_NULL == g_stColorSpaceInfo->pFpSrc, COLOR_CONVERT_INIT_FAIL,
        "Error, Open file %s failed!\n", pchSrcFileName);

 /*    g_stColorSpaceInfo->pFpDst = fopen(pchDstFileName, "wb"); */
    /* SAMPLE_CHECK_EXPR_GOTO(HI_NULL == g_stColorSpaceInfo->pFpDst, COLOR_CONVERT_INIT_FAIL, */
        /* "Error, Open file %s failed!\n", pchDstFileName); */

    s32Ret = HI_SUCCESS;

COLOR_CONVERT_INIT_FAIL:

    if (HI_SUCCESS != s32Ret) {
        SampleIveColorConvertUninit(g_stColorSpaceInfo);
    }
    return s32Ret;
}




static HI_S32 SampleIveReadFile(SampleIveColorSpaceConvInfo* g_stColorSpaceInfo)
{
    HI_S32 s32Ret = SAMPLE_COMM_IVE_ReadFile(&(g_stColorSpaceInfo->stSrc), g_stColorSpaceInfo->pFpSrc);
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret, s32Ret, "Error(%#x), Read src file failed!\n", s32Ret);
    return s32Ret;
}

/*
 * 将image由RGB格式转成BGR格式
 * Convert image from RGB format to BGR format
 */
int ImgRgbToBgr(IVE_IMAGE_S *img)
{
    uint8_t *rp = NULL;
    uint8_t *bp = NULL;
    uint8_t c;
    int i, j;

    HI_ASSERT(img->enType == IVE_IMAGE_TYPE_U8C3_PLANAR);
    HI_ASSERT(img->au32Stride[0] >= img->u32Width);
    HI_ASSERT(img->au32Stride[1] >= img->u32Width);
    HI_ASSERT(img->au32Stride[2] >= img->u32Width); // 2: au32Stride array subscript, not out of bounds

    rp = (uint8_t*)(uintptr_t)img->au64VirAddr[0];
    bp = (uint8_t*)(uintptr_t)img->au64VirAddr[2]; // 2: VirAddr array subscript, not out of bounds
    HI_ASSERT(rp && bp);
    for (i = 0; i < img->u32Height; i++) {
        for (j = 0; j < img->u32Width; j++) {
            c = rp[j];
            rp[j] = bp[j];
            bp[j] = c;
        }
        rp += img->au32Stride[0];
        bp += img->au32Stride[2]; // 2: au32Stride array subscript, not out of bounds
    }
    return 0;
}

/*
 * VIDEO_FRAME_INFO_S格式转换成IVE_IMAGE_S格式
 * 复制数据指针，不复制数据
 *
 * Video frame to IVE image.
 * Copy the data pointer, do not copy the data.
 */
int FrmToOrigImg(const VIDEO_FRAME_INFO_S* frm, IVE_IMAGE_S *img)
{
    static const int chnNum = 2; // Currently only supports YUV420/422, so only the addresses of 2 channels are copied
    PIXEL_FORMAT_E pixelFormat = frm->stVFrame.enPixelFormat;

    if (memset_s(img, sizeof(*img), 0, sizeof(*img)) != EOK) {
        HI_ASSERT(0);
    }

    img->u32Width = frm->stVFrame.u32Width;
    img->u32Height = frm->stVFrame.u32Height;

    if (pixelFormat == PIXEL_FORMAT_YVU_SEMIPLANAR_420) {
        img->enType = IVE_IMAGE_TYPE_YUV420SP;
    } else if (pixelFormat == PIXEL_FORMAT_YVU_SEMIPLANAR_422) {
        img->enType = IVE_IMAGE_TYPE_YUV422SP;
    } else {
        HI_ASSERT(0);
        return -1;
    }

    for (int i = 0; i < chnNum; i++) {
        img->au64PhyAddr[i] = frm->stVFrame.u64PhyAddr[i];
        img->au64VirAddr[i] = frm->stVFrame.u64VirAddr[i];
        img->au32Stride[i] = frm->stVFrame.u32Stride[i];
    }
    return 0;
}



static hi_s32
get_nv12_frame(VIDEO_FRAME_INFO_S *frame, VB_BLK *vb_blk, hi_s32 width, hi_s32 height)
{

    VB_CAL_CONFIG_S calc_cfg;


    // printf("%d:%d\n",width,height);
    COMMON_GetPicBufferConfig(width,height,SAMPLE_PIXEL_FORMAT,
    COMPRESS_MODE_NONE,DATA_BITWIDTH_8,0,&calc_cfg);

    hi_s32 head_y_size = calc_cfg.u32HeadYSize;
    hi_s32 head_size = calc_cfg.u32HeadSize;
    hi_s32 head_stride = calc_cfg.u32HeadStride;
    hi_s32 vb_size = calc_cfg.u32MainSize;
    hi_s32 main_y_size = calc_cfg.u32MainYSize;
    hi_s32 main_stride = calc_cfg.u32MainStride;

    // printf("size:%d\n",vb_size);

    VB_BLK temp_vb_blk = HI_MPI_VB_GetBlock(VB_INVALID_POOLID, vb_size, HI_NULL);
    if (temp_vb_blk == VB_INVALID_HANDLE) {
        printf("get vb blk(size:%d) failed!\n", vb_size);
        return HI_FAILURE;
    }

    frame->stVFrame.u64HeaderPhyAddr[0] = HI_MPI_VB_Handle2PhysAddr(temp_vb_blk);
    if (frame->stVFrame.u64HeaderPhyAddr[0] == HI_NULL) {
        printf("HI_MPI_VB_GetBlock failed\n");
        HI_MPI_VB_ReleaseBlock(temp_vb_blk);
        return HI_FAILURE;
    }
    printf("%llu\n",frame->stVFrame.u64HeaderPhyAddr[0]);

    frame->stVFrame.u64HeaderVirAddr[0] = HI_MPI_SYS_Mmap(frame->stVFrame.u64HeaderPhyAddr[0], vb_size);
    if (frame->stVFrame.u64HeaderVirAddr[0] == HI_NULL) {
        printf("ss_mpi_sys_mmap failed\n");
        HI_MPI_VB_ReleaseBlock(temp_vb_blk);
        return HI_FAILURE;
    }

    frame->enModId = HI_ID_IVE;
    frame->u32PoolId =  HI_MPI_VB_Handle2PoolId(temp_vb_blk);
    // printf("id:%d\n",frame->u32PoolId);

    printf("main_y_size:%d\n",main_y_size);
    frame->stVFrame.u64HeaderPhyAddr[1] =
        frame->stVFrame.u64HeaderPhyAddr[0] + head_y_size;
    frame->stVFrame.u64HeaderVirAddr[1] =
        frame->stVFrame.u64HeaderVirAddr[0] + head_y_size;
    frame->stVFrame.u64PhyAddr[0] =
        frame->stVFrame.u64HeaderPhyAddr[0] + head_size;
    frame->stVFrame.u64PhyAddr[1] =
        frame->stVFrame.u64PhyAddr[0] + main_y_size;
    frame->stVFrame.u64VirAddr[0] =
        frame->stVFrame.u64HeaderVirAddr[0] + head_size;
    frame->stVFrame.u64VirAddr[1] =
        frame->stVFrame.u64VirAddr[0] + main_y_size;
    frame->stVFrame.u32HeaderStride[0] = head_stride;
    frame->stVFrame.u32HeaderStride[1] = head_stride;
    frame->stVFrame.u32Stride[0] = main_stride;
    frame->stVFrame.u32Stride[1] = main_stride;

    frame->stVFrame.u32Width = width;
    frame->stVFrame.u32Height = height;
    frame->stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8;
    frame->stVFrame.enCompressMode = COMPRESS_MODE_NONE;
    frame->stVFrame.enVideoFormat = VIDEO_FORMAT_LINEAR;
    frame->stVFrame.enField = VIDEO_FIELD_FRAME;
    frame->stVFrame.enColorGamut = COLOR_GAMUT_BT601;
    frame->stVFrame.enPixelFormat = SAMPLE_PIXEL_FORMAT;

    *vb_blk = temp_vb_blk;

    return HI_SUCCESS;
}

static hi_s32
put_nv12_frame(VIDEO_FRAME_INFO_S *frame, VB_BLK vb_blk)
{
    hi_s32 vb_size = frame->stVFrame.u32Width * frame->stVFrame.u32Height * 3 / 2;

    HI_MPI_SYS_Munmap(frame->stVFrame.u64HeaderVirAddr[0], vb_size);
    hi_s32 ret = HI_MPI_VB_ReleaseBlock(vb_blk);
    if (ret != HI_SUCCESS) {
        printf("release vb failed %#x!\n",ret);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

/*
 * Sample实现将IVE image格式转成video frame格式
 * Sample implements converting IVE image format into video frame format
 */
static SampleIveColorSpaceConvInfo g_stColorSpaceInfo;
static SAMPLE_VO_CONFIG_S voconfig = {0};
static VB_CONFIG_S vbconfig={0};

/*
 * Sample实现将IVE image格式转成video frame格式
 * Sample implements converting IVE image format into video frame format
 */
HI_VOID StVbParamCfg(VB_CONFIG_S *self,HI_U32 h,HI_U32 w)
{
    memset_s(&vbconfig, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    // 2: The number of buffer pools that can be accommodated in the entire system
    self->u32MaxPoolCnt              = 128;

    /*
     * 获取一帧图片的buffer大小
     * Get picture buffer size
     */
    HI_U64 size = COMMON_GetPicBufferSize(1920, 1080,
        SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);

    self->astCommPool[0].u64BlkSize  = size;    // 10: Number of cache blocks per cache pool. Value range: (0, 10240]
    self->astCommPool[0].u32BlkCnt   = 16;

    /*
     * 获取raw buffer的大小
     * Get raw buffer size
     */
    self->astCommPool[1].u64BlkSize  = VI_GetRawBufferSize(w, h,
        PIXEL_FORMAT_RGB_BAYER_16BPP, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    // 4: Number of cache blocks per cache pool. Value range: (0, 10240]
    self->astCommPool[1].u32BlkCnt   = 4;

}


hi_s32 InitSystyem(HI_U32 h,HI_U32 w){

    sdk_init();
    VO_LAYER voLayer = 0;
    VO_CHN voChn = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    // const HI_S32 s32MilliSec = -1; /* 20000ms timeout */

    
    StVbParamCfg(&vbconfig, h, w);

    s32Ret = SAMPLE_COMM_SYS_Init(&vbconfig);
    SAMPLE_CHECK_EXPR_RET(s32Ret != HI_SUCCESS ,s32Ret,"Error(%#x),SAMPLE_COMM_SYS_Init failed!\n",
            s32Ret);

    SAMPLE_COMM_VO_GetDefConfig(&voconfig);
    voconfig.enPicSize = PIC_1080P;
    s32Ret = SAMPLE_COMM_VO_StartVO(&voconfig);
    // s32Ret = SAMPLE_COMM_IVE_StartVo();
    SAMPLE_CHECK_EXPR_RET(s32Ret != HI_SUCCESS ,s32Ret,"Error(%#x),SAMPLE_COMM_IVE_StartVo failed!\n",
            s32Ret);
    return HI_SUCCESS;
}

hi_void DeInitSystem(){
    SAMPLE_COMM_VO_StopVO(&voconfig);
    // SAMPLE_COMM_IVE_StopVo();
    SAMPLE_COMM_SYS_Exit();
    memset_s(&vbconfig, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    sdk_exit();
}

hi_s32 BufferToFrm(const HI_CHAR* filename, HI_U32 w, HI_U32 h)
{
   const HI_S32 s32MilliSec = 20000; /* 20000ms timeout */

    VIDEO_FRAME_INFO_S frm;
    HI_S32 s32Ret;

   /*
     * 初始化g_stColorSpaceInfo结构体
     * Initialize the g_stColorSpaceInfo structure
     */
    printf("%d:%d\n",w,h);
    memset_s(&g_stColorSpaceInfo, sizeof(g_stColorSpaceInfo), 0, sizeof(g_stColorSpaceInfo));

    
    s32Ret = SampleIveColorConvertInit_test(&g_stColorSpaceInfo, filename,  w, h);
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret, s32Ret,
        "Error(%#x), SampleIveColorConvertInit failed!\n", s32Ret);

    s32Ret = SampleIveReadFile(&g_stColorSpaceInfo);
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret, s32Ret, "Error(%#x), SampleIveReadFile failed!\n", s32Ret);

    VB_BLK vb_blk = VB_INVALID_HANDLE;
    s32Ret = get_nv12_frame(&frm,&vb_blk,w,h);
    if(s32Ret != HI_SUCCESS){
        printf("get nv12 frame failed\n");
        return HI_FAILURE;
    }


    printf("%d\n",g_stColorSpaceInfo.stSrc.au64VirAddr[0]);
    memcpy((void*)frm.stVFrame.u64HeaderVirAddr[0],(void*)g_stColorSpaceInfo.stSrc.au64VirAddr[0],
            w*h*1.5);

    s32Ret = HI_MPI_VO_SendFrame(0, 0, &frm, s32MilliSec);

    put_nv12_frame(&frm,vb_blk);
    memset_s(&frm, sizeof(VIDEO_FRAME_INFO_S), 0, sizeof(VIDEO_FRAME_INFO_S));
    IveImgDestroy(&g_stColorSpaceInfo.stSrc);
    IVE_CLOSE_FILE(g_stColorSpaceInfo.pFpDst);
    memset_s(&g_stColorSpaceInfo, sizeof(g_stColorSpaceInfo), 0, sizeof(g_stColorSpaceInfo));
       
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret, s32Ret, "Error(%#x), SendFrame failed!\n", s32Ret);

    return HI_SUCCESS; 
}

/* void ad_show_main(){
    sdk_init();
    VO_LAYER voLayer = 0;
    VO_CHN voChn = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    const HI_S32 s32MilliSec = -1; [>20000ms timeout<]
    HI_CHAR* pchSrcFileName = "./data/input/color_convert_img/0_nv12.bin";
    // VIDEO_FRAME_INFO_S adfrm;
    SAMPLE_VO_CONFIG_S voconfig1 = {0};
    VB_CONFIG_S vbconfig1;
    
    (HI_VOID)memset_s(&vbconfig1, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    vbconfig1.u32MaxPoolCnt = 128; [>max pool count: 128<]
    // StVbParamCfg(&vbconfig);
    unsigned long size  = COMMON_GetPicBufferSize(1920, 1080,
        SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    vbconfig1.astCommPool[0].u64BlkSize = (HI_U64)size;
    // 10: Number of cache blocks per cache pool. Value range: (0, 10240]
    vbconfig1.astCommPool[0].u32BlkCnt   = 16;

    // vbconfig1.astCommPool[1].u64BlkSize = (HI_U64)size;
    // // 10: Number of cache blocks per cache pool. Value range: (0, 10240]
    // vbconfig1.astCommPool[1].u32BlkCnt   = 16;

    // vbconfig1.astCommPool[1].u64BlkSize  = VI_GetRawBufferSize(1920, 1080,
    //     PIXEL_FORMAT_RGB_BAYER_16BPP, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    // // 4: Number of cache blocks per cache pool. Value range: (0, 10240]
    // vbconfig1.astCommPool[1].u32BlkCnt   = 4;


    // printf("%llu\n",(HI_U64)vbconfig1.astCommPool[0].u64BlkSize);
    s32Ret = SAMPLE_COMM_SYS_Init(&vbconfig1);
    SAMPLE_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS ,"Error(%#x),SAMPLE_COMM_SYS_Init failed!\n",
            s32Ret);
    SAMPLE_COMM_VO_GetDefConfig(&voconfig);
    voconfig.enPicSize = PIC_1080P;
    SAMPLE_COMM_VO_StartVO(&voconfig);
    // s32Ret = SAMPLE_COMM_IVE_StartVo();
    SAMPLE_CHECK_EXPR_RET_VOID(s32Ret != HI_SUCCESS ,"Error(%#x),SAMPLE_COMM_IVE_StartVo failed!\n",
            s32Ret);
    // printf("1111111111\n");
    // usleep(10*1000 * 1000);
    SampleIveOrigImgToFrm_test(pchSrcFileName,1920,1080); 
    usleep(10*1000 * 1000);
    [>s32Ret = HI_MPI_VO_SendFrame(voLayer, voChn, &adfrm, s32MilliSec);<]
        // SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
        //     "HI_MPI_VO_SendFrame failed, Error(%#x)!\n", s32Ret);
    //s32Ret = HI_MPI_VO_GetScreenFrame(voLayer,&adfrm, s32MilliSec);
    SAMPLE_COMM_VO_StopVO(&voconfig1);
    // SAMPLE_COMM_IVE_StopVo();
    
    sdk_exit();
} */
