#ifndef SORT
#define SORT

#ifdef __cplusplus
extern "C" {
#endif

#include "misc_util.h"  //包含了ai_infer_process.h
#include "hi_common.h"

#define BOX_MAX_WIDTH    640
#define BOX_MAX_HEIGHT   384

typedef struct SORT_ID
{
    RectBox box;
    HI_U32 ID;
    int cls;
    HI_FLOAT focused_time_total;
    HI_FLOAT live_time;
} SORT_ID;

int Sort_Track_Yolov2(DetectObjInfo resBuf[], int resLen, float usedTime, SORT_ID ID_arr[], int maxBoxNum,
    int validBoxNum, HI_U32 frame, HI_FLOAT *AD_attractiveness_dead);

float global_focused_time_total();

int SORT_ID_num_total();

#ifdef __cplusplus
}
#endif
#endif