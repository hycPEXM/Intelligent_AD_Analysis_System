// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-05 17:02
// Email        : 2650982628@qq.com
// Filename     : nnie_rtsp.h
// Description  : 
// ******************************************************

#ifdef NNIE_RTSP
#define NNIE_RTSP

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#define RTSP_PRT(fmt...) \
    do { \
        printf("[%s]-%d: ", __FUNCTION__, __LINE__); \
        printf(fmt); \
    } while (0)

int NNIE_START_RTSP(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
