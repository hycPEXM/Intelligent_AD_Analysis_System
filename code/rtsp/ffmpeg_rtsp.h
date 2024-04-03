// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-08-06 20:25
// Email        : 2650982628@qq.com
// Filename     : ffmpeg_rtsp.h
// Description  : 
// ******************************************************

#ifndef FFMPEG_RTSP_H
#define FFMPEG_RTSP_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int init_rtsp_stream(const char* rtsp_path,int h,int w,int fr);

int send_packet_to_server(uint8_t*data,uint32_t size,int is_key,uint64_t count);

void deinit_rtsp_stream();
#endif
