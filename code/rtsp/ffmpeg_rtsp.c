/* ****************************************************** */
/* Author       : Joey Qiu */
/* Last modified: 2023-08-06 19:47 */
/* Email        : 2650982628@qq.com */
/* Filename     : ffmpeg_rtsp.c */
/* Description  :  */
/* ****************************************************** */

#include "ffmpeg_rtsp.h"
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <stdint.h>

// 全局变量声明
static AVFormatContext *fmt_ctx = NULL;
static AVOutputFormat *out_fmt = NULL;
static AVStream *video_st = NULL;
static AVCodecContext *codec_ctx = NULL;
static AVPacket pkt;

// 初始化 RTSP 流
int init_rtsp_stream(const char* rtsp_path,int h,int w,int fr) {

    int ret;
    // av_register_all();
    if((ret = avformat_network_init()) < 0)
    {
            fprintf(stderr, "avformat_network_init failed!");
            return -1;
    }

    // 创建 AVFormatContext
    avformat_alloc_output_context2(&fmt_ctx, NULL, "rtsp", rtsp_path);

    if (!fmt_ctx) {
        printf("Failed to allocate output context\n");
        return -1;
    }

    // 查找 H.264 编码器
    // AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);

    // if (!codec) {
    //     printf("Codec not found\n");
    //     return -1;
    // }
    
    // 创建视频流
    video_st = avformat_new_stream(fmt_ctx, NULL);
    if (!video_st) {
        printf("Failed to create stream\n");
        return -1;
    }

    codec_ctx = avcodec_alloc_context3(NULL);
    if (!codec_ctx) {
        printf("Failed to allocate codec context\n");
        return -1;
    }

    // 设置编码参数
    codec_ctx->codec_id = AV_CODEC_ID_H264;
    codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_ctx->pix_fmt = AV_PIX_FMT_NV12;
    codec_ctx->width = w;
    codec_ctx->height = h;
    codec_ctx->bit_rate = w * h * fr * 8 * 1.5;
    codec_ctx->time_base = (AVRational){1, fr};

    // // 打开编码器
    // ret = avcodec_open2(codec_ctx, codec, NULL);
    // if (ret < 0) {
    //     printf("Failed to open codec\n");
    //     return -1;
    // }

    av_dump_format(fmt_ctx,0,rtsp_path,1);

    // 设置流参数
    ret = avcodec_parameters_from_context(video_st->codecpar, codec_ctx);
    if (ret < 0) {
        printf("Failed to copy codec parameters to stream\n");
        return -1;
    }

    // 打开输出 URL
    /* ret = avio_open(&fmt_ctx->pb, rtsp_path, AVIO_FLAG_WRITE);
    if (ret < 0) {
        printf("Failed to open output URL\n");
        return -1;
    } */

    // 打开输出URL（Open output URL）
    if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&fmt_ctx->pb, rtsp_path, AVIO_FLAG_WRITE);
        if (ret < 0) {
            printf( "Could not open output URL %s ,error code:%d \n", rtsp_path,ret);
            return -1;
        }
    }


    // 写入文件头
    ret = avformat_write_header(fmt_ctx, NULL);
    if (ret < 0) {
        printf("Failed to write header\n");
        return -1;
    }

    return 0;
}

// 发送数据包到 RTSP 服务器
int send_packet_to_server(uint8_t*data,uint32_t size,int is_not_key,uint64_t count) {
    pkt.data = data;
    pkt.size = size;
    if(!is_not_key)
        pkt.flags = AV_PKT_FLAG_KEY;
    /* pkt.pts = count; */
    /* pkt.dts = count; */
    pkt.pts = 0;
    pkt.dts = 0;
    pkt.stream_index = video_st->index;
    int ret = av_write_frame(fmt_ctx, &pkt);
    if (ret < 0) {
        printf("Failed to write frame\n");
        return -1;
    }
    
    return 0;
}

// 去初始化 RTSP 流
void deinit_rtsp_stream() {
    // 写入文件尾
    av_write_trailer(fmt_ctx);

    // 释放资源
    avcodec_free_context(&codec_ctx);
    avio_close(fmt_ctx->pb);
    avformat_free_context(fmt_ctx);
}

