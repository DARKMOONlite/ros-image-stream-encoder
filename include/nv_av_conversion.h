#ifndef NV_AV_CONVERSION_H
#define NV_AV_CONVERSION_H


#ifdef JETSON_PLATFORM 


extern "C"{ // this links to the C library
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}
#include "nvmpi.h"



    nvPacket av_to_nv_packet(AVPacket* av_packet){
        nvPacket nv_packet;
        nv_packet.payload_size = av_packet->size;
        nv_packet.payload = &av_packet->data;
        nv_packet.pts = av_packet->pts;
        return nv_packet;
    }

    AVPacket nv_to_av_packet(nvPacket* nv_packet){
        AVPacket av_packet;
        av_packet.size = nv_packet->payload_size;
        av_packet.data = &nv_packet->payload;
        return av_packet;
    }

    // AVFrame nv_to_av_frame(nvFrame* nv_frame){

    // }
    // nvFrame av_to_nv_frame(AVFrame* av_frame){

    // }




#endif

#endif // NV_AV_CONVERSION_H