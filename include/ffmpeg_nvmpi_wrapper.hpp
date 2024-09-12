#ifndef FFMPEG_NVMPI_WRAPPER_H
#define FFMPEG_NVMPI_WRAPPER_H

#ifdef JETSON_PLATFORM


#include "nvmpi.h"
#include <ffmpeg_wrapper.hpp>

    /**
     * @brief This class completely overrides the regular FFMPEG and adds functionaility for the NVMPi encoder
     * 
     */
    class FFMPEGNVMPIWrapper : public FFMPEGWrapper {

    public:

        FFMPEGNVMPIWrapper(const std::string& filename, AVCodecID codec_id):FFMPEGWrapper(filename,codec_id) {
            coding_type_ = convertCodecID(codec_id);
        }
        ~FFMPEGNVMPIWrapper(){
            nvmpi_encoder_close(nvmpi_ctx_);
        }

        

        bool ffmpeg_init(int width, int height, int fps) override {
            nvmpi_ctx_ = nvmpi_create_encoder(coding_type_,enc_param_);
        }

        void encodeFrame(const cv::Mat& image) override {
            // convert image to nvFrame
            nvFrame frame;
            frame.width = image.cols;
            frame.height = image.rows;
            frame.type = NV_PIX_YUV420;
            frame.timestamp = 0;
            frame.flags = 0;
            frame.payload_size[0] = image.cols * image.rows * 3;
            frame.payload[0] = image.data;
            frame.linesize[0] = image.cols * 3;

            nvmpi_encoder_put_frame(nvmpi_ctx_,&frame);

            nvPacket packet;
            nvmpi_encoder_get_packet(nvmpi_ctx_,&packet);

        }

        private:
        nvCodingType coding_type_;
        nvEncParam * enc_param_;
        nvmpictx * nvmpi_ctx_;

        nvCodingType convertCodecID(AVCodecID codec_id) {
            switch(codec_id) {
                case AV_CODEC_ID_H264:
                    return NV_VIDEO_CodingH264;
                case AV_CODEC_ID_MPEG4:
                    return NV_VIDEO_CodingMPEG4;
                case AV_CODEC_ID_MPEG2VIDEO:
                    return NV_VIDEO_CodingMPEG2;
                case AV_CODEC_ID_VP8:
                    return NV_VIDEO_CodingVP8;
                case AV_CODEC_ID_VP9:
                    return NV_VIDEO_CodingVP9;
                case AV_CODEC_ID_HEVC:
                    return NV_VIDEO_CodingHEVC;
                default:
                    return NV_VIDEO_CodingUnused;
            }
        }


    };


#endif

#endif