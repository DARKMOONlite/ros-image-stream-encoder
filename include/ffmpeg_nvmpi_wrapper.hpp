#ifndef FFMPEG_NVMPI_WRAPPER_H
#define FFMPEG_NVMPI_WRAPPER_H

#ifdef JETSON_PLATFORM // this is defined in the CMakeLists.txt file and thus only runs if the code is being compiled on a Jetson platform


#include "nvmpi.h"
#include <ffmpeg_wrapper.hpp>
#include "nv_av_conversion.h"
    /**
     * @brief This class completely overrides the regular FFMPEG and adds functionaility for the NVMPi encoder
     * 
     */
    class FFMPEGNVMPIWrapper : public FFMPEGWrapper {

    public:

        FFMPEGNVMPIWrapper(const std::string& filename, AVCodecID codec_id):FFMPEGWrapper(filename,codec_id) {
            coding_type_ = convertCodecID(codec_id);
            std::cout << "constructing FFMpeg object with NVMPI headers \n";
        }
        ~FFMPEGNVMPIWrapper(){
            nvmpi_encoder_close(nvmpi_ctx_);
        }

        

        bool ffmpeg_init(int width, int height, int fps) override {
            nvmpi_ctx_ = nvmpi_create_encoder(coding_type_,enc_param_);

            FFMPEGWrapper::ffmpeg_init(width, height, fps);



        }

        void encodeFrame(const cv::Mat& image) override {
            std::cout << "encodeFrame\n";
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

            if(nvmpi_encoder_put_frame(nvmpi_ctx_,&frame) < 0){
                std::cerr << "Error encoding frame\n";
            }

            nvPacket packet;
            if(nvmpi_encoder_get_packet(nvmpi_ctx_,&packet) < 0){
                std::cerr << "Error getting packet\n";
            }
            std::cout << "end encodeFrame\n";

            AVPacket av_packet = nv_to_av_packet(&packet);

            if(av_interleaved_write_frame(format_context_, &av_packet)){
                std::cerr << "Error writing frame to file\n";
            }
            av_packet_unref(&av_packet);

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