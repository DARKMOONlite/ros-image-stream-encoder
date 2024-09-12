#ifndef FFMPEG_WRAPPER_HPP
#define FFMPEG_WRAPPER_HPP

extern "C"{ // this links to the C library
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <iostream>

class FFMPEGWrapper {
public:
    FFMPEGWrapper(const std::string& filename, AVCodecID codec_id)
    : filename_(filename){
        std::cout << "constructing FFMpeg object \n";
        av_register_all();

        // Set up output format and codec
        avformat_alloc_output_context2(&format_context_, nullptr, "mp4", filename.c_str());
        if (!format_context_) {
            std::cerr << "Could not create output context\n";
            exit(1);
        }

        codec_ = avcodec_find_encoder(codec_id); 
        // codec_ = avcodec_find_encoder_by_name("h264_nvmpi");
        std::cout << "selected codec id: " << codec_->id << "\n";
        if (!codec_) {
            std::cerr << "codec not found\n";
            exit(1);
        }

        codec_context_ = avcodec_alloc_context3(codec_);


    }



    ~FFMPEGWrapper() {
        av_write_trailer(format_context_);
        avcodec_free_context(&codec_context_);
        av_frame_free(&frame_);
        av_free(buffer_);
        sws_freeContext(sws_context_);
        avio_closep(&format_context_->pb);
        avformat_free_context(format_context_);
        
    }

    bool ffmpeg_init( int width, int height, int fps){ // this should be got when the first frame is received and things like width and height can be determined.
        width_ = width;
        height_ = height;
        fps_ = fps;

        codec_context_->width = width_; 
        codec_context_->height = height_;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
        codec_context_->time_base =  {1,fps_*1000}; //{1, fps_};
        codec_context_->framerate = {fps_, 1};

        // ------------------------------------------------ Open codec ------------------------------------------------
        if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
            std::cerr << "Could not open codec\n";
            return(false);
        }

        // ------------------------------------------- Add stream to output context -------------------------------------------
        AVStream* stream = avformat_new_stream(format_context_, codec_);
        stream->time_base = codec_context_->time_base;
        if(avcodec_parameters_from_context(stream->codecpar, codec_context_) < 0){
            std::cerr << "Could not copy codec parameters to stream\n";
            return(false);
        }

        // ------------------------------------------- Prepare frame and buffer -------------------------------------------
        frame_ = av_frame_alloc();
        frame_->format = AV_PIX_FMT_YUV420P;
        frame_->width = width_;
        frame_->height = height_;

        int buffer_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, width_, height_, 1);
        buffer_ = (uint8_t*)av_malloc(buffer_size * sizeof(uint8_t));
        av_image_fill_arrays(frame_->data, frame_->linesize, buffer_, AV_PIX_FMT_YUV420P, width_, height_, 1);

        sws_context_ = sws_getContext(width_, height_, AV_PIX_FMT_BGR24, width_, height_, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);


        // Open output file
        if (!(format_context_->oformat->flags & AVFMT_NOFILE)) {
            if (avio_open(&format_context_->pb, filename_.c_str(), AVIO_FLAG_WRITE) < 0) {
                std::cerr << "Could not open output file\n";
                exit(1);
            }
        }

        // Write file header
        if (avformat_write_header(format_context_, nullptr) < 0) {
            std::cerr << "Could not write file header\n";
            exit(1);
        }

        return(true);


    }

    void encodeFrame(const cv::Mat& image) {


        if (image.empty()) {
            std::cerr << "Received an empty frame\n";
            return;
        }
        std::cout << "Encoding frame\n";
        // Convert OpenCV image (BGR) to YUV
        const uint8_t* in_data[1] = {image.data};
        int in_linesize[1] = {image.step};
        std::cout << "sws_scale\n";
        sws_scale(sws_context_, in_data, in_linesize, 0, height_, frame_->data, frame_->linesize);

        // Set frame PTS
        std::cout << "Frame index: " << frame_index_ << std::endl;
        frame_->pts =  frame_index_*(codec_context_->time_base.den / codec_context_->time_base.num) / fps_;
        frame_index_++;

        // Send frame to encoder
        if (avcodec_send_frame(codec_context_, frame_) < 0) {
            std::cerr << "Error sending frame to encoder\n";
            return;
        }
        std::cout << "Frame sent to encoder\n";

        // ----------------------------------------- Receive packet from encoder -----------------------------------------
        AVPacket packet;
        av_init_packet(&packet);
        packet.data = nullptr;
        packet.size = 0;
        packet.duration = codec_context_->time_base.den / codec_context_->time_base.num * fps_;

        if (avcodec_receive_packet(codec_context_, &packet) == 0) {
            std::cout << "Received packet from encoder of size: " << packet.size << " for frame:" << frame_index_ << std::endl;
            packet.stream_index = 0;
            if(av_interleaved_write_frame(format_context_, &packet)){
                std::cerr << "Error writing frame to file\n";
            }
            av_packet_unref(&packet);
        }
        else {
            std::cerr << "Error receiving packet from encoder\n";
        }
        

    }

private:
    AVFormatContext* format_context_ = nullptr;
    AVCodecContext* codec_context_ = nullptr;
    AVCodec* codec_ = nullptr;
    SwsContext* sws_context_ = nullptr;
    AVFrame* frame_ = nullptr;
    uint8_t* buffer_ = nullptr;
    int width_, height_, fps_;
    int frame_index_ = 0;
    std::string filename_;
};

#endif // GENERAL_VIDEO_ENCODER_HPP
