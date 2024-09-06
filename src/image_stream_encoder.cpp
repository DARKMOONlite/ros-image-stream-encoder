#include "image_stream_encoder.hpp"
#include <iomanip>
#include <ctime>
#include <sstream>
ImageStreamEncoder::ImageStreamEncoder() {

    nh = ros::NodeHandle("~");

    // ------------------------ FILE NAME ------------------------

    std::stringstream output_file;
    { // scope block to limit the scope of the time variables
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        output_file << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S"); 
    }
    if(!nh.getParam("video_format", video_format)){
        ROS_WARN("No output_file parameter found, defaulting to .mp4");
        video_format = ".mp4";
    }
    if(!nh.getParam("output-prepend", output_prepend)){
        ROS_INFO("No output-prepend, file name defaulting to %s", output_file.str().c_str());
        
    }
    std::stringstream temp;
    temp << output_prepend << output_file.str() << video_format;
    full_filename = temp.str();

    // ------------------------ BATCH PROCESSING ------------------------

    if(!nh.getParam("batch", batch_processing_)){
        ROS_WARN("No batch parameter found, defaulting to false");
        batch_processing_ = false;
    }

    if(!nh.getParam("batch-period", batch_period_)){
        ROS_INFO("No batch-rate parameter found, defaulting to 2 seconds");
        batch_period_ = 2;
    }


    // ------------------------- SUBSCRIBER -------------------------

    if(batch_processing_){
        ROS_INFO("Batch processing enabled");
        
        image_stream = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::Image>("image_stream", 10, [this](sensor_msgs::Image::ConstPtr image_msg){
            this->image_callback(image_msg);
        }));

        timer = nh.createTimer(ros::Duration(batch_period_), [this](const ros::TimerEvent&){
            this->encode_image();
        });


    } else {
        ROS_INFO("Batch processing disabled");
        image_stream = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::Image>("image_stream", 10, [this](sensor_msgs::Image::ConstPtr image_msg){
            this->image_and_encode(image_msg);
        }));
    }



    // ------------------------ AV LIBRARIES ------------------------
    ffmpeg_init();

}

ImageStreamEncoder::~ImageStreamEncoder() {
    // Free all allocated resources
    av_write_trailer(output_format_context);
    avformat_free_context(output_format_context);
    delete output_video_stream;
    
    
    avcodec_free_context(&output_codec_context);
    delete output_codec;
    delete output_codec_parameters;
}

void ImageStreamEncoder::image_and_encode(sensor_msgs::Image::ConstPtr image_msg) {
    // Callback function for image stream
    image_callback(image_msg);
    encode_image();
}

void ImageStreamEncoder::image_callback(sensor_msgs::Image::ConstPtr image_msg) {
    // Callback function for image stream with batching
    
}
/**
 * @brief this will be on a timer and happen every couple of seconds
 * 
 */
void ImageStreamEncoder::encode_image() {
    // Encode image stream

}


bool ImageStreamEncoder::ffmpeg_init() {
    av_register_all();

    avformat_alloc_output_context2(&output_format_context, nullptr, nullptr, full_filename.c_str());
        if (!output_codec_context) {
            std::cerr << "Could not create output context\n";
            ros::shutdown();

    output_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if(!output_codec){
        ROS_ERROR("Could not find encoder");
        ros::shutdown();
    }
    output_codec_context = avcodec_alloc_context3(output_codec);
    output_codec_context->width = image_width_;
    output_codec_context->height = image_height_;
    output_codec_context->pix_fmt = AV_PIX_FMT_YUV420P;
    int fps =25;
    output_codec_context->time_base = (AVRational){1, fps};
    output_codec_context->framerate = (AVRational){fps, 1};


    // Open the codec
    if(avcodec_open2(output_codec_context, output_codec, nullptr) < 0){
        ROS_ERROR("Could not open codec");
        ros::shutdown();
    }
    output_video_stream = avformat_new_stream(output_format_context, output_codec);
    output_video_stream->time_base = output_codec_context->time_base;
    output_video_stream->codecpar->codec_id = AV_CODEC_ID_H264;
    output_video_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    output_video_stream->codecpar->width = image_width_;
    output_video_stream->codecpar->height = image_height_;
    output_video_stream->codecpar->format = output_codec_context->pix_fmt;

    // open the file
    if(!(output_format_context->oformat->flags & AVFMT_NOFILE)){
        if(avio_open(&output_format_context->pb, full_filename.c_str(), AVIO_FLAG_WRITE) < 0){
            ROS_ERROR("Could not open file");
            ros::shutdown();
        }
    }
    // write the header
    if(avformat_write_header(output_format_context, nullptr) < 0){
        ROS_ERROR("Could not write header");
        ros::shutdown();
    }
    // allocate the frame
    if(!batch_processing_){
        frames.push_back(av_frame_alloc());
        frames.back()->width = output_codec_context->width;
        frames.back()->height = output_codec_context->height;
        frames.back()->format = output_codec_context->pix_fmt;
    }
    else{
        for(int i = 0; i < batch_period_ * fps; i++){
            frames.push_back(av_frame_alloc());
            frames.back()->width = output_codec_context->width;
            frames.back()->height = output_codec_context->height;
            frames.back()->format = output_codec_context->pix_fmt;
        }
    }

    int buffer_size = av_image_get_buffer_size(output_codec_context->pix_fmt, output_codec_context->width, output_codec_context->height, 1);
    output_buffer = static_cast<uint8_t*>(av_malloc(buffer_size*sizeof(uint8_t)));
        av_image_fill_arrays(frames.back()->data, frames.back()->linesize, output_buffer, AV_PIX_FMT_YUV420P, image_width_, image_height_, 1);

}