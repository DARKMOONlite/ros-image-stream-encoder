#include "image_stream_encoder.hpp"
#include <iomanip>
#include <ctime>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <bits/stdc++.h>
ImageStreamEncoder::ImageStreamEncoder() {

    nh = ros::NodeHandle("~");
    rolling_timestamps.resize(10, 0);
    // ------------------------ FILE NAME ------------------------

    std::stringstream output_file;
    { // scope block to limit the scope of the time variables
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        output_file << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S"); 
    }
    if(!nh.getParam("video_format", video_format)){
        ROS_WARN("No output_file parameter found, defaulting format to .mp4");
        video_format = ".mp4";
    }
    if(!nh.getParam("output_prepend", output_prepend)){
        ROS_INFO("No output_prepend, file name defaulting to %s", output_file.str().c_str());
        
    }
    ROS_INFO("test2");
    std::stringstream temp;
    temp << output_prepend << output_file.str() << video_format;
    full_filename = temp.str();
    ROS_INFO("Output file: %s", full_filename.c_str());
    // ------------------------ BATCH PROCESSING ------------------------

    if(!nh.getParam("batch", batch_processing_)){
        ROS_WARN("No batch parameter found, defaulting to false");
        batch_processing_ = false;
    }

    if(!nh.getParam("batch_period", batch_period_)){
        ROS_INFO("No batch_period parameter found, defaulting to 2 seconds");
        batch_period_ = 2;
    }
    if(!nh.getParam("input_stream", input_stream_string_)){
        ROS_WARN("No input_stream parameter found, defaulting to image_stream");
        input_stream_string_ = "/image_stream";
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
        image_stream = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::Image>(input_stream_string_, 10, [this](sensor_msgs::Image::ConstPtr image_msg){
            this->image_and_encode(image_msg);
        }));
    }

    


    // ------------------------ AV LIBRARIES ------------------------
    ffmpeg_wrapper_ = std::make_shared<FFMPEGWrapper>(full_filename, AV_CODEC_ID_MPEG4);

}

ImageStreamEncoder::~ImageStreamEncoder() {

}

void ImageStreamEncoder::image_and_encode(sensor_msgs::Image::ConstPtr image_msg) {
    // Callback function for image stream
    image_callback(image_msg);
    encode_image();
}

void ImageStreamEncoder::image_callback(sensor_msgs::Image::ConstPtr image_msg) {
    // Merge sec and nsec into a single value
    double fps = calculate_fps(image_msg->header.stamp);
    if(frame_count < rolling_timestamps.size()){ // this is to prevent the fps from being calculated before the rolling_timestamps vector is full
        return;
    }

    ROS_INFO("Image Callback");
    if(!ffmpeg_initialized_){
        if( ! (ffmpeg_initialized_ = ffmpeg_wrapper_->ffmpeg_init(image_msg->width, image_msg->height, static_cast<int>(fps))) ){
            ROS_ERROR("FFMPEG initialization failed");
            return;
        }
    }
    ffmpeg_wrapper_->encodeFrame(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image);
}
/**
 * @brief this will be on a timer and happen every couple of seconds
 * 
 */
void ImageStreamEncoder::encode_image() {// sensor_msgs::Image::ConstPtr image_msg
    ROS_INFO("Encode Image");
    // Encode image stream

}


/**
 * @brief 
 * 
 * @param header_stamp timestamp from the message header
 * @return double when this has only been run once, it will return -1
 */
double ImageStreamEncoder::calculate_fps(ros::Time header_stamp) {

    double timestamp = header_stamp.sec + (header_stamp.nsec / 1e9);
    rolling_timestamps.at(frame_count % rolling_timestamps.size()) = timestamp;
    frame_count++;
    double period = (*max_element(rolling_timestamps.begin(), rolling_timestamps.end()) - *min_element(rolling_timestamps.begin(), rolling_timestamps.end()))/rolling_timestamps.size();
    if(period != 0){
        ROS_INFO("fps: %f", 1 / period);
        return(1 / period);
    }
    else{
        return(-1);
    }
    
}



int main(int argc, char** argv){
    ros::init(argc, argv, "image_stream_encoder");
    ImageStreamEncoder ise;
    ros::spin();
}

