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

    // if(!nh.getParam("batch", batch_processing_)){
    //     ROS_WARN("No batch parameter found, defaulting to false");
    //     batch_processing_ = false;
    // }

    if(!nh.getParam("batch_period", batch_period_)){
        ROS_INFO("No batch_period parameter found, defaulting to 2 seconds");
        batch_period_ = 2;
    }
    if(!nh.getParam("input_stream", input_stream_string_)){
        ROS_WARN("No input_stream parameter found, defaulting to image_stream");
        input_stream_string_ = "/image_stream";
    }
    if(!nh.getParam("batch_size", batch_size_)){
        ROS_INFO("No batch_size parameter found, defaulting to 0");
        batch_size_ = 0;
    }
    if(batch_size_ > 0){
        batch_processing_ = true;
        ROS_INFO("Batch processing enabled");
    }


    // ------------------------- SUBSCRIBER -------------------------

    if(batch_processing_){
        image_stream = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::Image>("image_stream", 10, [this](sensor_msgs::Image::ConstPtr image_msg){
            this->image_callback(image_msg);
        }));

        timer = nh.createTimer(ros::Duration(batch_period_), [this](const ros::TimerEvent& event){
            this->timer_callback(event);
        });


    } else {
        ROS_INFO("Batch processing disabled");
        image_stream = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::Image>(input_stream_string_, 10, [this](sensor_msgs::Image::ConstPtr image_msg){
            this->image_and_encode_callback(image_msg);
        }));
    }

    


    // ------------------------ AV LIBRARIES ------------------------
    ffmpeg_wrapper_ = std::make_shared<FFMPEGWrapper>(full_filename, AV_CODEC_ID_MPEG4);

}

ImageStreamEncoder::~ImageStreamEncoder() {

}

void ImageStreamEncoder::image_and_encode_callback(sensor_msgs::Image::ConstPtr image_msg) {
    if(!image_callback(image_msg)){return;}
    if(ffmpeg_initialized_){
        encode_image(image_msg);
    }
}

bool ImageStreamEncoder::image_callback(sensor_msgs::Image::ConstPtr image_msg) {
    double fps = calculate_fps(image_msg->header.stamp); //TODO find a way to allow dynamic fps changes to impact the encoding
    if(fps<0){
        return(false);
    }

    if(!ffmpeg_initialized_){
        if( ! (ffmpeg_initialized_ = ffmpeg_wrapper_->ffmpeg_init(image_msg->width, image_msg->height, static_cast<int>(fps))) ){
            ROS_ERROR("FFMPEG initialization failed");
            return(false);
        }
    }
    return(true);

    if(batch_processing_){image_buffer.push_back(image_msg);}

}

void ImageStreamEncoder::timer_callback(const ros::TimerEvent& event) {
    ROS_INFO("Timer Callback");
    if(image_buffer.size() > 0){
        encode_image(image_buffer);
        image_buffer.clear();
    }
    else{
        ROS_INFO("No images in buffer");
    }
}


void ImageStreamEncoder::encode_image(sensor_msgs::Image::ConstPtr image_msg) {// sensor_msgs::Image::ConstPtr image_msg
    ROS_INFO("Encode Image");
    // Encode image stream
    ffmpeg_wrapper_->encodeFrame(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image);
}
void ImageStreamEncoder::encode_image(std::vector<sensor_msgs::Image::ConstPtr> image_msgs){
    ROS_INFO("Encode %i Batched Images", image_msgs.size());
    for(auto image_msg : image_msgs){
        ffmpeg_wrapper_->encodeFrame(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image);
    }
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

    if(frame_count < rolling_timestamps.size()){
        return(-1);
    }
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

