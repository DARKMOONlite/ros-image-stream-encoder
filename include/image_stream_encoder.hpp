
#include <string>
#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/Image.h>
#include <ros/timer.h>
#include <ffmpeg_wrapper.hpp>


class ImageStreamEncoder {
    public:


    ImageStreamEncoder();
    ~ImageStreamEncoder();

    void image_and_encode(sensor_msgs::Image::ConstPtr image_msg);
    void image_callback(sensor_msgs::Image::ConstPtr image_msg);
    void encode_image(); //sensor_msgs::Image::ConstPtr image_msg
    bool ffmpeg_init();
    double calculate_fps(ros::Time timestamp);



    private:
    bool batch_processing_ = false, ffmpeg_initialized_ = false;
    double batch_period_ = 2.0;
    int image_width_ = 0, image_height_ = 0; // this should be able to be set by the user or gotten by the image stream

    ros::NodeHandle nh;
    std::unique_ptr<ros::Subscriber> image_stream;
    // std::vector<std::unique_ptr<ros::Subscriber>> image_streams;
    std::string output_prepend, video_format, full_filename, input_stream_string_;

    uint32_t frame_count = 0;
    std::vector<double> rolling_timestamps;

    ros::Timer timer; // used for atch processing, not activated if batch processing is not enabled

    std::shared_ptr<FFMPEGWrapper> ffmpeg_wrapper_;

};

