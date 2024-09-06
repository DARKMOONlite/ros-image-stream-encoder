extern "C"{ // this links to the C library
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}
#include <string>
#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/Image.h>
#include <ros/timer.h>



class ImageStreamEncoder {
    public:


    ImageStreamEncoder();
    ~ImageStreamEncoder();

    void image_and_encode(sensor_msgs::Image::ConstPtr image_msg);
    void image_callback(sensor_msgs::Image::ConstPtr image_msg);
    void encode_image();
    bool ffmpeg_init();



    private:
    bool batch_processing_ = false;
    double batch_period_ = 2.0;
    int image_width_ = 0, image_height_ = 0; // this should be able to be set by the user or gotten by the image stream

    ros::NodeHandle nh;
    std::unique_ptr<ros::Subscriber> image_stream;
    // std::vector<std::unique_ptr<ros::Subscriber>> image_streams;
    std::string output_prepend, video_format, full_filename;



    AVFormatContext *output_format_context = nullptr;
    AVStream *output_video_stream = nullptr;

    AVCodec *output_codec = nullptr;
    AVCodecContext *output_codec_context = nullptr;
    AVCodecParameters *output_codec_parameters = nullptr;
    u_int8_t *output_buffer = nullptr;

    ros::Timer timer; // used for atch processing, not activated if batch processing is not enabled

    std::vector<AVFrame *> frames;

};

