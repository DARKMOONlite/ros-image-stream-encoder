#ifndef STREAM_ENCODER_NODELET_HPP
#define STREAM_ENCODER_NODELET_HPP
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "image_stream_encoder.hpp"
#include <pluginlib/class_list_macros.hpp>


namespace stream_encoder{

class StreamEncoderNodelet : public nodelet::Nodelet {
    public:
        StreamEncoderNodelet() = default;
        ~StreamEncoderNodelet() = default;

        virtual void onInit() override{
            image_stream_encoder_ = std::make_shared<ImageStreamEncoder>();
        }

    private:
        std::shared_ptr<ImageStreamEncoder> image_stream_encoder_;

};







} // namespace stream_encoder
PLUGINLIB_EXPORT_CLASS(stream_encoder::StreamEncoderNodelet, nodelet::Nodelet)
#endif // STREAM_ENCODER_NODELET_HPP
