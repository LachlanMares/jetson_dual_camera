/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    GPL-3.0

Description:

*/

#include "ros/ros.h"
#include <camera_utils.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <sstream>
#include <new>
#include <string>


class RPi2Camera
{
    private:
        // Node handles
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        image_transport::ImageTransport _it;
        
        // Variables
        bool _running;
        int _device_id;
        int _camera_mode;
        int _flip_method;
        int _frame_rate;
        int _image_width;
        int _image_height;

        std::string _image_topic;
        // std::string _gstreamer_str;

        // Publishers
        image_transport::Publisher _image_publisher;

        // CV2
        cv::VideoCapture _video;
        cv::Mat _frame;

        // Timers
        ros::Timer _timer;

    public:
    
        RPi2Camera(): _it(_nh), _nh(), _nh_priv("~") {

            // Private Parameters
            _nh_priv.param<int>("device_id", _device_id, 0);
            _nh_priv.param<int>("camera_mode", _camera_mode, 0);
            _nh_priv.param<int>("flip_method", _flip_method, 0);
            _nh_priv.param<int>("frame_rate", _frame_rate, 21);
            _nh_priv.param<int>("image_width", _image_width, 3280);
            _nh_priv.param<int>("image_height", _image_height, 2464);

            std::ostringstream _image_topic_default;
            _image_topic_default << "/camera_" << _device_id;
            _nh_priv.param<std::string>("image_topic", _image_topic, _image_topic_default.str());

            // Variables
            _running = false;

            //std::string g_string = get_gstreamer_str(_device_id, _camera_mode, _flip_method, _image_width, _image_height);

            //std::stringstream ss;
            //ss << "nvarguscamerasrc sensor-id=" << _device_id << " sensor-mode=" << _camera_mode << " ! video/x-raw(memory:NVMM), width=" << camera_capture_width[_camera_mode] << ", height=" << camera_capture_height[_camera_mode] << ", format=NV12, framerate=" << camera_capture_fps[_camera_mode] << "/1 ! nvvidconv flip-method=" << _flip_method << " ! video/x-raw, width=" << _image_width << ", height=" << _image_height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
 
            _video.open(get_gstreamer_str(_device_id, _camera_mode, _flip_method, _image_width, _image_height));

            if (!_video.isOpened()) {
                ROS_ERROR("Unable to get video from the camera!");
            }

            _image_publisher = _it.advertise(_image_topic, 1);

            // Timers
            _timer = _nh.createTimer(ros::Duration(get_timer_period(_frame_rate, _camera_mode)), &RPi2Camera::timerCallback, this);
        }

        ~RPi2Camera() {}

        void start_capture() {
            _running = true;
        }

        void stop_capture() {
            _running = false;
        }

        void timerCallback(const ros::TimerEvent &) { 
            if (_running && ros::ok()) {
                if (_video.read(_frame)) { 
                    cv_bridge::CvImage camera_msg; 
                    camera_msg.header.stamp = ros::Time::now();
                    camera_msg.encoding = sensor_msgs::image_encodings::BGR8;
                    camera_msg.image = _frame; 
                    _image_publisher.publish(camera_msg.toImageMsg());
                } 
            }           
        }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_node");
  
  RPi2Camera rp_cam;

  rp_cam.start_capture();

  ros::spin();

  return 0;
}