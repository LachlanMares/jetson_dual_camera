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
#include <signal.h>

class RPi2Camera
{
    private:
        // Node handles
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        image_transport::ImageTransport _it;
        
        // Variables
        bool _running;
        int _camera_id;
        int _camera_mode;
        int _flip_method;
        int _frame_rate;
        int _image_width;
        int _image_height;
        std::string _image_topic;

        // Publishers
        image_transport::Publisher _image_publisher;

        // CV2
        cv::VideoCapture _video;

        // Timers
        ros::Timer _timer;

    public:
    
        RPi2Camera(): _it(_nh), _nh(), _nh_priv("~") {

            // Private Parameters
            _nh_priv.param<int>("camera/id", _camera_id, 0);
            _nh_priv.param<int>("camera/mode", _camera_mode, 0);
            _nh_priv.param<int>("camera/flip_method", _flip_method, 0);
            _nh_priv.param<int>("video/frame_rate", _frame_rate, 30);
            _nh_priv.param<int>("image/width", _image_width, 1280);
            _nh_priv.param<int>("image/height", _image_height, 720);
            _nh_priv.param<std::string>("image/topic", _image_topic, "/camera_0");

            // Variables
            _running = false;

            // Start up the camera
            _video.open(get_gstreamer_str(_camera_id, _camera_mode, _flip_method, _image_width, _image_height));

            if (!_video.isOpened()) {
                ROS_ERROR("Unable to get video from the camera!");
            }

            // Publishers
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
            if (ros::ok()) {
                if (_running) {
                    cv_bridge::CvImage camera_msg;

                    if (_video.read(camera_msg.image)) {            
                        camera_msg.encoding = sensor_msgs::image_encodings::BGR8;
                        camera_msg.header.stamp = ros::Time::now(); 
                        _image_publisher.publish(camera_msg.toImageMsg());
                    } 
                }
            } else {
                kill(getpid(), SIGINT);
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