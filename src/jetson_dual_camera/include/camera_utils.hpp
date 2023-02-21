/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    GPL-3.0

Description:

*/

#include <ros/ros.h>
#include <sstream>

std::string get_gstreamer_str(int camera_id, int camera_mode, int flip_method, int width, int height, int fps) {
    int _fps = 1;
    std::stringstream gs_str;

    switch(camera_mode) {     
        case 0: 
            _fps = (std::max(std::min(fps, 21), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";           
            break;

        case 1: 
            _fps = (std::max(std::min(fps, 28), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=1 ! video/x-raw(memory:NVMM), width=3264, height=1848, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
            break;

        case 2: 
            _fps = (std::max(std::min(fps, 30), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=2 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
            break;

        case 3: 
            _fps = (std::max(std::min(fps, 30), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=3 ! video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
            break;

        case 4: 
            _fps = (std::max(std::min(fps, 60), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=4 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
            break;

        case 5: 
            _fps = (std::max(std::min(fps, 120), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=4 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
            break;   
        
        default:
            _fps = (std::max(std::min(fps, 21), 1));
            gs_str << "nvarguscamerasrc sensor-id=" << camera_id << " sensor-mode=0 ! video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=" << _fps << "/1 ! nvvidconv flip-method=" << flip_method << " ! video/x-raw, width=" << width << ", height=" << height << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
            break;
    }

    return gs_str.str();
}

double get_timer_period(int fps, int camera_mode) {
    double timer_period = 1.0;

    switch(camera_mode) {  
        case 0: 
            timer_period = 1.0 / (std::max(std::min(fps, 21), 1));
            break;
        
        case 1:
            timer_period = 1.0 / (std::max(std::min(fps, 28), 1));
            break;
        
        case 2:
            timer_period = 1.0 / (std::max(std::min(fps, 30), 1));
            break;
        
        case 3:
            timer_period = 1.0 / (std::max(std::min(fps, 30), 1));
            break;
        
        case 4:
            timer_period = 1.0 / (std::max(std::min(fps, 60), 1));
            break;

        case 5:
            timer_period = 1.0 / (std::max(std::min(fps, 120), 1));
            break;

        default:
            timer_period = 1.0 / (std::max(std::min(fps, 21), 1));
            break;
    }

    return timer_period;
}
