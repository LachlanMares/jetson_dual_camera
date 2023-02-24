#!/usr/bin/env python3

import time
import cv2
import numpy as np
import rospy
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


if __name__ == "__main__":

    rospy.init_node("video_creator")

    video_title = rospy.get_param(param_name="~video/title")
    frame_rate = rospy.get_param(param_name="~video/frame_rate")
    width = rospy.get_param(param_name="~video/width")
    height = rospy.get_param(param_name="~video/height")

    bag_timeout = False
    bridge = CvBridge()

    video_writer = cv2.VideoWriter("camera_0.avi", cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))

    data = rospy.wait_for_message("/stereo_front/left/image_raw", Image, timeout=30)

    while not rospy.is_shutdown():
        if not bag_timeout:
            data = rospy.wait_for_message("/stereo_front/left/image_raw", Image, timeout=1)
            # Use cv_bridge to convert ros image message
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

            try:
                video_writer.write(cv_image)
                last_image_time = time.time()
                cv2.waitKey(1)

            except:
                bag_timeout = True
                video_writer.release()
                rospy.loginfo("Image timeout")
        else:
            video_writer.release()
            rospy.loginfo("End of bag timer expired")
            rospy.signal_shutdown(" ")

    SystemExit(0)

