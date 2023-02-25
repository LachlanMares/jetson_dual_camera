#!/usr/bin/env python3

import os
import cv2
import rospy
import subprocess
from pathlib import Path
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def bag_loop():
    global bag_filename, bag_directory
    with subprocess.Popen(['rosbag', 'play', str(bag_filename)], cwd=bag_directory) as bag_player:
        stdoutdata, stderrdata = bag_player.communicate()
        if bag_player.returncode == 0:
            waiting = False


if __name__ == "__main__":
    """ Don't ask me why video encoding has to happen in the main thread, it just does """
    rospy.init_node("video_creator")
    global bag_filename, bag_directory

    if rospy.get_param(param_name="~start_rosbag"):
        bag_directory = Path(rospy.get_param(param_name="~rosbag_directory"))
        bag_filename = bag_directory / rospy.get_param(param_name="~rosbag_filename")

        bag_loop_thread = Thread(target=bag_loop, daemon=True)

    jetson_dual_camera_dir = Path(os.getenv("HOME")) / 'jetson_dual_camera' / 'src' / 'jetson_dual_camera' / 'videos'

    video_title = jetson_dual_camera_dir / rospy.get_param(param_name="~video/title")
    frame_rate = rospy.get_param(param_name="~video/frame_rate")
    width = rospy.get_param(param_name="~video/width")
    height = rospy.get_param(param_name="~video/height")
    image_topic = rospy.get_param(param_name="~image_topic")

    bag_timeout = False
    first_frame = True
    bridge = CvBridge()
    video_writer = cv2.VideoWriter(str(video_title), cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), frame_rate, (width, height))

    if rospy.get_param(param_name="~start_rosbag"):
        bag_loop_thread.start()

    # Wait until the rosbag is started
    data = rospy.wait_for_message(image_topic, Image, timeout=30)

    while not rospy.is_shutdown():
        if not bag_timeout:
            try:
                if not first_frame:
                    data = rospy.wait_for_message(image_topic, Image, timeout=2)
                else:
                    first_frame = False

                # Use cv_bridge to convert ros image message
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                video_writer.write(cv_image)
                cv2.waitKey(1)

            except Exception as e:
                rospy.logerr(str(type(e)) + " " + str(e))
                bag_timeout = True

        else:
            video_writer.release()
            rospy.loginfo("End of Bag")
            rospy.signal_shutdown(" ")

    SystemExit(0)

