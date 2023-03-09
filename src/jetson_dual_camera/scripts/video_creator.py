#!/usr/bin/env python3

import os
import cv2
import rospy
import subprocess
import signal
import queue
import time
from pathlib import Path

from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image_queue = queue.Queue(maxsize=3)


def image_callback(data):
    global bag_timeout, height, width
    # Use cv_bridge to convert ros image message
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    if cv_image.shape[0] != height or cv_image.shape[1] != width:
        rospy.logerr(f'Image shape incorrect, killing node \n')
        bag_timeout = True

    image_queue.put(cv_image, block=False)


def bag_loop():
    global bag_filename, bag_directory, bag_timeout
    with subprocess.Popen(['rosbag', 'play', str(bag_filename)], cwd=bag_directory) as bag_player:
        stdoutdata, stderrdata = bag_player.communicate()

        if bag_player.returncode == 0:
            bag_timeout = True
            bag_player.terminate()


if __name__ == "__main__":
    """ Don't ask me why video encoding has to happen in the main thread, it just does """
    rospy.init_node("video_creator")
    global bag_filename, bag_directory, bag_running

    jetson_dual_camera_dir = Path(os.getenv("HOME")) / 'jetson_dual_camera' / 'videos'
    print(jetson_dual_camera_dir)

    video_title = jetson_dual_camera_dir / rospy.get_param(param_name="~video/title")
    frame_rate = rospy.get_param(param_name="~video/frame_rate")
    width = rospy.get_param(param_name="~image/width")
    height = rospy.get_param(param_name="~image/height")
    image_topic = rospy.get_param(param_name="~image/topic")

    bag_timeout = False
    bridge = CvBridge()
    video_writer = cv2.VideoWriter(str(video_title), cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), frame_rate, (width, height))

    # SUBSCRIBERS
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)

    if rospy.get_param(param_name="~rosbag/start"):
        bag_directory = Path(rospy.get_param(param_name="~rosbag/directory"))
        bag_filename = bag_directory / rospy.get_param(param_name="~rosbag/filename")
        bag_loop_thread = Thread(target=bag_loop, daemon=True)
        bag_loop_thread.start()

    while not rospy.is_shutdown():
        while not bag_timeout:
            try:
                queue_data = image_queue.get(timeout=2.0)
                video_writer.write(queue_data)

            except Exception as e:
                rospy.loginfo(str(type(e)) + " " + str(e))
                bag_timeout = True

        else:
            video_writer.release()
            rospy.loginfo("End of Bag")
            os.killpg(os.getpid(), signal.SIGTERM)

    SystemExit(0)

