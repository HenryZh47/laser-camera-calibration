#!/usr/bin/env python

# script for saving flir thermal images from topic /thermal

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import glob
import sys
import getopt

RECORD_INTERVAL = 2
BAG_DIR = ''
IMG_DIR = ''

bridge = CvBridge()
image_index = 0
prev_time = 0

def image_callback(msg):
    # rospy.loginfo('recieved frame')
    global image_index
    global prev_time
    now = rospy.get_time()
    if (now - prev_time > RECORD_INTERVAL):
        prev_time = now
        rospy.loginfo('save the frame')
        try:
            # convert sensor message to openCV image
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except (CvBridgeError):
            print("cv bridge error")
        else:
            # save the image to the following path
            image_path = "image%d.jpg" %image_index
            image_path = os.path.join(IMG_DIR, image_path)
            cv2.imwrite(image_path, cv2_img)

            # save image every second
            # rospy.sleep(1)
            # increment image index
            image_index += 1


def delete_files(img_dir):
    for f in os.listdir(img_dir):
        path = os.path.join(img_dir, f)
        os.remove(path)
    print('previous images deleted!')


def setup_dir(img_dir):
    if os.path.exists(img_dir):
        delete_files(img_dir)
    else:
        os.mkdir(img_dir)

def get_param():
    global RECORD_INTERVAL
    global BAG_DIR
    global IMG_DIR
    
    argv = sys.argv[1:]
    if (len(argv) < 2):
        print("usage: flir_save_images.py -b <bag_dir> --o <image_dest_dir> --t <time_interval>")
        sys.exit(2)
    try:
        opts, args = getopt.getopt(argv,"hb:",["o=", "t="])
    except getopt.GetoptError:
        print("usage: flir_save_images.py -b <bag_dir> --o <image_dest_dir> --t <time_interval>")
        sys.exit(2)
    for o, a in opts:
        if o == "-h":
            print("usage: flir_save_images.py -b <bag_dir> --o <image_dest_dir> --t <time_interval>")
            sys.exit()
        elif o == "-b":
            BAG_DIR = a
            if IMG_DIR == '':
                IMG_DIR = BAG_DIR + "-img"   
        elif o == "--o":
            IMG_DIR = a
        elif o == "--t":
            RECORD_INTERVAL = int(a)


def main():
    global prev_time
    global IMG_DIR
    global BAG_DIR

    # get parameters from command line
    get_param()
    print("bag dir is %s" %BAG_DIR)
    print("img dir is %s" %IMG_DIR)
    print("record time interval is %d seconds" %RECORD_INTERVAL)

    # delete files in the target image dir
    setup_dir(IMG_DIR)

    flir_topic = "/thermal"
    rospy.init_node('flir_image_listener', log_level=rospy.INFO)
    prev_time = rospy.get_time()

    rospy.Subscriber(flir_topic, Image, image_callback, queue_size=1, buff_size=1440*1024)
    rospy.spin()

if __name__ == "__main__":
    main()
