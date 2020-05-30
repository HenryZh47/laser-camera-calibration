#!/usr/bin/env python

import os
import argparse

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract one image from a ROS bag.")
    parser.add_argument("bag_dir", help="Input ROS bag directory.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print ("Extract images from %s on topic %s into %s" % (args.bag_dir,
                                                          args.image_topic, args.output_dir))

    # bag_dir = rosbag.Bag(args.bag_file, "r")
    bag_dir = args.bag_dir
    bridge = CvBridge()
    count = 1
    for bag_filename in sorted(os.listdir(bag_dir)):
        bag_path = os.path.join(bag_dir, bag_filename)
        bag = rosbag.Bag(bag_path, 'r')
        for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            cv2.imwrite(os.path.join(args.output_dir, "image%02i.jpg" % count), cv_img)
            print ("Wrote image %i" % count)
            break

        count += 1

        bag.close()

    return

if __name__ == '__main__':
    main()