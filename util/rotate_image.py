#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image


class Rotator(object):
    def __init__(self, topic):
        self._bridge = cv_bridge.CvBridge()
        self._image_sub = rospy.Subscriber(
            topic, Image, self._rotate_image, queue_size=10, buff_size=1 << 24
        )
        self._image_pub = rospy.Publisher(
            topic + "_rotated", Image, queue_size=10
        )

    def _rotate_image(self, msg):
        #  encoding = "mono16" if msg.encoding == "16UC1" else msg.encoding
        encoding = msg.encoding
        cv_image = self._bridge.imgmsg_to_cv2(msg, encoding)

        if encoding == "16UC1":
            cv_image_rotated = cv_image[::-1, ::-1]
        else:
            cv_image_rotated = cv_image[::-1, ::-1, :]

        img_msg = self._bridge.cv2_to_imgmsg(cv_image_rotated, encoding)
        img_msg.header = msg.header  # Copy header over, mainly for timestamp.

        self._image_pub.publish(img_msg)


def main():

    rospy.init_node("rotator")
    rotators = []
    for topic in rospy.get_param("~topics"):
        rotators.append(Rotator(topic))

    rospy.spin()


if __name__ == "__main__":
    main()
