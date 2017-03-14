#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from cv2 import imread
from sensor_msgs.msg import Image


class ImageLatchedPublisher:

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._filename = rospy.get_param('~file', 'image.jpg')
        self._pub = rospy.Publisher('image', Image,
                                    latch=True, queue_size=1)

    def spin(self):
        img = imread(self._filename)
        msg = self._cv_bridge.cv2_to_imgmsg(img, 'bgr8')
        self._pub.publish(msg)
        rospy.spin()


rospy.init_node('image_latch_publisher', anonymous=True)
ilp = ImageLatchedPublisher()
ilp.spin()

