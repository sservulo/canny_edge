#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CannyEdgeDetector:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback)
        self.publisher = rospy.Publisher("/canny_edge", Image, queue_size = 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        low_threshold = 40
        ratio = 3
        kernel_size = 3
        blur = cv2.blur(image, (kernel_size, kernel_size))
        mask = cv2.Canny(blur, low_threshold, low_threshold*ratio, kernel_size)
        canny_edge = cv2.bitwise_and(image, image, mask = mask)

        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(canny_edge, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node("canny_edge_detector")
    CannyEdgeDetector()
    rospy.spin()