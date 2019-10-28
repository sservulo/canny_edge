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
        
        canny_image = cv2.Canny(image,100,200)

        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(canny_image, "mono8"))
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node("canny_edge_detector")
    CannyEdgeDetector()
    rospy.spin()