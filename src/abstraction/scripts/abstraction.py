#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from threading import Lock
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class abstraction:
    def __init__(self):
        self.image = None
        self.imageLock = Lock()
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.callback_redraw)
        
        # ---- Subscribers ----
        self.chesscamsubscriber = rospy.Subscriber('/chesscam/compressed', Image, self.callback_chesscam)
        rospy.loginfo('subscribed to /chesscam/compressed')

        # ---- Publishers ----
        #self.chesscam_pub = rospy.Publisher('chessimage', Image, queue_size=10)
        #rospy.loginfo('created a publisher for topic chessimage')

    # ---- Callbacks ----
    def callback_redraw(self, event):
        if self.image is not None:
            self.imageLock.acquire()
            try:
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()
            image_cv = cv2.resize(image_cv, dsize=(800, 550), interpolation=cv2.INTER_CUBIC)
            rospy.loginfo("publishing image to gui")
            self.chesscam_pub.publish(image_cv)

            cv2.imshow("image", image_cv)
            key = cv2.waitKey(5)
                if key == 27:
                    cv2.destroyAllWindows()   
    
    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
            except CvBridgeError as error:
                raise Exception("Failed to convert to OpenCV image")

    def callback_chesscam(self, msg):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

if __name__ == '__main__':
    rospy.init_node('abstraction')
    rospy.loginfo('abstraction node has been initialized')
    abstraction()
    rospy.spin()