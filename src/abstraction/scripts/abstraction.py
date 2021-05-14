#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from threading import Lock
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv_utils

class abstraction:
    def __init__(self):
        self.image = None
        self.imageLock = Lock()
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.GUI_UPDATE_PERIOD = 0.7
        self.redrawTimer = rospy.Timer(rospy.Duration(self.GUI_UPDATE_PERIOD), self.callback_redraw)
        self.corners = None
        self.previous_images = []
        self.max_length = 10

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

            image_cv = cv2.resize(image_cv, (1069, 599))

            corners = cv_utils.calculate_corners(image_cv)
            if corners is not None:
                self.corners = corners
            # img = cv_utils.get_squares(corners, image_cv)
            #self.move_piece(image_cv)
            #img = cv_utils.detect_and_draw_circles(image_cv)
            img = image_cv
            if self.corners is not None:
                img = cv_utils.get_squares(self.corners,image_cv)

            # print(arr)
            # img = cv_utils.calculate_corners(image_cv)
            cv2.imshow("image", img)
            key = cv2.waitKey(5)
            if key == 27:
                cv2.destroyAllWindows()

    def move_piece(self, current_img):
        current_img = cv2.cvtColor(current_img, cv2.COLOR_BGR2GRAY)
        if len(self.previous_images) > 0:
            total_difference = np.zeros_like(current_img)
            for img in self.previous_images:
                difference = cv2.subtract(img, current_img)
                total_difference = cv2.addWeighted(total_difference, 1, difference, 1, 0.0)
            cv2.imshow('difference', total_difference)
            key = cv2.waitKey(5)
            if key == 27:
                cv2.destroyAllWindows
        self.previous_images.append(current_img)
        if len(self.previous_images) > self.max_length:
            self.previous_images.pop(0)


    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def callback_chesscam(self, data):
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
