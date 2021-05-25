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
        self.max_length = 3
        self.got_moves = False

        # ---- Subscribers ----
        self.chesscamsubscriber = rospy.Subscriber('/chesscam/compressed', Image, self.callback_chesscam)
        rospy.loginfo('subscribed to /chesscam/compressed')

        # ---- Publishers ----
        #self.chesscam_pub = rospy.Publisher('chessimage', Image, queue_size=10)
        #rospy.loginfo('created a publisher for topic chessimage')

        self.move_pub = rospy.Publisher('visionMove', String, queue_size=10)
        rospy.loginfo('created a publisher for topic visionMove')

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
            # if corners is not None:
            
            if corners is not None:
                corners, img = cv_utils.get_squares(corners,image_cv)
                self.corners = corners
                gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
                if len(self.previous_images) > 0:
                    x1, y1, x2, y2 = cv_utils.get_move(self.corners, gray, self.previous_images)
                    print(f'{x1},{y1} -- {x2},{y2}')
                    #print(self.got_moves)
                    if not self.got_moves and x2 != None:
                        self.got_moves = True
                        self.move = (x1, y1, x2, y2)
                        rospy.loginfo('advertising to topic visionMove with value ' + str(self.move))
                        self.move_pub.publish(str(self.move))
                        print(self.move)
                    elif x2 == None:
                        self.got_moves = False 
                self.previous_images.append(gray)
                if len(self.previous_images) > self.max_length:
                    self.previous_images.pop(0)
                img = cv2.resize(img, (900,900))
                cv2.imshow("image", img)
                key = cv2.waitKey(5)
                if key == 27:
                    cv2.destroyAllWindows()


    def move_piece(self, current_img):
        current_img = cv2.cvtColor(current_img, cv2.COLOR_BGR2GRAY)
        if len(self.previous_images) > 0:
            diff = cv_utils.get_move(current_img, self.previous_images)
            cv2.imshow('difference', diff)
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
