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
        self.previous_placement = None
        self.frameCount = 0
        self.frameVal = 10
        self.white_cascade = cv2.CascadeClassifier('haarcascade_white.xml')
        self.black_cascade = cv2.CascadeClassifier('haarcascade_black.xml')
        self.black_cascade2 = cv2.CascadeClassifier('haarcascade_black2.xml')
        self.chess_cascade = cv2.CascadeClassifier('haarcascade_chesspiece.xml')
        self.placement = [[0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0],
                          [None, None, None, None, None, None, None, None],
                          [None, None, None, None, None, None, None, None],
                          [None, None, None, None, None, None, None, None],
                          [None, None, None, None, None, None, None, None],
                          [1, 1, 1, 1, 1, 1, 1, 1],
                          [1, 1, 1, 1, 1, 1, 1, 1]]
        self.ignore = [(3, 5)]

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
            if self.corners is None:
                corners = cv_utils.calculate_corners(image_cv)
                if corners is not None:
                    self.corners = corners

            # if corners is not None:
            corners = cv_utils.calculate_corners(image_cv)
            if corners is not None:
                corners, image = cv_utils.get_squares(self.corners, image_cv)
                #self.corners = corners
                #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                row_start = col_start = row_end = col_end = None
                placement, begin, end = cv_utils.find_move(self.white_cascade, self.black_cascade, self.chess_cascade, self.placement, corners, image_cv)
                self.placement = placement
                for b in begin:
                    if b in self.ignore:
                        begin.remove(b)
                for e in end:
                    if e in self.ignore:
                        end.remove(e)
                if len(begin) > 0 and len(end) == 0:
                    self.ignore.append(begin)
                elif len(begin) == 0 and len(end) > 0:
                    self.ignore.append(end)
                elif len(begin) > 0 and len(end) > 0:
                    if len(begin) == 1:
                        row_start, col_start = begin[0]
                    else:
                        row_start = begin
                        col_start = ''
                    if len(end) == 1:
                        row_end, col_end = end[0]
                    else:
                        row_end = end
                        col_end = ''
                print(f'({row_start},{col_start}) -- ({row_end},{col_end})')
                self.move_pub.publish(f'({row_start},{col_start})--({row_end},{col_end})')
                # print(f'{x1},{y1} -- {x2},{y2}')
                # print(self.got_moves)
                # self.previous_placement = placement
                # if not self.got_moves and x2 != None:
                #     self.got_moves = True
                #     self.move = (x1, y1, x2, y2)
                # elif self.got_moves and x2 != None:
                #      self.move = (x1, y1, x2, y2)
                # elif self.got_moves and x2 == None:
                #      self.got_moves = False
                #      print(self.move)
                # self.previous_images.append(img)
                # if len(self.previous_images) > self.max_length:
                #     self.previous_images.pop(0)
                # image = cv2.resize(image, (900,900))
                # cv2.imshow("image", image)
                # key = cv2.waitKey(5)
                # if key == 27:
                #     cv2.destroyAllWindows()

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
