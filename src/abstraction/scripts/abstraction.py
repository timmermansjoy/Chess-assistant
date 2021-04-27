#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

class abstraction:
    def __init__(self):
        # ---- Subscribers ----
        self.chesscamsubscriber = rospy.Subscriber('/chesscam/compressed', Image, self.publishChesscam)
        rospy.loginfo('subscribed to /chesscam/compressed')

    def publishChesscam(self, msg):
        pass

if __name__ == '__main__':
    rospy.init_node('abstraction')
    rospy.loginfo('abstraction node has been initialized')
    abstraction()
    rospy.spin()