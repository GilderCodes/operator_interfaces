#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import os

class ArucoSubscriber:
    def __init__(self):
        self.last_detected_id = None
        rospy.Subscriber('/aruco_id', Int32, self.aruco_callback)
        rospy.loginfo("Aruco Subscriber Node Started")

    def aruco_callback(self, data):
        current_id = data.data

        # Check if the detected ID is different from the last detected ID
        if current_id != self.last_detected_id:
            rospy.loginfo(f"New Aruco ID detected: {current_id}")
            self.speak_aruco_id(current_id)
            self.last_detected_id = current_id

    def speak_aruco_id(self, aruco_id):
        # Use espeak to announce the ArUco number
        os.system(f'espeak "ArUco Number {aruco_id} Identified"')
        rospy.loginfo(f"Announced Aruco ID: {aruco_id}")

def main():
    rospy.init_node('aruco_subscriber', anonymous=True)
    aruco_subscriber = ArucoSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
