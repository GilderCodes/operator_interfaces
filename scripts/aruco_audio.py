#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import os

class ArucoSubscriber:
    def __init__(self):
        self.last_detected_id = None
        # Subscribe to the '/aruco_detection' topic of type Float32MultiArray
        rospy.Subscriber('/aruco_detection', Float32MultiArray, self.aruco_callback)
        rospy.loginfo("Aruco Subscriber Node Started")

    def aruco_callback(self, data):
        # The first element in the data array is the marker ID, the rest are corner coordinates
        current_id = int(data.data[0])
        corners = data.data[1:]

        # Check if the detected ID is different from the last detected ID
        if current_id != self.last_detected_id:
            rospy.loginfo(f"New Aruco ID detected: {current_id}")
            rospy.loginfo(f"Corners: {corners}")  # Log the corner coordinates
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
