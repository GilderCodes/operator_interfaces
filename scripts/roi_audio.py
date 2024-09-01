#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import os

class ROISubscriber:
    def __init__(self):
        # Subscribe to the '/roi_detection' topic of type Float32MultiArray
        rospy.Subscriber('/roi_detection', Float32MultiArray, self.roi_callback)
        rospy.loginfo("ROI Subscriber Node Started")

    def roi_callback(self, data):
        # The first element in the data array is the ROI ID or number
        roi_id = int(data.data[0])
        rospy.loginfo(f"New ROI ID detected: {roi_id}")
        self.speak_roi_id(roi_id)

    def speak_roi_id(self, roi_id):
        # Use espeak to announce the ROI number
        os.system(f'espeak "ROI Number {roi_id} Identified"')
        rospy.loginfo(f"Announced ROI ID: {roi_id}")

def main():
    rospy.init_node('roi_subscriber', anonymous=True)
    roi_subscriber = ROISubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
