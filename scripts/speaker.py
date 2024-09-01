#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os

class SpeakSubscriber:
    def __init__(self):
        # Subscribe to the '/speak' topic of type String
        rospy.Subscriber('/speak', String, self.speak_callback)
        rospy.loginfo("Speak Subscriber Node Started")

    def speak_callback(self, data):
        message = data.data
        rospy.loginfo(f"Received message to speak: {message}")
        self.speak_message(message)

    def speak_message(self, message):
        # Use espeak to announce the message
        os.system(f'espeak "{message}"')
        rospy.loginfo(f"Spoken message: {message}")

def main():
    rospy.init_node('speak_subscriber', anonymous=True)
    speak_subscriber = SpeakSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
