#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, String
import os
import queue
import threading

class MultiTopicSubscriber:
    def __init__(self):
        # Queue to manage speech messages
        self.speech_queue = queue.Queue()

        # Start a separate thread to process the speech queue
        self.speech_thread = threading.Thread(target=self.process_speech_queue)
        self.speech_thread.daemon = True  # Daemonize the thread to exit when the main program does
        self.speech_thread.start()

        # Subscribe to the '/aruco_detection' topic of type Float32MultiArray
        rospy.Subscriber('/aruco_detection', Float32MultiArray, self.aruco_callback)
        # Subscribe to the '/roi_detection' topic of type Float32MultiArray
        rospy.Subscriber('/roi_detection', Float32MultiArray, self.roi_callback)
        # Subscribe to the '/speak' topic of type String
        rospy.Subscriber('/speak', String, self.speak_callback)

        rospy.loginfo("Multi-Topic Subscriber Node Started")

        # Store last detected ArUco ID to avoid repeated announcements
        self.last_detected_aruco_id = None

    def aruco_callback(self, data):
        """Callback for the '/aruco_detection' topic."""
        # The first element in the data array is the marker ID, the rest are corner coordinates
        current_id = int(data.data[0])
        corners = data.data[1:]

        # Check if the detected ID is different from the last detected ID
        if current_id != self.last_detected_aruco_id:
            rospy.loginfo(f"New Aruco ID detected: {current_id}")
            rospy.loginfo(f"Corners: {corners}")  # Log the corner coordinates
            self.queue_message(f"ArUco Number {current_id} Identified")
            self.last_detected_aruco_id = current_id

    def roi_callback(self, data):
        """Callback for the '/roi_detection' topic."""
        # The first element in the data array is the ROI ID or number
        roi_id = int(data.data[0]*10)
        rospy.loginfo(f"ID: {roi_id}")

        if roi_id == 1:
            label = "Backpack"
        elif roi_id == 2:
            label = "Human"
        elif roi_id == 3:
            label = "Drone"
        elif roi_id == 4:
            label = "Phone"
        else:
            label = "Unknown"
            
        rospy.loginfo(f"New ROI ID detected: {label}")
        self.queue_message(f"{label} Identified")

    def speak_callback(self, data):
        """Callback for the '/speak' topic."""
        message = data.data
        rospy.loginfo(f"Received message to speak: {message}")
        self.queue_message(message)

    def queue_message(self, message):
        """Method to add a message to the speech queue."""
        self.speech_queue.put(message)
        rospy.loginfo(f"Queued message: {message}")

    def process_speech_queue(self):
        """Method to process messages in the speech queue one by one."""
        while not rospy.is_shutdown():
            message = self.speech_queue.get()  # Block until a message is available
            self.speak_message(message)
            self.speech_queue.task_done()  # Mark the message as processed

    def speak_message(self, message):
        """Method to use espeak to announce a message."""
        os.system(f'espeak "{message}"')
        rospy.loginfo(f"Spoken message: {message}")

def main():
    rospy.init_node('multi_topic_subscriber', anonymous=True)
    multi_topic_subscriber = MultiTopicSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
