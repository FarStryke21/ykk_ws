#!/usr/bin/env python3

import rospy
import zmq
import cv2

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CLIENT_ADDRESS = "tcp://*:5556"

def flip_image(frame):
    return cv2.flip(cv2.flip(frame, 0), 1)  # 0 means flipping around the x-axis (vertical flip)
    # return cv2.flip(frame, 1)

def main():
    rospy.init_node('camera_stream_publisher')
    
    # ZMQ setup
    context = zmq.Context()
    socket_send = context.socket(zmq.PUB)
    socket_send.bind(CLIENT_ADDRESS)
    
    # ROS publisher setup
    ros_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    
    bridge = CvBridge()
    
    rospy.loginfo(f"Sending camera data on ZMQ: {CLIENT_ADDRESS}")
    rospy.loginfo("Publishing camera data on ROS topic: /camera/image_raw")
    
    # Setup camera capture (adjust parameters as needed)
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    rate = rospy.Rate(30)  # Adjust rate as needed
    
    while not rospy.is_shutdown():
        try:
            # Capture frame
            ret, frame = camera.read()
            if ret:
                # Flip the image vertically
                frame = flip_image(frame)
                
                # Send over ZMQ
                _, buffer = cv2.imencode('.jpg', frame)
                jpg_as_text = buffer.tobytes()
                socket_send.send(jpg_as_text)
                
                # Publish to ROS topic
                ros_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                ros_msg.header.stamp = rospy.Time.now()
                ros_pub.publish(ros_msg)
                
            rate.sleep()
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

    camera.release()
    socket_send.close()
    context.term()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
