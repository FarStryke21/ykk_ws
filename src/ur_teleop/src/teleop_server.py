#!/usr/bin/env python3

import rospy
import socket
import json
from sensor_msgs.msg import Joy

PORT = 55555

def process_data(data, joy_pub):
    try:
        joy_data = json.loads(data)
        joy_msg = Joy()
        joy_msg.axes = joy_data['axes']
        joy_msg.buttons = joy_data['buttons']
        joy_pub.publish(joy_msg)
        # rospy.loginfo(f"Published Joy message: axes={joy_msg.axes}, buttons={joy_msg.buttons}")
    except json.JSONDecodeError as e:
        rospy.logerr(f"JSON decode error: {e}")

def main():
    rospy.init_node('remote_joy_publisher')
    joy_pub = rospy.Publisher('/spacenav/joy', Joy, queue_size=10)

    hostname = 'cerlab42.lan.local.cmu.edu'
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((hostname, PORT))
        s.listen()
        rospy.loginfo(f"Listening for connections on {hostname}:{PORT}")
        
        while not rospy.is_shutdown():
            try:
                conn, addr = s.accept()
                with conn:
                    rospy.loginfo(f"Connected by {addr}")
                    buffer = ""
                    while not rospy.is_shutdown():
                        data = conn.recv(1024).decode()
                        if not data:
                            break
                        buffer += data
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            process_data(line, joy_pub)
            except Exception as e:
                rospy.logerr(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        passexit