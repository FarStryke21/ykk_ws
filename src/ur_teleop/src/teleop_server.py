#!/usr/bin/env python3

import rospy
import zmq
import json
from sensor_msgs.msg import Joy

PORT = 55557

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

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    
    hostname = 'cerlab42.lan.local.cmu.edu'
    address = f"tcp://{hostname}:{PORT}"
    socket.bind(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
    
    rospy.loginfo(f"Listening for connections on {address}")
    
    while not rospy.is_shutdown():
        try:
            message = socket.recv_string()
            process_data(message, joy_pub)
        except zmq.ZMQError as e:
            rospy.logerr(f"ZMQ error: {e}")
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

    socket.close()
    context.term()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
