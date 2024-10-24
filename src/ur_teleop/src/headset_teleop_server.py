#!/usr/bin/env python3


import rospy

import zmq

import json

import math

from sensor_msgs.msg import Joy



PORT = 55557



class JoyPublisher:

    def __init__(self):

        self.clear_initial_position()

        self.joy_pub = rospy.Publisher('/spacenav/joy', Joy, queue_size=10)



    def clear_initial_position(self):

        """Reset the initial position to None"""

        self.initial_axes = None

        rospy.loginfo("Initial position has been reset to None")



    def convert_rotation_to_radians(self, axes):

        """Convert rotation values (indices 3,4,5) from degrees to radians"""

        result = list(axes)  # Create a copy of the axes list

        for i in [3, 4, 5]:  # Convert rotation values

            if i < len(result):

                result[i] = math.radians(result[i])

        return result



    def process_data(self, data):

        try:

            joy_data = json.loads(data)

            current_axes = joy_data['axes']

            

            # Convert rotation values to radians

            current_axes = self.convert_rotation_to_radians(current_axes)

            

            # Store initial axes values if not already stored

            if self.initial_axes is None:

                self.initial_axes = current_axes

                rospy.loginfo(f"Initial axes values stored (with radians): {self.initial_axes}")

                return



            # Subtract initial values from current values

            offset_axes = [curr - init for curr, init in zip(current_axes, self.initial_axes)]

            

            joy_msg = Joy()

            joy_msg.axes = offset_axes

            joy_msg.buttons = joy_data.get('buttons', [0, 0])

            self.joy_pub.publish(joy_msg)

            

        except json.JSONDecodeError as e:

            rospy.logerr(f"JSON decode error: {e}")

        except Exception as e:

            rospy.logerr(f"An error occurred: {e}")

            self.clear_initial_position()



def main():

    rospy.init_node('remote_joy_publisher')

    joy_publisher = JoyPublisher()



    context = zmq.Context()

    socket = context.socket(zmq.SUB)

    

    hostname = 'cerlab42.lan.local.cmu.edu'

    address = f"tcp://{hostname}:{PORT}"

    socket.bind(address)

    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

    

    rospy.loginfo(f"Listening for connections on {address}")

    

    # Add a shutdown hook to close connections cleanly

    def shutdown_hook():

        rospy.loginfo("Shutting down remote_joy_publisher...")

        socket.close()

        context.term()

    

    rospy.on_shutdown(shutdown_hook)

    

    while not rospy.is_shutdown():

        try:

            message = socket.recv_string()

            joy_publisher.process_data(message)

        except zmq.ZMQError as e:

            rospy.logerr(f"ZMQ error: {e}")

            joy_publisher.clear_initial_position()  # Reset on ZMQ error

        except Exception as e:

            rospy.logerr(f"An error occurred: {e}")

            joy_publisher.clear_initial_position()  # Reset on any error



if __name__ == "__main__":

    try:

        main()

    except rospy.ROSInterruptException:

        pass