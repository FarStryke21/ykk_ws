#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

class SpaceMouseController:
    def __init__(self):
        rospy.init_node('spacemouse_controller')
        self.joy_sub = rospy.Subscriber('/spacenav/joy', Joy, self.joy_callback)
        self.cmd_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        
        # Initialize command and smoothing parameters
        self.current_cmd = TwistStamped()
        self.smoothing_factor = 0.0 # Adjust this value between 0 and 1
        
        # Deadzone parameter
        self.deadzone = 0.0
        
        # Scaling factors
        self.linear_scale = 0.1  # Max linear velocity (m/s)
        self.angular_scale = 0.1  # Max angular velocity (rad/s)

        # Get the planning frame from ROS parameter server
        self.planning_frame = rospy.get_param('/robot_description_planning/planning_frame', 'base_link')

        # Set up timer for continuous command publishing
        self.publish_rate = 500  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_command)

        # Service clients for button presses
        self.button_states = [False, False]  # Initialize button states
        self.service1 = rospy.ServiceProxy('/depth_processor_zivid_cpp/capture_measurement', Trigger)
        self.service2 = rospy.ServiceProxy('/depth_processor_zivid_cpp/save_all_measurements', Trigger)

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0
        return value

    def joy_callback(self, joy_msg):
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.planning_frame
        
        # Update linear and angular velocities from SpaceMouse input
        # Assuming the first 6 axes correspond to 6DOF input
        # Swap X and Y axes for both linear and angular velocities
        cmd.twist.linear.x = self.apply_deadzone(joy_msg.axes[1], self.deadzone) * self.linear_scale
        cmd.twist.linear.y = self.apply_deadzone(joy_msg.axes[0], self.deadzone) * self.linear_scale
        cmd.twist.linear.z = self.apply_deadzone(joy_msg.axes[2], self.deadzone) * self.linear_scale
        cmd.twist.angular.x = self.apply_deadzone(joy_msg.axes[4], self.deadzone) * self.angular_scale
        cmd.twist.angular.y = self.apply_deadzone(joy_msg.axes[3], self.deadzone) * self.angular_scale
        cmd.twist.angular.z = self.apply_deadzone(joy_msg.axes[5], self.deadzone) * self.angular_scale

        # Apply smoothing
        for attr in ['linear', 'angular']:
            for axis in ['x', 'y', 'z']:
                current_value = getattr(getattr(cmd.twist, attr), axis)
                last_value = getattr(getattr(self.current_cmd.twist, attr), axis)
                smoothed_value = self.smoothing_factor * current_value + (1 - self.smoothing_factor) * last_value
                setattr(getattr(self.current_cmd.twist, attr), axis, smoothed_value)

        self.current_cmd = cmd

        # Check button presses and call services
        for i, button_state in enumerate(joy_msg.buttons[:2]):  # Only check first two buttons
            if button_state and not self.button_states[i]:
                self.call_service(i)
            self.button_states[i] = button_state

    def call_service(self, button_index):
        try:
            if button_index == 0:
                rospy.loginfo("Capturing Measurement!")
                response = self.service1()
            elif button_index == 1:
                rospy.loginfo("Saving Captured Measurements!")
                response = self.service2()
            rospy.loginfo(f"Service response: {response.success}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
    def publish_command(self, event):
        self.current_cmd.header.stamp = rospy.Time.now()
        self.cmd_pub.publish(self.current_cmd)

if __name__ == '__main__':
    try:
        SpaceMouseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
