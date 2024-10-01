#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_controller')
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_pub = rospy.Publisher('joystick_commands', Twist, queue_size=1)
        
        # Initialize last command and smoothing parameters
        self.last_cmd = Twist()
        self.smoothing_factor = 1.0 # Adjust this value between 0 and 1
        
        # Deadzone parameter
        self.deadzone = 0.1
        
        # Scaling factors
        self.linear_scale = 0.5  # Max linear velocity (m/s)
        self.angular_scale = 1.0  # Max angular velocity (rad/s)

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0
        return value

    def joy_callback(self, joy_msg):
        cmd = Twist()
        
        if joy_msg.buttons[6]:  # Linear mode
            cmd.linear.x = self.apply_deadzone(joy_msg.axes[0], self.deadzone) * self.linear_scale
            cmd.linear.y = self.apply_deadzone(joy_msg.axes[1], self.deadzone) * self.linear_scale
            cmd.linear.z = self.apply_deadzone(joy_msg.axes[3], self.deadzone) * self.linear_scale
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
        else:  # Angular mode
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = -self.apply_deadzone(joy_msg.axes[1], self.deadzone) * self.angular_scale
            cmd.angular.y = -self.apply_deadzone(joy_msg.axes[0], self.deadzone) * self.angular_scale
            cmd.angular.z = self.apply_deadzone(joy_msg.axes[2], self.deadzone) * self.angular_scale

        # Apply smoothing
        for attr in ['linear', 'angular']:
            for axis in ['x', 'y', 'z']:
                current_value = getattr(getattr(cmd, attr), axis)
                last_value = getattr(getattr(self.last_cmd, attr), axis)
                smoothed_value = self.smoothing_factor * current_value + (1 - self.smoothing_factor) * last_value
                setattr(getattr(self.last_cmd, attr), axis, smoothed_value)

        self.cmd_pub.publish(self.last_cmd)

if __name__ == '__main__':
    JoystickController()
    rospy.spin()
