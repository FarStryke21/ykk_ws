#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_controller')
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        
        # Initialize command and smoothing parameters
        self.current_cmd = TwistStamped()
        self.smoothing_factor = 0.5 # Adjust this value between 0 and 1
        
        # Deadzone parameter
        self.deadzone = 0.1
        
        # Scaling factors
        self.linear_scale = 0.1  # Max linear velocity (m/s)
        self.angular_scale = 0.1  # Max angular velocity (rad/s)

        # Get the planning frame from ROS parameter server
        self.planning_frame = rospy.get_param('/robot_description_planning/planning_frame', 'base_link')

        # Set up timer for continuous command publishing
        self.publish_rate = 50  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_command)

        # Flag to determine control mode
        self.linear_mode = False

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0
        return value

    def joy_callback(self, joy_msg):
        self.linear_mode = joy_msg.buttons[4]
        
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.planning_frame
        
        if self.linear_mode:  # Linear mode
            cmd.twist.linear.x = -self.apply_deadzone(joy_msg.axes[0], self.deadzone) * self.linear_scale
            cmd.twist.linear.y = self.apply_deadzone(joy_msg.axes[1], self.deadzone) * self.linear_scale
            cmd.twist.linear.z = self.apply_deadzone(joy_msg.axes[4], self.deadzone) * self.linear_scale
            cmd.twist.angular.x = 0
            cmd.twist.angular.y = 0
            cmd.twist.angular.z = 0
        else:  # Angular mode
            cmd.twist.linear.x = 0
            cmd.twist.linear.y = 0
            cmd.twist.linear.z = 0
            cmd.twist.angular.x = -self.apply_deadzone(joy_msg.axes[1], self.deadzone) * self.angular_scale
            cmd.twist.angular.y = -self.apply_deadzone(joy_msg.axes[0], self.deadzone) * self.angular_scale
            cmd.twist.angular.z = self.apply_deadzone(joy_msg.axes[3], self.deadzone) * self.angular_scale

        # Apply smoothing
        for attr in ['linear', 'angular']:
            for axis in ['x', 'y', 'z']:
                current_value = getattr(getattr(cmd.twist, attr), axis)
                last_value = getattr(getattr(self.current_cmd.twist, attr), axis)
                smoothed_value = self.smoothing_factor * current_value + (1 - self.smoothing_factor) * last_value
                setattr(getattr(self.current_cmd.twist, attr), axis, smoothed_value)

    def publish_command(self, event):
        self.current_cmd.header.stamp = rospy.Time.now()
        self.cmd_pub.publish(self.current_cmd)

if __name__ == '__main__':
    try:
        JoystickController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
