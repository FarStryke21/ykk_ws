#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from threading import Lock
import sys
import tf

class MoveItController:
    def __init__(self):
        rospy.init_node('moveit_controller')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        planning_grp = "arm"
        # planning_grp = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(planning_grp)
        self.cmd_sub = rospy.Subscriber('joystick_commands', Twist, self.cmd_callback, queue_size=1)
        self.group.set_pose_reference_frame("world")
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)
        
        self.linear_scale = 0.02
        self.angular_scale = 0.02
        
        self.current_cmd = Twist()
        self.cmd_lock = Lock()
        self.executing = False

        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # Give time for the TF listener to get data

        rospy.loginfo("Starting pose: {}".format(self.get_current_pose()))

    def get_current_pose(self):
        base_frame = "world"
        # base_frame = "base_link"
        try:
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, self.group.get_end_effector_link(), rospy.Time(0))
            current_pose = Pose()
            current_pose.position.x = trans[0]
            current_pose.position.y = trans[1]
            current_pose.position.z = trans[2]
            current_pose.orientation.x = rot[0]
            current_pose.orientation.y = rot[1]
            current_pose.orientation.z = rot[2]
            current_pose.orientation.w = rot[3]
            return current_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF lookup failed: %s" % e)
            return None

    def cmd_callback(self, cmd_msg):
        with self.cmd_lock:
            self.current_cmd = cmd_msg
        
        if not self.executing:
            self.execute_motion()

    def execute_motion(self):
        self.executing = True
        rospy.loginfo("Current pose: {}".format(self.get_current_pose()))
        while not rospy.is_shutdown():
            with self.cmd_lock:
                cmd = self.current_cmd
                self.current_cmd = Twist()  # Reset the command

            if cmd.linear.x == 0 and cmd.linear.y == 0 and cmd.linear.z == 0 and \
               cmd.angular.x == 0 and cmd.angular.y == 0 and cmd.angular.z == 0:
                self.executing = False
                return

            current_pose = self.get_current_pose()
            if current_pose is None:
                rospy.logerr("Failed to get current pose. Skipping this iteration.")
                continue
            
            # Update position
            current_pose.position.x += cmd.linear.x * self.linear_scale
            current_pose.position.y += cmd.linear.y * self.linear_scale
            current_pose.position.z += cmd.linear.z * self.linear_scale
            
            # Update orientation
            orientation_q = current_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            roll += cmd.angular.x * self.angular_scale
            pitch += cmd.angular.y * self.angular_scale
            yaw += cmd.angular.z * self.angular_scale
            q = quaternion_from_euler(roll, pitch, yaw)
            current_pose.orientation = Pose().orientation
            current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w = q

            rospy.loginfo("Target pose: {}".format(current_pose))

            # Set the target pose
            self.group.set_pose_target(current_pose)
            
            # Plan and execute the movement
            success = self.group.go(wait=True)
            
            if success:
                rospy.loginfo("Movement executed successfully")
            else:
                rospy.logwarn("Movement execution failed")
            
            # Clear targets after execution
            self.group.clear_pose_targets()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = MoveItController()
    controller.run()
