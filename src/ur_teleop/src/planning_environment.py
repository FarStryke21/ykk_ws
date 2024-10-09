#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface

class GroundPlaneAdder:
    def __init__(self):
        rospy.init_node('ground_plane_adder', anonymous=True)
        self.scene = PlanningSceneInterface()
        rospy.sleep(1.0)  # Wait for the planning scene to initialize

    def clear_scene(self):
        # Get the names of all objects in the scene
        object_names = self.scene.get_known_object_names()
        
        # Remove all objects
        for obj in object_names:
            self.scene.remove_world_object(obj)
        
        rospy.loginfo("Cleared all objects from the planning scene")
        rospy.sleep(1.0)  # Wait for the scene to update

    def add_ground_plane(self):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = -0.01
        pose.pose.orientation.w = 1.0

        # Add a plane with name 'ground', normal pointing up (z-axis), and offset of 0
        self.scene.add_plane("ground", pose, normal=(0, 0, 1), offset=0)
        rospy.loginfo("Ground plane added to the planning scene")

    def run(self):
        self.clear_scene()
        self.add_ground_plane()
        rospy.spin()

if __name__ == '__main__':
    try:
        ground_plane_adder = GroundPlaneAdder()
        ground_plane_adder.run()
    except rospy.ROSInterruptException:
        pass
