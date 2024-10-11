#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface

class CageAdder:
    def __init__(self):
        rospy.init_node('cage_adder', anonymous=True)
        self.scene = PlanningSceneInterface()
        rospy.sleep(1.0)  # Wait for the planning scene to initialize

    def clear_scene(self):
        object_names = self.scene.get_known_object_names()
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

        self.scene.add_plane("ground", pose, normal=(0, 0, 1), offset=0)
        rospy.loginfo("Ground plane added to the planning scene")

    def add_cage_walls(self):
        cage_height = 1.0
        cage_size = 1.2
        wall_thickness = 0.01

        # Add walls
        self.add_box("wall_front", 0, cage_size/2, cage_height/2, cage_size, wall_thickness, cage_height)
        self.add_box("wall_back", 0, -cage_size/2, cage_height/2, cage_size, wall_thickness, cage_height)
        self.add_box("wall_left", -cage_size/2, 0, cage_height/2, wall_thickness, cage_size, cage_height)
        self.add_box("wall_right", cage_size/2, 0, cage_height/2, wall_thickness, cage_size, cage_height)

        rospy.loginfo("Cage walls added to the planning scene")

    def add_box(self, name, x, y, z, size_x, size_y, size_z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.scene.add_box(name, pose, (size_x, size_y, size_z))

    def run(self):
        self.clear_scene()
        self.add_ground_plane()
        self.add_cage_walls()
        rospy.spin()

if __name__ == '__main__':
    try:
        cage_adder = CageAdder()
        cage_adder.run()
    except rospy.ROSInterruptException:
        pass
