#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
import open3d as o3d
import numpy as np
import os
import time

class DepthProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('depth_processor', anonymous=True)

        # Subscribe to point cloud topic
        self.cloud_sub = rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.point_cloud_callback)

        # Advertise services
        self.capture_service = rospy.Service('/depth_processor_zivid_cpp/capture_measurement', Trigger, self.capture_measurement)
        self.save_service = rospy.Service('/depth_processor_zivid_cpp/save_all_measurements', Trigger, self.save_measurements)

        # Storage for measurements
        self.measurements = []

    def point_cloud_callback(self, cloud_msg):
        # Convert ROS PointCloud2 message to Open3D point cloud
        cloud = self.convert_ros_to_open3d(cloud_msg)
        if cloud is not None:
            self.measurements.append(cloud)
            rospy.loginfo("Point cloud captured and stored.")

    def convert_ros_to_open3d(self, ros_cloud):
        # Conversion from ROS PointCloud2 to Open3D point cloud
        try:
            # Assuming the use of a helper function to convert PointCloud2 to numpy array
            points = np.frombuffer(ros_cloud.data, dtype=np.float32).reshape(-1, 6)
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points[:, :3])
            cloud.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
            return cloud
        except Exception as e:
            rospy.logerr(f"Failed to convert point cloud: {e}")
            return None

    def capture_measurement(self, req):
        if len(self.measurements) > 0:
            rospy.loginfo("Measurement captured successfully!")
            return TriggerResponse(success=True, message="Measurement captured successfully!")
        else:
            rospy.logwarn("No measurement available.")
            return TriggerResponse(success=False, message="No measurement available.")

    def save_measurements(self, req):
        if len(self.measurements) > 0:
            timestamp = time.strftime("%Y%m%d%H%M%S")
            base_dir = "/home/catkin_ws/scans"
            os.makedirs(base_dir, exist_ok=True)
            dir_path = os.path.join(base_dir, timestamp)
            os.makedirs(dir_path, exist_ok=True)

            for i, cloud in enumerate(self.measurements):
                file_path = os.path.join(dir_path, f"{i:03}.ply")
                o3d.io.write_point_cloud(file_path, cloud)
                rospy.loginfo(f"Saved point cloud: {file_path}")

            return TriggerResponse(success=True, message="Save Success!")
        else:
            rospy.logwarn("No measurements to save.")
            return TriggerResponse(success=False, message="Save Failed!")

if __name__ == "__main__":
    dp = DepthProcessor()
    rospy.spin()
