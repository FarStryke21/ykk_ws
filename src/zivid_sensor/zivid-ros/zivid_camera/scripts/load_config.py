#!/usr/bin/env python

import rospy
from zivid_camera.srv import LoadSettingsFromFile

def load_settings():
    rospy.init_node('load_zivid_settings_node')

    # Retrieve the file path from the parameter server
    file_path = rospy.get_param('~file_path', '')

    if not file_path:
        rospy.logerr("No file path provided. Please set the 'file_path' parameter.")
        return

    # Wait for the service to become available
    rospy.wait_for_service('/zivid_camera/load_settings_from_file')

    try:
        # Create a service proxy
        load_settings_service = rospy.ServiceProxy('/zivid_camera/load_settings_from_file', LoadSettingsFromFile)

        
        # Call the service
        response = load_settings_service(file_path)
        
        # Check the response
        if response:
            rospy.loginfo("Settings loaded successfully from file: %s", file_path)
        else:
            rospy.logwarn("Failed to load settings from file: %s", file_path)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        load_settings()
    except rospy.ROSInterruptException:
        pass
