#!/usr/bin/env python

import tkinter as tk
import rospy
from std_srvs.srv import Empty  # Replace with the service type you're using
from master.srv import CaptureMeasurement, GetMeasurement, AlignPointClouds, PublishAllMeasurements, SavePointClouds
# Initialize ROS node
rospy.init_node('depth_gui')

# Function to call a ROS service
def call_service(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, service_type)
        request = service_type._request_class()  # Initialize the request object
        # Populate request fields here if needed
        response = service(request)
        print("Service call to {} was successful.".format(service_name))
    except rospy.ServiceException as e:
        print("Service call to {} failed: {}".format(service_name, e))


root = tk.Tk()
root.title("ROS Service GUI")

button1 = tk.Button(root, text="Capture Measurement", command=lambda: call_service('/depth_processor_zivid_cpp/capture_measurement', CaptureMeasurement))
button1.pack()

button2 = tk.Button(root, text="Retrieve Last ", command=lambda: call_service('/depth_processor_zivid_cpp/get_measurement', GetMeasurement))
button2.pack()

button3 = tk.Button(root, text="Publish All Measurements", command=lambda: call_service('/depth_processor_zivid_cpp/publish_all_measurements', PublishAllMeasurements))
button3.pack()

button4 = tk.Button(root, text="Save Measurements", command=lambda: call_service('/depth_processor_zivid_cpp/save_all_measurements', SavePointClouds))
button4.pack()

root.mainloop()