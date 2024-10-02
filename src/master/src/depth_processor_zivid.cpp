#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <boost/filesystem.hpp>

#include <zivid_camera/Capture.h>

// Global Fixed Frame: world
// Camera optical frame: zivid_optical_center
// point cloud topic: /zivid_camera/points/xyzrgba
// Depth topic: /zivid_camera/depth/image
// RGB Topic: /zivid_camera/color/image_color
// For point cloud capture, call: rosservice call /zivid_camera/capture

class DepthProcessor
{
public:
     
    DepthProcessor() : nh_("~"), current_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
    {   
        cloud_sub_ = nh_.subscribe("/zivid_camera/points/xyzrgba", 1, &DepthProcessor::pointCloudCallback, this);
        depth_image_sub_ = nh_.subscribe("/zivid_camera/depth/image", 1, &DepthProcessor::depthImageCallback, this);
        rgb_image_sub_ = nh_.subscribe("/zivid_camera/color/image_color", 1, &DepthProcessor::rgbImageCallback, this);

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1);
        retrieved_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("retrieved_cloud", 1);
        combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);

        capture_service_ = nh_.advertiseService("capture_measurement", &DepthProcessor::captureMeasurement, this);
        retrieve_service_ = nh_.advertiseService("get_measurement", &DepthProcessor::getMeasurement, this);
        publish_all_service_ = nh_.advertiseService("publish_all_measurements", &DepthProcessor::publishAllMeasurements, this);
        save_all_service_ = nh_.advertiseService("save_all_measurements", &DepthProcessor::saveMeasurements, this);

        capture_client_ = nh_.serviceClient<zivid_camera::Capture>("/zivid_camera/capture");
        
        tf_listener_ = new tf::TransformListener();

    }   
    
    /**
     * @brief Service callback to capture the current point cloud measurement.
     * 
     * This function triggers the capture service for the Zivid camera to capture a point cloud. 
     * The pointcloud callback is responsible for storing the captured point cloud in the measurements_ vector.
     * This was done to ensure that the point cloud is captured only when the service is called, and that the most latest pointcloud is saved.
     * The implementation of this function is DIFFERENT compared to other variations for depth sensors which provide a constant stream of point clouds.
     * 
     * @param req Service request (empty for this service).
     * @param res Service response. Contains a success flag and a message.
     * @return true if the service completed successfully, false otherwise.
     */
    bool captureMeasurement(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        // Call the capture service
        zivid_camera::Capture capture_srv;
        if (!capture_client_.call(capture_srv))
        {
            res.success = false;
            res.message = "Failed to call /zivid_camera/capture service.";
            return false;
        }
        res.success = true;
        res.message = "Measurement captured successfully!"; 
        return true;
    }


    /**
     * @brief Service callback to retrieve and publish the last captured point cloud measurement.
     * 
     * This function retrieves the last captured point cloud from the measurements_ vector 
     * and publishes it on the retrieved_cloud topic for visualization or further processing.
     * 
     * @param req Service request (empty for this service).
     * @param res Service response. Contains a success flag and a message.
     * @return true if the service completed successfully, false otherwise.
     */
    bool getMeasurement(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        // Check if there are any measurements stored
        if (!measurements_.empty())
        {
            ROS_INFO("Measurement Size: {%zd}", measurements_.size());
            // Convert the last stored point cloud to a ROS message
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*measurements_.back(), output);

            // Publish the point cloud on the retrieved_cloud topic
            output.header.frame_id = "world";
            retrieved_cloud_pub_.publish(output);
            
            // Set the success flag and message for the service response
            res.success = true;
            res.message = "Published the last captured measurement on the retrieved_cloud topic.";
            return true;
        }
        else
        {
            // If no measurements are available, set the service response accordingly
            ROS_WARN("No measurements found! Capture something first!");
            res.success = false;
            res.message = "No measurements available.";
            return false;
        }
    }

    bool publishAllMeasurements(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

        // Check if there are any measurements stored
        if (!measurements_.empty())
        {
            // Create an empty combined point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Iterate through all measurements and append them to the combined_cloud
            for (size_t i = 0; i < measurements_.size(); i++)
            {
                ROS_INFO("Iteration : %ld", i);
                *combined_cloud += *measurements_[i];
            }

            // Convert the combined point cloud to a ROS message
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*combined_cloud, output);
            output.header.frame_id = "world";  // We'll use the world frame since it's a combined cloud

            // Publish the combined point cloud on the combined_cloud_pub_ topic
            combined_cloud_pub_.publish(output);
            
            // Set the success flag and message for the service response
            res.success = true;
            res.message = "Published the combined measurements on the combined_cloud topic.";
            return true;
        }
        else
        {
            // If no measurements are available, set the service response accordingly
            res.success = false;
            res.message = "No measurements available.";
            return false;
        }

    }

    bool saveMeasurements(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
    {
        if (!measurements_.empty()) 
        {
            // Get the current working directory
            boost::filesystem::path current_path = boost::filesystem::current_path();
            
            // Print the current working directory
            std::cout << "Current working directory: " << current_path.string() << std::endl;

            // Get current timestamp
            std::time_t t = std::time(nullptr);
            std::tm* now = std::localtime(&t);
            std::stringstream ss;
            ss << std::put_time(now, "%Y%m%d%H%M%S");
            std::string timestamp = ss.str();

            // Define the base directory and create experiments directory if it doesn't exist
            boost::filesystem::path base_dir("/home/catkin_ws/scans");
            if (!boost::filesystem::exists(base_dir))
            {
                if (!boost::filesystem::create_directory(base_dir)) 
                {
                    ROS_INFO("Created experiments directory!");
                    res.success = false;
                    res.message = "Failed to create experiments directory!";
                    return false;
                }
                else
                {
                    ROS_INFO("Created experiments directory!");
                }
            }
            else
            {
                ROS_INFO("Experiments directory already exists!");
            }
            // Print the absolute path of the base directory
            ROS_INFO("Base directory: %s", base_dir.c_str());
            // Create a subdirectory within experiments with the timestamp as its name
            boost::filesystem::path dir = base_dir / timestamp;
            if (!boost::filesystem::create_directory(dir)) 
            {
                res.success = false;
                res.message = "Failed to create timestamp directory!";
                return false;
            }
            else
            {
                ROS_INFO("Created timestamp directory: %s", timestamp.c_str());
            }

            // Create pointclouds and rgb_images subdirectories
            boost::filesystem::path pointclouds_dir = dir / "pointclouds";
            boost::filesystem::path rgb_images_dir = dir / "rgb_images";
            
            if (!boost::filesystem::create_directory(pointclouds_dir)) 
            {
                res.success = false;
                res.message = "Failed to create pointclouds directory!";
                return false;
            }
            else
            {
                ROS_INFO("Created pointclouds directory");
            }

            if (!boost::filesystem::create_directory(rgb_images_dir)) 
            {
                res.success = false;
                res.message = "Failed to create rgb_images directory!";
                return false;
            }
            else
            {
                ROS_INFO("Created rgb_images directory");
            }

            // Save each point cloud in the directory
            for (size_t i = 0; i < measurements_.size(); ++i) 
            {
                std::stringstream file_name;
                file_name << std::setw(3) << std::setfill('0') << i << ".pcd";
                std::string file_path = (pointclouds_dir / file_name.str()).string();

                if (pcl::io::savePCDFile(file_path, *measurements_[i]) == -1)
                {
                    res.success = false;
                    res.message = "Failed to save point cloud: " + file_name.str();
                    return false;
                }
                else
                {
                    ROS_INFO("Saved point cloud: %s", file_name.str().c_str());
                }
            }

            // Save each RGB image in the rgb_images directory
            for (size_t i = 0; i < rgb_images.size(); ++i) 
            {
                std::stringstream file_name;
                file_name << std::setw(3) << std::setfill('0') << i << ".png";
                std::string file_path = (rgb_images_dir / file_name.str()).string();

                if (!cv::imwrite(file_path, rgb_images[i]))
                {
                    res.success = false;
                    res.message = "Failed to save RGB image: " + file_name.str();
                    return false;
                }
                else
                {
                    ROS_INFO("Saved RGB image: %s", file_name.str().c_str());
                }
            }

            // Save each transform in a CSV file in the transforms directory
            std::ofstream transforms_file((dir / "transforms.csv").string());
            if (!transforms_file.is_open())
            {
                res.success = false;
                res.message = "Failed to create transforms.csv file!";
                return false;
            }

            transforms_file << "timestamp,frame_id,child_frame_id,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w\n";
            for (const auto& transform : transforms_)
            {
                transforms_file << transform.stamp_.toSec() << ","
                                << transform.frame_id_ << ","
                                << transform.child_frame_id_ << ","
                                << transform.getOrigin().x() << ","
                                << transform.getOrigin().y() << ","
                                << transform.getOrigin().z() << ","
                                << transform.getRotation().x() << ","
                                << transform.getRotation().y() << ","
                                << transform.getRotation().z() << ","
                                << transform.getRotation().w() << "\n";
            }

            transforms_file.close();

            res.success = true;
            res.message = "Save Success!";

            return true;
        }
        else
        {
            res.success = false;
            res.message = "Save Failed!";
            return false;
        }
        
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
    {
        ROS_INFO("Received depth image with width: %d and height: %d", image_msg->width, image_msg->height);
        // TODO: Any direct processing on the depth image if required
    }

    void rgbImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
    {
        try
        {
            // Convert the ROS image message to an OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            
            // Store the image in the vector
            rgb_images.push_back(cv_ptr->image);
            
            // Log the image dimensions
            ROS_INFO("Received RGB image with width: %d and height: %d", image_msg->width, image_msg->height);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    }


    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        tf::StampedTransform transform;

        try
        {
            tf_listener_->lookupTransform("world", "zivid_optical_center", ros::Time(0), transform);
            pcl_ros::transformPointCloud(*cloud, *current_cloud_, transform);

            // Push the transform to the transforms vector
            transforms_.push_back(transform);
            
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*current_cloud_, output);
            output.header = cloud_msg->header;  // Use the same header as the input cloud for consistency
            output.header.frame_id = "world";
            cloud_pub_.publish(output); 
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("TF exception:\n%s", ex.what());
        }

        // Pause for one second to make sure the point cloud has been received
        // ros::Duration(2.0).sleep();
        // ROS_INFO("Current Cloud Published");
        if (current_cloud_) // Check if there's any data
        {   
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>(*current_cloud_));
            measurements_.push_back(cloud_copy);  
        }
        
        else
        {
            ROS_WARN("Capture unsuccessful!");
        }

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber rgb_image_sub_;

    ros::Publisher cloud_pub_;  
    ros::Publisher retrieved_cloud_pub_;
    ros::Publisher combined_cloud_pub_;

    ros::ServiceServer capture_service_;
    ros::ServiceServer retrieve_service_;
    ros::ServiceServer publish_all_service_;
    ros::ServiceServer save_all_service_;

    ros::ServiceClient capture_client_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> measurements_;
    std::vector<tf::StampedTransform> transforms_;
    std::vector<cv::Mat> rgb_images;
    

    tf::TransformListener* tf_listener_;
    tf::TransformListener tf_listener_capture_;
    tf::TransformListener listener; 
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_processor");
    DepthProcessor dp;
    ros::spin();
    return 0;
}
