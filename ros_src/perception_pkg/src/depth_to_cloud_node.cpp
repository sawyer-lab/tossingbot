#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
image_geometry::PinholeCameraModel cam_model;
bool cam_model_initialized = false;

cv::Mat latest_rgb_image;
ros::Time latest_rgb_stamp;
bool rgb_ready = false;

void rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
        latest_rgb_image = cv_ptr->image.clone();
        latest_rgb_stamp = rgb_msg->header.stamp;
        rgb_ready = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in RGB callback: %s", e.what());
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    static int count = 0;

    if (!cam_model_initialized || !rgb_ready) {
        ROS_WARN_THROTTLE(5, "Waiting for camera model or RGB image...");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in depth callback: %s", e.what());
        return;
    }

    const cv::Mat& depth_image = cv_ptr->image;
    if (latest_rgb_image.empty() ||
        latest_rgb_image.rows != depth_image.rows ||
        latest_rgb_image.cols != depth_image.cols) {
        ROS_WARN_THROTTLE(5, "RGB and depth images not aligned in size.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.header.frame_id = depth_msg->header.frame_id;
    cloud.width = depth_image.cols;
    cloud.height = depth_image.rows;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    int valid_points = 0;

    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            uint16_t depth = depth_image.at<uint16_t>(v, u);
            pcl::PointXYZRGB& pt = cloud.at(u, v);

            if (depth == 0) {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            float z = depth * 0.001f;
            cv::Point2d pixel(u, v);
            cv::Point3d ray = cam_model.projectPixelTo3dRay(pixel);

            pt.x = ray.x * z;
            pt.y = ray.y * z;
            pt.z = z;

            // Get RGB color
            cv::Vec3b rgb = latest_rgb_image.at<cv::Vec3b>(v, u);
            pt.b = rgb[0];
            pt.g = rgb[1];
            pt.r = rgb[2];

            valid_points++;
        }
    }

    // Downsample with voxel grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud_ptr);
    voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f); // 1cm voxel size
    voxel_filter.filter(cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    output.header = depth_msg->header;
    pub.publish(output);

    ROS_INFO_STREAM_THROTTLE(5, "Published voxelized cloud #" << ++count << " with " << valid_points << " valid points");
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cam_model.fromCameraInfo(info_msg);
    cam_model_initialized = true;
    ROS_INFO_ONCE("Camera model initialized.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_voxel_cloud_node");
    ros::NodeHandle nh;

    ROS_INFO("Starting depth_to_voxel_cloud_node...");

    ros::Subscriber info_sub = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);
    ros::Subscriber rgb_sub = nh.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
    ros::Subscriber image_sub = nh.subscribe("/camera/depth/image_raw", 1, imageCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("depth_voxel_cloud", 1);

    ROS_INFO("Node initialized. Subscribed to depth, RGB and camera info topics.");

    ros::spin();
    return 0;
}
