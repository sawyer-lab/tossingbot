#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(cloud_filtered);

    sensor_msgs::PointCloud2 filtered_points;
    pcl_conversions::fromPCL(cloud_filtered, filtered_points);

    pub.publish(filtered_points);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxel_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("rgbd_camera/depth/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_filtered_points", 1);
    ros::spin();
    return 0;
}