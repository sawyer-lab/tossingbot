#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class VoxelFilterNode {
public:
  VoxelFilterNode() {
    ros::NodeHandle pnh("~");
    pnh.param("leaf_size", leaf_size_, 0.007); // 7 mm default

    sub_ = nh_.subscribe("rgbd_camera/depth/points", 1, &VoxelFilterNode::cloudCb, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("voxel_filtered_points", 1);
    ROS_INFO("VoxelFilterNode started. leaf_size = %.4f m", leaf_size_);
  }

private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);

    pcl::PCLPointCloud2 pcl_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(pcl_pc2));
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(pcl_filtered);

    sensor_msgs::PointCloud2 out;
    pcl_conversions::fromPCL(pcl_filtered, out);
    out.header = cloud_msg->header;
    pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  double leaf_size_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxel_filter_node");
  VoxelFilterNode node;
  ros::spin();
  return 0;
}
