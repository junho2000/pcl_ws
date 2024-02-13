#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

class PointCloudSubscriber : public rclcpp::Node
{
public:
  PointCloudSubscriber()
  : Node("lidar_process")
  {
    RCLCPP_INFO_STREAM(this->get_logger(),"Init of point_cloud_subscriber");
    
    // pointcloud subscribe
    pointcloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", 10, std::bind(&PointCloudSubscriber::lidar_callback, this, _1));
    // pointcloud publish
    pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/pcl_points", 10);
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    // logging
    // RCLCPP_INFO_STREAM(get_logger(), "[Input PointCloud] width " << cloud_msg->width << " height " << cloud_msg->height); // width 1024 height 128

    // Receive lidar frame from ROS
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pointcloud);

    // Voxelization
    pcl::PointCloud<pcl::PointXYZ> pc_voxelized;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    float voxelsize = 0.2;
    voxel_filter.setInputCloud(pointcloud);
    voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
    voxel_filter.filter(*ptr_filtered);
    pc_voxelized = *ptr_filtered;
    // Pass Through Filter

    // Ground Remover

    // publish processed lidar
    sensor_msgs::msg::PointCloud2 cloudmsg;
    pcl::toROSMsg(pc_voxelized, cloudmsg);
    cloudmsg.header.frame_id = "laser_data_frame";
    cloudmsg.header.stamp = this->get_clock()->now();
    pointcloud_publisher->publish(cloudmsg);

  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}
