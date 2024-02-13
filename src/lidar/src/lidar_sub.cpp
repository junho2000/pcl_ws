#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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

    //Filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr x_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr y_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rm_gnd(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Voxelization
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    float voxelsize = 0.15; // m
    voxel_filter.setInputCloud(pointcloud);
    voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
    voxel_filter.filter(*ptr_filtered);

    // Pass Through Filter
    float x_offset = 1.5;
    float y_offset = 1.5;
    pcl::PassThrough<pcl::PointXYZ> xfilter;
    xfilter.setInputCloud(ptr_filtered);
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(-x_offset, x_offset);
    xfilter.filter(*center);
    xfilter.setFilterLimitsNegative(true);
    xfilter.filter(*x_out);
    pcl::PassThrough<pcl::PointXYZ> yfilter;
    yfilter.setInputCloud(center);
    yfilter.setFilterFieldName("y");
    yfilter.setFilterLimits(-y_offset, y_offset);
    yfilter.setFilterLimitsNegative(true);
    yfilter.filter(*y_out);
    *y_out += *x_out;
    *ptr_filtered = *y_out;
    
    // Ground Remover
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud (ptr_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    extract.setInputCloud(ptr_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*rm_gnd);
    *ptr_filtered = *rm_gnd;

    // publish processed lidar
    sensor_msgs::msg::PointCloud2 cloudmsg;
    pcl::toROSMsg(*ptr_filtered, cloudmsg);
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
