#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 转换 ROS 点云消息到 PCL 点云格式
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);

    // 保存到 PCD 文件
    pcl::io::savePCDFileASCII("/home/service-sdu/catkin_ws/src/pointcloud_pkg/src/output.pcd", cloud);
    ROS_INFO("Saved point cloud to output.pcd");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_saver");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/pointcloud_topic", 1, cloudCallback);

    ros::spin();
    return 0;
}

