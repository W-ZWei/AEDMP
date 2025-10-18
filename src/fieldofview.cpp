#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/format.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/filesystem.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>  
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h> // 用于将PCL点云数据转换为ROS消息
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <map>
#include <chrono>

using namespace std;
//namespace fs = std::experimental::filesystem;
tf2_ros::Buffer tf_buffer; //坐标转换
using Eigen::Matrix4d;
Eigen::Matrix4d T;
Eigen::Affine3d transformation;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr pcl_cloud(new PointCloud);


cv::Mat colorImgs, depthImgs;    // 彩色图和深度图

geometry_msgs::TransformStamped transformStamped; //位姿消息

double cx = 320.5;
double cy = 240.5;
double fx = 522.191;
double fy = 522.191;
double depthScale = 1000.0; //相机内参

ros::Publisher pointcloud_pub; //新增点云消息发布者 pointcloud2类型

void poseCallback(const geometry_msgs::TransformStamped& transformStamped) {
    
    Eigen::Vector3d t(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,        
        transformStamped.transform.translation.z
    );
    Eigen::Quaterniond q(
    transformStamped.transform.rotation.w,
    transformStamped.transform.rotation.x,
    transformStamped.transform.rotation.y,
    transformStamped.transform.rotation.z
    );
    transformation = Eigen::Translation3d(t) * q;
    
}

void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    
    try {

        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_float = cv_ptr->image;
        cv::Mat depth_uint16;
        depth_float.convertTo(depth_uint16, CV_16U, 1000.0);
        depthImgs = depth_uint16;
        //cout << "getDepth" << endl;
        transformStamped = tf_buffer.lookupTransform("map", "xtion_rgb_optical_frame", ros::Time(0));
        poseCallback(transformStamped);     //getPOSE
        T = transformation.matrix(); 
        //cout << "getPose" << endl;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

//rgb接收者回调函数
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    
    try {
        //cv_bridge::CvImageConstPtr cv_ptr;
        //cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        //cv::Mat color = cv_ptr->image;
        //colorImgs = color;               //getRGB
        colorImgs = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
        //cout << "getColor" << endl;
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


//下采样
void toVoxelGrid(double resolution = 0.03) {
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(resolution, resolution, resolution);
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.filter(*pcl_cloud);

}



void pointcloudPub(){

    PointT pcl_point;
    for (int v = 0; v < depthImgs.rows; v++) {
        for (int u = 0; u < depthImgs.cols; u++) {
            unsigned int d = depthImgs.at<unsigned short>(v, u); // 深度值
            if (d == 0) continue;
                
            pcl_point.z = double(d) / depthScale;
            pcl_point.x = (u - cx) * pcl_point.z / fx;
            pcl_point.y = (v - cy) * pcl_point.z / fy;

            pcl_point.b = colorImgs.at<cv::Vec3b>(v,u)[0];
            pcl_point.g = colorImgs.at<cv::Vec3b>(v,u)[1];
            pcl_point.r = colorImgs.at<cv::Vec3b>(v,u)[2];

            pcl_cloud->points.push_back(pcl_point);

        }
    }


        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, T);  //transform to map
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pcl_cloud, cloud_msg);               //pcl to ROSMSG:pointcloud2
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";   // 设置坐标系
        pointcloud_pub.publish(cloud_msg);   //发布PointCloud2消息
        pcl_cloud->clear();
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "FieldofView");
    ros::NodeHandle n;

    ros::Subscriber color_sub = n.subscribe("/xtion/rgb/image_raw", 1, imageCallback);              //创建rgb图像接收者
    ros::Subscriber depth_sub = n.subscribe("/xtion/depth_registered/image_raw", 1, depthCallback);   //创建depth图像接收者

    pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/FieldofView", 1000);                    //创建pointcloud2消息的发布者

    // 初始化TF2监听器
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate wait(5);            
    wait.sleep();               //wait all MSG read

    while (ros::ok()) {
        ros::spinOnce();
        pointcloudPub();
        
    }

    return 0;
}

