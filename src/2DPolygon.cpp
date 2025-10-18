#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

using namespace geometry_msgs;
using namespace ros;

ros::Publisher PC_pub;
ros::Publisher View_pub;

// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
// {

//     // Convert ROS PointCloud2 message to PCL PointCloud<PointXYZRGB> data type
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::fromROSMsg(*cloud_msg, *cloud);

//     // Project the point cloud onto the XY plane
//     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
//     // pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//     // proj.setModelType(pcl::SACMODEL_PLANE);
//     // proj.setAxis(Eigen::Vector3f(0, 0, 1));
//     // proj.setInputCloud(cloud);
//     // proj.filter(*cloud_projected);

//     for (size_t i = 0; i < cloud->size(); ++i){
//       (*cloud)[i].z = 0;
//     }

//     // Compute convex hull of the projected points
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::ConvexHull<pcl::PointXYZRGB> chull;
//     chull.setInputCloud(cloud);
//     chull.reconstruct(*cloud_hull);

//     //Publish the convex hull as a polygon
//     PolygonStamped polygon_msg;
//     polygon_msg.header.frame_id = "/map";
//     polygon_msg.header.stamp = ros::Time::now();
//     for (const auto& point : cloud_hull->points){
      
//       Point32 p;
//       p.x = point.x;
//       p.y = point.y;
//       p.z = point.z;
//       polygon_msg.polygon.points.push_back(p);

//     }
    
//     PC_pub.publish(polygon_msg);

// }

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);


  for (size_t i = 0; i < cloud->size(); ++i){
    (*cloud)[i].z = 0;
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud(cloud);
  chull.reconstruct(*cloud_hull);


  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.04;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1.0;
  for (size_t i = 0; i < cloud_hull->points.size() - 1; ++i) {
      geometry_msgs::Point p1, p2;
      p1.x = cloud_hull->points[i].x;
      p1.y = cloud_hull->points[i].y;
      p1.z = cloud_hull->points[i].z;

      p2.x = cloud_hull->points[i + 1].x;
      p2.y = cloud_hull->points[i + 1].y;
      p2.z = cloud_hull->points[i + 1].z;

      marker.points.push_back(p1);
      marker.points.push_back(p2);
  }

    // 添加凸包的最后一条边
  geometry_msgs::Point p1, p2;
  p1.x = cloud_hull->points.back().x;
  p1.y = cloud_hull->points.back().y;
  p1.z = cloud_hull->points.back().z;

  p2.x = cloud_hull->points.front().x;
  p2.y = cloud_hull->points.front().y;
  p2.z = cloud_hull->points.front().z;

  marker.points.push_back(p1);
  marker.points.push_back(p2);

  PC_pub.publish(marker);// 发布marker数组
}



void FieldCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);


  for (size_t i = 0; i < cloud->size(); ++i){
    (*cloud)[i].z = 0;
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud(cloud);
  chull.reconstruct(*cloud_hull);


  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.04;
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1.0;
  for (size_t i = 0; i < cloud_hull->points.size() - 1; ++i) {
      geometry_msgs::Point p1, p2;
      p1.x = cloud_hull->points[i].x;
      p1.y = cloud_hull->points[i].y;
      p1.z = cloud_hull->points[i].z;

      p2.x = cloud_hull->points[i + 1].x;
      p2.y = cloud_hull->points[i + 1].y;
      p2.z = cloud_hull->points[i + 1].z;

      marker.points.push_back(p1);
      marker.points.push_back(p2);
  }

    // 添加凸包的最后一条边
  geometry_msgs::Point p1, p2;
  p1.x = cloud_hull->points.back().x;
  p1.y = cloud_hull->points.back().y;
  p1.z = cloud_hull->points.back().z;

  p2.x = cloud_hull->points.front().x;
  p2.y = cloud_hull->points.front().y;
  p2.z = cloud_hull->points.front().z;

  marker.points.push_back(p1);
  marker.points.push_back(p2);


  View_pub.publish(marker); // 发布marker数组
}
// void FieldCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
// {

//     // Convert ROS PointCloud2 message to PCL PointCloud<PointXYZRGB> data type
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::fromROSMsg(*cloud_msg, *cloud);

//     // Project the point cloud onto the XY plane
//     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
//     // pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//     // proj.setModelType(pcl::SACMODEL_PLANE);
//     // proj.setAxis(Eigen::Vector3f(0, 0, 1));
//     // proj.setInputCloud(cloud);
//     // proj.filter(*cloud_projected);

//     for (size_t i = 0; i < cloud->size(); ++i){
//       (*cloud)[i].z = 0;
//     }

//     // Compute convex hull of the projected points
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::ConvexHull<pcl::PointXYZRGB> chull;
//     chull.setInputCloud(cloud);
//     chull.reconstruct(*cloud_hull);

//     //Publish the convex hull as a polygon
//     PolygonStamped polygon_msg;
//     polygon_msg.header.frame_id = "/map";
//     polygon_msg.header.stamp = ros::Time::now();
//     for (const auto& point : cloud_hull->points){
      
//       Point32 p;
//       p.x = point.x;
//       p.y = point.y;
//       p.z = point.z;
//       polygon_msg.polygon.points.push_back(p);

//     }
//     View_pub.publish(polygon_msg);

// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Polygon2D");
    ros::NodeHandle nh;

    // Subscribe to the input point cloud topic
    ros::Subscriber PC_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud_topic", 1, cloudCallback);
    ros::Subscriber View_sub = nh.subscribe<sensor_msgs::PointCloud2>("/FieldofView", 1, FieldCallback);

    PC_pub = nh.advertise<visualization_msgs::Marker>("/pointcloud_2D", 1);
    View_pub = nh.advertise<visualization_msgs::Marker>("/FieldofView_2D", 1);

    ros::spin();

    return 0;
}
