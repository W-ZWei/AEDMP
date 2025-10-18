#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "clicked_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg->point.x;
    marker.pose.position.y = msg->point.y;
    marker.pose.position.z = msg->point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Alpha channel
    marker.color.r = 1.0; // Red channel
    marker.color.g = 0.0; // Green channel
    marker.color.b = 0.0; // Blue channel

    marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Marker_publisher");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber sub = nh.subscribe("/clicked_point", 10, clickedPointCallback);

    ros::spin();

    return 0;
}
