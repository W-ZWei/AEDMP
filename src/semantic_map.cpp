#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>

ros::Publisher map_pub;
bool pointInsideConvexPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& convexPolygon);

std::mutex mapMutex;

nav_msgs::OccupancyGrid inputMap;
nav_msgs::OccupancyGrid outputMap;

void cropMap(nav_msgs::OccupancyGrid& inputMap, nav_msgs::OccupancyGrid& outputMap, int x1, int y1, int x2, int y2) {
    // 计算裁剪后地图的宽度和高度
    int croppedWidth = x2 - x1 + 1;
    int croppedHeight = y2 - y1 + 1;

    // 设置裁剪后地图的信息
    outputMap.info = inputMap.info;
    outputMap.info.width = croppedWidth;
    outputMap.info.height = croppedHeight;

    // 计算裁剪后地图原点的坐标
    outputMap.info.origin.position.x = inputMap.info.origin.position.x + x1 * inputMap.info.resolution;
    outputMap.info.origin.position.y = inputMap.info.origin.position.y + y1 * inputMap.info.resolution;

    // 复制裁剪后的地图数据
    outputMap.data.resize(croppedWidth * croppedHeight);
    for (int i = x1; i <= x2; ++i) {
        for (int j = y1; j <= y2; ++j) {
            int index = i + j * inputMap.info.width;
            int croppedIndex = i - x1 + (j - y1) * croppedWidth;
            outputMap.data[croppedIndex] = inputMap.data[index];
        }
    }
}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mapMutex);
    inputMap = *msg;
    // mapReceived = true;
    std::cout << "Received map" << std::endl;

    int x1 = 800;
    int y1 = 650;
    int x2 = 1200;
    int y2 = 1150;
    auto start_time = std::chrono::high_resolution_clock::now();
    cropMap(inputMap, outputMap, x1, y1, x2, y2);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "is" << duration.count() << "ms" << std::endl; 
}



void convexPointsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

    // 假设每个Marker代表一个多边形的凸点，MarkerArray包含了所有多边形的凸点信息
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "AAAAA" << std::endl; 
    for(const auto& marker : msg->markers)
    {
        // 假设每个Marker的points字段包含了多边形的凸点信息
        std::vector<geometry_msgs::Point> convexPoints = marker.points;

        // 使用凸点信息确定多边形的形状，并将多边形内部的像素值设置为1
        // 这里假设您已经有了一个用于处理地图数据的方法，可以根据多边形的形状将地图中的像素值设置为1
        // 以下代码仅作为示例，实际实现需要根据您的地图数据结构进行调整
        for(int i = 0; i < outputMap.info.width; ++i)
        {
            for(int j = 0; j < outputMap.info.height; ++j)
            {
                geometry_msgs::Point point;
                point.x = outputMap.info.origin.position.x + (i + 0.5) * outputMap.info.resolution;
                point.y = outputMap.info.origin.position.y + (j + 0.5) * outputMap.info.resolution;
                // 检查点是否在多边形内部，根据需要设置值为1
                if(pointInsideConvexPolygon(point, convexPoints))
                {

                    int index = i + j * outputMap.info.width;
                    outputMap.data[index] = 70;
                }
            }
        }
        // 在这里发布修改后的地图数据，如使用发布者发布 mapData
    }
    map_pub.publish(outputMap);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "is" << duration.count() << "ms" << std::endl; 
}

bool pointInsideConvexPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& convexPolygon)
{
    // 假设多边形是凸多边形
    int n = convexPolygon.size();
    bool inside = true;

    for(int i = 0; i < n; i += 2)
    {
        int j = (i + 1) % n;
        float dx1 = convexPolygon[j].x - convexPolygon[i].x;
        float dy1 = convexPolygon[j].y - convexPolygon[i].y;
        float dx2 = point.x - convexPolygon[i].x;
        float dy2 = point.y - convexPolygon[i].y;
        if(dx1 * dy2 - dx2 * dy1 > 0)
        {
            inside = false;
            break;
        }
    }
    return inside;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Semantic_map");
    ros::NodeHandle nh;

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/semantic_map", 1);
    // 订阅多边形的凸点信息
    ros::Subscriber sub = nh.subscribe("/visualization_marker_array", 100, convexPointsCallback);
    ros::Subscriber map_sub = nh.subscribe("/map", 100, mapCallback);
    

    ros::spin();

    return 0;
}
