// cloud_map_lib.h
#ifndef CLOUD_MAP_LIB_H
#define CLOUD_MAP_LIB_H

#include <map>
#include <utility>
#include <array>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

extern std::map<std::pair<int, std::array<float, 9>>, PointCloud::Ptr> cloud_map;

#endif // CLOUD_MAP_LIB_H