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
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/surface/convex_hull.h>
#include <map>



using namespace std;
//namespace fs = std::experimental::filesystem;
tf2_ros::Buffer tf_buffer; //坐标转换
using Eigen::Matrix4d;
Eigen::Matrix4d T;
Eigen::Affine3d transformation;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr pcl_cloud(new PointCloud);


cv::Mat colorImgs, depthImgs, segImgs;    // 彩色图和深度图

int Mask;
uchar pixel,pixel1;

std::map<std::pair<int, std::array<float, 9>>, PointCloud::Ptr> cloud_map;

geometry_msgs::TransformStamped transformStamped; //位姿消息

double cx = 320.5;
double cy = 240.5;
double fx = 522.191;
double fy = 522.191;
double depthScale = 1000.0; //相机内参

ros::Publisher pointcloud_pub; //新增点云消息发布者 pointcloud2类型

//Send and Receive ShareMemory
void sendImgReseiveSeg(){
    boost::interprocess::shared_memory_object shm_r(boost::interprocess::create_only, "image_data", boost::interprocess::read_write); 
    // 设置共享内存大小
    shm_r.truncate(480 * 640 * 3);
    // 映射共享内存到进程的地址空间 
    boost::interprocess::mapped_region region_rgb(shm_r, boost::interprocess::read_write); 
    // 将图像数据拷贝到共享内存 
    std::memcpy(region_rgb.get_address(), colorImgs.data, 480 * 640 * 3); 
    boost::interprocess::shared_memory_object shm_d(boost::interprocess::create_only, "depth_data", boost::interprocess::read_write); 
    shm_d.truncate(480 * 640 * sizeof(uint16_t));
    // 映射共享内存到进程的地址空间 
    boost::interprocess::mapped_region region_dep(shm_d, boost::interprocess::read_write); 
    // 将图像数据拷贝到共享内存 
    std::memcpy(region_dep.get_address(), depthImgs.data, 480 * 640 * sizeof(uint16_t)); 

    while (true){
        try{
            boost::interprocess::shared_memory_object shm_s(boost::interprocess::open_only, "seg_data", boost::interprocess::read_only); 
            boost::interprocess::mapped_region region_seg(shm_s, boost::interprocess::read_only); 
            void* mapped_memory = region_seg.get_address();
            std::size_t image_size = 614400;
            cv::Mat segImg (480, 640, CV_8UC2);
            segImgs = segImg;
            std::memcpy(segImgs.data, mapped_memory, image_size);

            boost::interprocess::shared_memory_object shm_mask(boost::interprocess::open_only, "seg_done", boost::interprocess::read_only); 
            boost::interprocess::mapped_region region_mask(shm_mask, boost::interprocess::read_only); 
            void* mapped_memory_mask = region_mask.get_address();
            std::memcpy(&Mask, mapped_memory_mask, sizeof(int));
            //cout << Mask << endl;


            boost::interprocess::shared_memory_object::remove("seg_data");
            boost::interprocess::shared_memory_object::remove("seg_done");
            break;
        } catch (const boost::interprocess::interprocess_exception& e){
            ros::Duration(0.02).sleep();
        }
    }


}

//将转化后的位姿话题转化为EIgen形式
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

//depth接收者回调函数
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

//计算重叠体积(面积)IOM，用于新旧点云相似性比较
array<float, 3> computeIOM(const PointT min_point_old, const PointT max_point_old, const PointT min_point_new, const PointT max_point_new) {

    //intersection old new
    array<float, 3> ion = {0, 0, 0};
    
    Eigen::Vector3f min_point_intersection = min_point_old.getVector3fMap().cwiseMax(min_point_new.getVector3fMap());
    Eigen::Vector3f max_point_intersection = max_point_old.getVector3fMap().cwiseMin(max_point_new.getVector3fMap());

    //计算新旧两个点云交集的体积ss
    Eigen::Vector3f dimensions_intersection = max_point_intersection - min_point_intersection;

    for (int i = 0; i < dimensions_intersection.size(); i++) {
        if (dimensions_intersection[i] <= 0) {

            return ion;
        }
    }

    float volume_intersection = dimensions_intersection.prod();


    //计算新旧两个点云中更小的体积
    Eigen::Vector3f dimensions_old = max_point_old.getVector3fMap() - min_point_old.getVector3fMap();
    Eigen::Vector3f dimensions_new = max_point_new.getVector3fMap() - min_point_new.getVector3fMap();
    float volum_old = dimensions_old.prod();
    float volum_new = dimensions_new.prod();

    float volum_OldorNew_min = std::min(volum_new, volum_old);
    float volum_OldorNew_max = std::max(volum_new, volum_old);
    
    //计算IOM，即两者交集体积与两者更小体积间的比值
    ion[0] = volume_intersection / volum_OldorNew_min;
    ion[1] = volum_old;
    ion[2] = volum_new;
    // cout << "volum_OldorNew_min: " << volum_OldorNew_min << "volume_intersection: "<< volume_intersection << endl;
    // cout << "volum_old: " << volum_old << "volum_new: " << volum_new << endl;

    return ion;
}

bool isSameObjectPointcloud(const array<float, 9> oldarray, const array<float, 9> newarray, float ION_max = 0.5, float distance = 0.2) {

    PointT min_point_old, max_point_old, min_point_new, max_point_new;

    min_point_old.x = oldarray[3];
    min_point_old.y = oldarray[4];
    min_point_old.z = oldarray[5];

    max_point_old.x = oldarray[6];
    max_point_old.y = oldarray[7];
    max_point_old.z = oldarray[8];

    min_point_new.x = newarray[3];
    min_point_new.y = newarray[4];
    min_point_new.z = newarray[5];

    max_point_new.x = newarray[6];
    max_point_new.y = newarray[7];
    max_point_new.z = newarray[8];

    float x_width = max_point_new.x - min_point_new.x;
    float y_width = max_point_new.y - min_point_new.y;
    float z_width = max_point_new.z - min_point_new.z;

    // 增加一个二维判定，如果点云在某一维度很薄，则忽略这一维度，转而计算面积
    if (x_width < 0.1) {
        max_point_new.x = 2.0;
        min_point_new.x = 1.0;
        max_point_old.x = 2.0;
        min_point_old.x = 1.0;

        array<float, 3> ION = computeIOM(min_point_old, max_point_old, min_point_new, max_point_new);
        if (ION[0] > ION_max) {
            float distance_x = std::abs(oldarray[0] - newarray[0]);
            if (distance_x < distance) {
                return true;
            }
        }
    }

    if (y_width < 0.1) {
        max_point_new.y = 2.0;
        min_point_new.y = 1.0;
        max_point_old.y = 2.0;
        min_point_old.y = 1.0;

        array<float, 3> ION = computeIOM(min_point_old, max_point_old, min_point_new, max_point_new);
        if (ION[0] > ION_max) {
            float distance_y = std::abs(oldarray[1] - newarray[1]);
            if (distance_y < distance) {
                return true;
            }
        }
    }

    if (z_width < 0.1) {
        max_point_new.z = 2.0;
        min_point_new.z = 1.0;
        max_point_old.z = 2.0;
        min_point_old.z = 1.0;

        array<float, 3> ION = computeIOM(min_point_old, max_point_old, min_point_new, max_point_new);
        if (ION[0] > ION_max) {
            float distance_z = std::abs(oldarray[2] - newarray[2]);
            if (distance_z < distance) {
                return true;
            }
        }
    }

    float thin = std::min(std::min(x_width, y_width), z_width);
    float thick = std::max(std::max(x_width, y_width), z_width);
    float ION_min = ION_max * thin / thick;

    array<float, 3> ION = computeIOM(min_point_old, max_point_old, min_point_new, max_point_new);
    if (ION[0] > ION_min) {
        return true;
    }

    return false;

}

//物品更新逻辑，根据距离、语义、重叠体积进行评估
bool AddObjectPointcloud(int target_key, array<float, 9> new_array, 
    float max_Distance = 2.4, float min_Distance = 0.1){
    bool isAdd = false;
    bool Add = true;

    std::pair<int, array<float, 9>> key(target_key, new_array);

    if (cloud_map.empty()) {
        cout << "Empty" << endl;
        cloud_map[key] = pcl_cloud->makeShared();
        return Add;
    } else {

        for (auto entry = cloud_map.begin(); entry != cloud_map.end();) {

            int key_int = entry->first.first;

            const array<float, 9>& key_array = entry->first.second;
            // std::array<float, 3> center = {};
            // std::array<float, 3> min = {};
            // std::array<float, 3> max = {};
            // std::copy(key_array.begin(), key_array.begin() + 3, center.begin());
            // std::copy(key_array.begin() + 3, key_array.begin() + 6, min.begin());
            // std::copy(key_array.begin() + 6, key_array.begin() + 9, max.begin());

            PointCloud::Ptr value = entry->second;

            float distance = std::cbrt((new_array[0] - key_array[0]) * (new_array[0] - key_array[0])
            + (new_array[1] - key_array[1]) * (new_array[1] - key_array[1])
            + (new_array[2] - key_array[2]) * (new_array[2] - key_array[2]));
            cout << "diastance: " << distance << endl;
            if (distance < max_Distance) {
                //获取新旧点云个数
                size_t size_oldPoints = value->size();
                size_t size_newPoints = pcl_cloud->size();

                if (isSameObjectPointcloud(key_array, new_array)) {
                    Add = false;
                    
                    if(size_newPoints > size_oldPoints) {
                        entry = cloud_map.erase(entry);

                        if (!isAdd) {
                            cloud_map[key] = pcl_cloud->makeShared();
                            isAdd = true;
                        }
                    }
                    ++entry;
                    continue;
                } 

                if (key_int == target_key && distance < min_Distance) {
                    if(size_newPoints > size_oldPoints) {
                        entry = cloud_map.erase(entry);

                        if (!isAdd) {
                            cloud_map[key] = pcl_cloud->makeShared();
                            isAdd = true;
                        }
                    }
                    Add = false;
                    ++entry;
                    continue;
                }
            }
            ++entry;
        }
    }

    if (!isAdd && Add) {
        cloud_map[key] = pcl_cloud->makeShared();
        isAdd = true;
    }
    return isAdd;
}


//点云滤波
void PointCloudOutlierRemoval(int MeanK = 50, double Thresh = 1.0) {
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    statistical_filter.setInputCloud(pcl_cloud);
    statistical_filter.setMeanK(MeanK);
    statistical_filter.setStddevMulThresh(Thresh);
    statistical_filter.filter(*pcl_cloud);
}

//下采样
void toVoxelGrid(double resolution = 0.01) {
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(resolution, resolution, resolution);
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.filter(*pcl_cloud);

}

//高度滤波
void lowPointRemoval(float threshold = 0.1) {
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, threshold)));

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(pcl_cloud);
    condrem.filter(*pcl_cloud);
    
}

//深度滤波（未使用）
void depthFilter(int i, double depthChangeThreshold = 2500){
    PointT pcl_point;
    double sum = 0;
    int count = 0;
    for (int v = 0; v < segImgs.rows; v++) {
        for (int u = 0; u < segImgs.cols; u++) {
            unsigned int d = depthImgs.at<unsigned short>(v, u); // 深度值
            int InstanceMask = segImgs.at<cv::Vec2b>(v,u)[0];

            if (d == 0 || InstanceMask != i) continue;

            sum += d;
            count++;
        }
    }

    cout << "count: " << count << endl;
    float ave = sum / count;
    int counta = 0;
    for (int v = 0; v < segImgs.rows; v++) {
        for (int u = 0; u < segImgs.cols; u++) {
            unsigned int d = depthImgs.at<unsigned short>(v, u); // 深度值
            int InstanceMask = segImgs.at<cv::Vec2b>(v,u)[0];
            float cha = (d - ave)  * (d - ave);
            if (d == 0 || InstanceMask != i || cha > depthChangeThreshold) continue;


            pcl_point.z = double(d) / depthScale;
            pcl_point.x = (u - cx) * pcl_point.z / fx;
            pcl_point.y = (v - cy) * pcl_point.z / fy;

            pcl_point.b = colorImgs.at<cv::Vec3b>(v,u)[0];
            pcl_point.g = colorImgs.at<cv::Vec3b>(v,u)[1];
            pcl_point.r = colorImgs.at<cv::Vec3b>(v,u)[2];
            pcl_cloud->points.push_back(pcl_point);
            pixel = segImgs.at<cv::Vec2b>(v,u)[1];
            counta++;
        }
    }  
    cout << "counta: " << counta << endl;

}

//区域增长
bool regionGrowing(int a, int depthThreshold = 2500, int minPoints = 200) {
    PointT pcl_point;
    cv::Mat visited = cv::Mat::zeros(depthImgs.size(), CV_16U);

    int rows = depthImgs.rows;
    int cols = depthImgs.cols;

    int seedX = 0;
    int seedY = 0;
    int count = 0;

    for (int v = 0; v < rows; v++) {
        for (int u = 0; u < cols; u++) {
            unsigned int d = depthImgs.at<unsigned short>(v, u); // 深度值
            int InstanceMask = segImgs.at<cv::Vec2b>(v,u)[0];

            if (d == 0 || InstanceMask != a) continue;
            seedX += u;
            seedY += v;
            count++;
        }
    } 
    cout << "count: " << count << endl;

    if (count < minPoints) {
        return false;
    }
    seedX /= count;
    seedY /= count;

    while(segImgs.at<cv::Vec2b>(seedY, seedX)[0] != a) {
            
        seedX++;
        seedY++;
        if (seedX > cols || seedY > rows) {
            return false;
        }
    }

    pixel = segImgs.at<cv::Vec2b>(seedY,seedX)[1];//获得识别到的标签信息

    cv::Mat outputImage = cv::Mat::zeros(depthImgs.size(), CV_16U);

    queue<cv::Point> q;

    q.push(cv::Point(seedX, seedY));

    visited.at<unsigned short>(seedY, seedX) = 1;
    
    while (!q.empty()) {
        cv::Point current = q.front();
        q.pop();

        int x = current.x;
        int y = current.y;
        unsigned int seedValue = depthImgs.at<unsigned short>(y, x);

        if (segImgs.at<cv::Vec2b>(y,x)[0] == a) {

            for(int i = -1; i <= 1; ++i) {
                for(int j = -1; j <= 1; ++j){
                    int nx = x + i;
                    int ny = y + j;
                    if(nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                        unsigned int mark = visited.at<unsigned short>(ny, nx);
                        if (mark == 0) {
                            unsigned int neighborValue = depthImgs.at<unsigned short>(ny, nx);
                            int absNum = (seedValue - neighborValue) * (seedValue - neighborValue);
                            if (absNum < depthThreshold) {
                                q.push(cv::Point(nx, ny));
                                visited.at<unsigned short>(ny, nx) = 1;
                                outputImage.at<unsigned short>(ny, nx) = depthImgs.at<unsigned short>(ny, nx);
                            }
                        }
                    }
                }
            }           
        }
    }

    for (int v = 0; v < outputImage.rows; v++) {
        for (int u = 0; u < outputImage.cols; u++) {
            unsigned int d = outputImage.at<unsigned short>(v, u); // 深度值
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
    return true;
}

array<float, 9> getBoxAndCenter() {
    //getboundingbox
    array<float, 9> threepoints;
    PointT min_point, max_point;
    pcl::getMinMax3D(*pcl_cloud, min_point, max_point);
    threepoints[3] = min_point.x;
    threepoints[4] = min_point.y;
    threepoints[5] = min_point.z;
    threepoints[6] = max_point.x;
    threepoints[7] = max_point.y;
    threepoints[8] = max_point.z;

    //compute center   threepoints存储三个点，第一个是中心点，第二个是最小点，第三个是最大点
    threepoints[0] = 0.5 * (min_point.x + max_point.x);
    threepoints[1] = 0.5 * (min_point.y + max_point.y);
    threepoints[2] = 0.5 * (min_point.z + max_point.z);

    return threepoints;
}

//根据墙体的形状特征去除墙壁和地面
bool identifyWallofShape(array<float, 9> BoxAndCenter) {
    //compute weight or length height
    float width = std::abs(BoxAndCenter[3] - BoxAndCenter[6]);
    float length = std::abs(BoxAndCenter[4] - BoxAndCenter[7]);
    float height = std::abs(BoxAndCenter[5] - BoxAndCenter[8]);

    float position_z = BoxAndCenter[2];

    cout << "width: " << width << "length: " << length << "height: " << height << endl;

    if (width < 0.05 || length < 0.05 || position_z < 0.05) {

        pcl_cloud->clear();
        return true;
    } else {
        return false;
    }

}

//根据墙体的颜色特征去除墙壁
void wallFilterofColor() {
    for (int v = 0; v < segImgs.rows; v++) {
        for (int u = 0; u < segImgs.cols; u++) {

            int r = colorImgs.at<cv::Vec3b>(v,u)[2];
            int g = colorImgs.at<cv::Vec3b>(v,u)[1];
            int b = colorImgs.at<cv::Vec3b>(v,u)[0];

            if (r == b && b == g && r < 75 && r > 50) {

                depthImgs.at<unsigned short>(v, u) = 0;
            }
        }
    }  

}

//Publish PointCloud2
void saveStuffPointcloud(){

    wallFilterofColor();

    for (int i = 1; i < Mask; i++){
        // auto start_time = std::chrono::high_resolution_clock::now();
        if (!regionGrowing(i)) continue;
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // std::cout << "is" << duration.count() << "ms" << std::endl; 
        if (pcl_cloud->empty()) continue;
        
        //下采样
        toVoxelGrid();
        
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, T);

        //地面点云滤除
        if (pcl_cloud->empty()) continue;
        lowPointRemoval();

        //离群点滤波
        if (pcl_cloud->empty()) continue;
        PointCloudOutlierRemoval();

        //getboundingbox
        array<float, 9> BoxAndCenter = getBoxAndCenter();

        cout << "center: " << BoxAndCenter[0] << BoxAndCenter[1] << BoxAndCenter[2] <<endl;

        if (identifyWallofShape(BoxAndCenter)) continue;

        //semantic information
        int SegMask = static_cast<int>(pixel);
        cout << "segresult: " << SegMask << endl;

        if (AddObjectPointcloud(SegMask, BoxAndCenter)) {
            cout << "ADD " << endl; 
        }
        // sensor_msgs::PointCloud2 cloud_msg;
        // pcl::toROSMsg(*pcl_cloud, cloud_msg);               //pcl to ROSMSG:pointcloud2
        // cloud_msg.header.stamp = ros::Time::now();
        // cloud_msg.header.frame_id = "map";   // 设置坐标系
        // pointcloud_pub.publish(cloud_msg);   //发布PointCloud2消息

        pcl_cloud->clear();
        
    }
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "is" << duration.count() << "ms" << std::endl;

    size_t num_entries = cloud_map.size();
    cout << "num_entries: " << num_entries << endl;
    
}



void pointcloudMergeVisualization(ros::NodeHandle& nh) {
    PointCloud::Ptr merged_cloud(new PointCloud);
    for (const auto& entry : cloud_map) {
        *merged_cloud += *(entry.second);
    }
    sensor_msgs::PointCloud2 merged_cloud_msg;
    pcl::toROSMsg(*merged_cloud, merged_cloud_msg);               //pcl to ROSMSG:pointcloud2
    merged_cloud_msg.header.stamp = ros::Time::now();
    merged_cloud_msg.header.frame_id = "map";   // 设置坐标系
    pointcloud_pub.publish(merged_cloud_msg);   //发布PointCloud2消息
}

void generateMarkers_points(ros::Publisher& marker_pub) {
    visualization_msgs::MarkerArray marker_array;

    int marker_id = 0; 

    std::map<int, std::array<float, 3>> color_map;

    color_map[3] = {1,0,0};
    color_map[6] = {0,1,0};
    color_map[7] = {0,0,1};
    color_map[9] = {188.0/255,189.0/255,34.0/255};
    color_map[14] = {197.0/255,176.0/255,213.0/255};
    color_map[15] = {255.0/255,255.0/255,0};
    color_map[25] = {153.0/255,63.0/255,0};
    color_map[35] = {206.0/255,0/255,44.0/255};
    color_map[38] = {255.0/255,121.0/255,131.0/255};
    color_map[39] = {140.0/255,57.0/255,197.0/255};
    color_map[40] = {223.0/255,49.0/255,230.0/255};
    color_map[5] = {44.0/255,160.0/255,44.0/255};
    color_map[22] = {255.0/255,182.0/255,210.0/255};
    color_map[18] = {82.0/255,84.0/255,163.0/255};

    for (const auto& entry : cloud_map) {
        const PointCloud::Ptr& original_point_cloud_ptr = entry.second;
        PointCloud::Ptr point_cloud_ptr(new PointCloud);
        // for (size_t i = 0; i < point_cloud_ptr->size(); ++i){
        //     (*point_cloud_ptr)[i].z = 0;
        // }
        *point_cloud_ptr = *original_point_cloud_ptr;

        for (size_t i = 0; i < point_cloud_ptr->size(); ++i){
            (*point_cloud_ptr)[i].z = 0;
        }


        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // Set the appropriate frame id
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = color_map[entry.first.first][0];
        marker.color.g = color_map[entry.first.first][1];
        marker.color.b = color_map[entry.first.first][2];
        marker.id = marker_id++;



        for (const auto& point : point_cloud_ptr->points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
}

void generateMarkers(ros::Publisher& marker_pub) {
    visualization_msgs::MarkerArray marker_array;

    pcl::ConvexHull<PointT> chull;

    int marker_id = 0; // 用于唯一标识每个marker

    // std::map<int, std_msgs::ColorRGBA> color_map;
    // color_map[0] = getColor(0,0,0);

    std::map<int, std::array<float, 3>> color_map;

    color_map[3] = {1,0,0};
    color_map[6] = {0,1,0};
    color_map[7] = {0,0,1};
    color_map[9] = {188.0/255,189.0/255,34.0/255};
    color_map[14] = {197.0/255,176.0/255,213.0/255};
    color_map[15] = {255.0/255,255.0/255,0};
    color_map[25] = {153.0/255,63.0/255,0};
    color_map[35] = {206.0/255,0/255,44.0/255};
    color_map[38] = {255.0/255,121.0/255,131.0/255};
    color_map[39] = {140.0/255,57.0/255,197.0/255};
    color_map[40] = {223.0/255,49.0/255,230.0/255};
    color_map[5] = {44.0/255,160.0/255,44.0/255};
    color_map[22] = {255.0/255,182.0/255,210.0/255};
    color_map[18] = {82.0/255,84.0/255,163.0/255};


    for (const auto& entry : cloud_map) {
        const PointCloud::Ptr& original_point_cloud_ptr = entry.second;

        PointCloud::Ptr point_cloud_ptr(new PointCloud); // Create a copy of the original point cloud
        *point_cloud_ptr = *original_point_cloud_ptr; // Copy the point cloud data       

        PointCloud::Ptr cloud_hull(new PointCloud); // 用于存储凸包的点云数据

        for (size_t i = 0; i < point_cloud_ptr->size(); ++i){
            (*point_cloud_ptr)[i].z = 0;
        }

        chull.setInputCloud(point_cloud_ptr);
        chull.reconstruct(*cloud_hull); // 生成凸包

        // 创建一个新的marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.04;
        marker.color.r = color_map[entry.first.first][0];
        marker.color.g = color_map[entry.first.first][1];
        marker.color.b = color_map[entry.first.first][2];
        cout << marker.color.r << marker.color.g << marker.color.b << endl;
        marker.color.a = 1.0;
        marker.id = marker_id++; // 设置marker的唯一ID
        // cout << color_map[entry.first.first][0] << endl;
        // 根据语义编码设置颜色
        // if (color_map.find(entry.first.first) != color_map.end()) {
        //     marker.color.r = color_map[entry.first.first][0]/255;
        //     marker.color.g = color_map[entry.first.first][1]/255;
        //     marker.color.b = color_map[entry.first.first][2]/255;
        // } else {
        //     // Default color if code not found in the map
        //     marker.color.r = 0;
        //     marker.color.g = 0;
        //     marker.color.b = 1;
        // }
        // 将凸包的边缘添加到marker中
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

        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array); // 发布marker数组
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_generation");
    ros::NodeHandle n;

    ros::Subscriber color_sub = n.subscribe("/xtion/rgb/image_raw", 1, imageCallback);              //创建rgb图像接收者
    ros::Subscriber depth_sub = n.subscribe("/xtion/depth_registered/image_raw", 1, depthCallback);   //创建depth图像接收者

    pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_topic", 1000);                    //创建pointcloud2消息的发布者
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    // 初始化TF2监听器
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate wait(5);            
    wait.sleep();               //wait all MSG read

    while (ros::ok()) {

        ros::spinOnce();
        
        sendImgReseiveSeg();

        saveStuffPointcloud();

        pointcloudMergeVisualization(n);

        generateMarkers(marker_pub);

    }

    return 0;
}

