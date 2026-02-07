#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Cluster.h"
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <random>
#include <std_msgs/ColorRGBA.h>

ros::Publisher marker_pub;


// 定义点的维度
const int DIMENSION = 2;

// 定义点类
class Point {
public:
    std::vector<double> coordinates;
    int clusterId;
    std_msgs::ColorRGBA color; // Add this line
    int persistentId;

    Point() : clusterId(-1) {
        coordinates.resize(DIMENSION);
    }
};

// 计算两点之间的欧氏距离
double distance(const Point& p1, const Point& p2) {
    double dist = 0.0;
    for (int i = 0; i < DIMENSION; ++i) {
        dist += (p1.coordinates[i] - p2.coordinates[i]) * (p1.coordinates[i] - p2.coordinates[i]);
    }
    return std::sqrt(dist);
}

void initializeKMeansPlusPlus(std::vector<Point>& points, std::vector<Point>& centroids, int numClusters) {
    centroids.resize(numClusters);

    // 随机选择第一个聚类中心
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_int_distribution<int> distribution(0, points.size() - 1);
    int firstCentroidIndex = distribution(generator);
    centroids[0] = points[firstCentroidIndex];

    // 为每个聚类中心选择新的中心
    for (int i = 1; i < numClusters; ++i) {
        double totalDistance = 0.0;
        std::vector<double> distances(points.size());

        for (size_t j = 0; j < points.size(); ++j) {
            double minDist = std::numeric_limits<double>::max();
            for (int k = 0; k < i; ++k) {
                double dist = distance(points[j], centroids[k]);
                if (dist < minDist) {
                    minDist = dist;
                }
            }
            distances[j] = minDist;
            totalDistance += minDist;
        }

        // 根据距离选择新的聚类中心
        std::uniform_real_distribution<double> probDistribution(0.0, totalDistance);
        double prob = probDistribution(generator);
        for (size_t j = 0; j < points.size(); ++j) {
            prob -= distances[j];
            if (prob <= 0.0) {
                centroids[i] = points[j];
                break;
            }
        }
    }
}

// K-Means 算法
void kMeans(std::vector<Point>& points, int numClusters) {
    std::vector<Point> centroids;
    initializeKMeansPlusPlus(points, centroids, numClusters);
    int numPoints = points.size();
    // 迭代更新聚类中心
    const int maxIterations = 1000;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // 分配每个点到最近的聚类中心
        for (auto& point : points) {
            double minDist = std::numeric_limits<double>::max();
            int clusterId = 0;

            for (int i = 0; i < numClusters; ++i) {
                double dist = distance(point, centroids[i]);
                if (dist < minDist) {
                    minDist = dist;
                    clusterId = i;
                }
            }

            // 更新点的聚类
            point.clusterId = clusterId;
        }

        // 更新聚类中心为各聚类内点的均值
        for (int i = 0; i < numClusters; ++i) {
            Point sum;
            int numPointsInCluster = 0;
            for (const auto& point : points) {
                if (point.clusterId == i) {
                    for (int d = 0; d < DIMENSION; ++d) {
                        sum.coordinates[d] += point.coordinates[d];
                    }
                    ++numPointsInCluster;
                }
            }
            if (numPointsInCluster > 0) {
                for (int d = 0; d < DIMENSION; ++d) {
                    centroids[i].coordinates[d] = sum.coordinates[d] / numPointsInCluster;
                }
            }
        }
    }
}

void fill_callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // 将 ROS 中的点云数据填充到 points 向量中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_pass(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_pass2(new pcl::PointCloud<PointT>);
    pcl::PCDWriter writer;  
    pcl::PassThrough<PointT> pass;  
    int count[4] = {0,0,0,0};
    std::vector<std::vector<float>> pillar_sum(4, std::vector<float>(3));
    std::vector<std::vector<float>> pillar_ave(4, std::vector<float>(3));
    
    // 填充点云数据到 cloud 中
    // ...
    pcl::fromROSMsg (*input, *cloud);//cloud is the output
    // pass.setInputCloud(cloud);           //将无人机模型从点云数据中过滤掉
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-0.1,0.1);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-0.1,0.1);
    // pass.setFilterLimitsNegative (true);   //过滤掉范围内的点云
    // pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.3,0.3);
    pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);
    
    pass.setInputCloud(cloud_filtered);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5,0);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud_pass);


    pass.setInputCloud(cloud_pass);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("x");
    pass.setFilterLimits(7,15);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud_pass2);
    writer.write("/home/host/ius_uav/ros_ws/src/pcl_cluster/data/pass.pcd", *cloud_pass2, false);
    
    std::vector<Point> points(cloud_pass2->size());
    for (size_t i = 0; i < cloud_pass2->size(); ++i) {
        points[i].coordinates[0] = cloud_pass2->points[i].x;
        points[i].coordinates[1] = cloud_pass2->points[i].y;
        // points[i].coordinates[2] = cloud_pass2->points[i].z;
        points[i].persistentId = i; // Set the persistent ID
    }

    // 调用 K-Means 算法
    int numClusters = 4;
    kMeans(points, numClusters);
    for (auto& point : points) {
        point.persistentId = point.clusterId;
    }

    // 定义一些颜色值（这里只定义了几种颜色）
std::vector<std_msgs::ColorRGBA> clusterColors(points.size());

// 使用循环为每个聚类设置颜色
if (int i = 0) {
    clusterColors[i].r = 1;
    clusterColors[i].g = 1;
    clusterColors[i].b = 1;
    clusterColors[i].a = 1.0;
}

if (int i = 1) {
    clusterColors[i].r = 1;
    clusterColors[i].g = 1;
    clusterColors[i].b = 1;
    clusterColors[i].a = 1.0;
}

if (int i = 2) {
    clusterColors[i].r = 1;
    clusterColors[i].g = 1;
    clusterColors[i].b = 1;
    clusterColors[i].a = 1.0;
}

if (int i = 3) {
    clusterColors[i].r = 1;
    clusterColors[i].g = 1;
    clusterColors[i].b = 1;
    clusterColors[i].a = 1.0;
}

    // 输出每个点的聚类结果
    for (const auto& point : points) {
    int persistentId = point.persistentId;
        int clusterId = point.clusterId;
        
        // 根据 persistentId 找到对应的点，并更新其聚类 ID
        for (auto& updatedPoint : points) {
            if (updatedPoint.persistentId == persistentId) {
                updatedPoint.clusterId = clusterId;
                break;
            }
        }
        count[point.persistentId]++;
        pillar_sum[point.persistentId][0] += point.coordinates[0];
        pillar_sum[point.persistentId][1] += point.coordinates[1];
        // pillar_sum[point.persistentId][2] += point.coordinates[2];
        std::cout << "Point (" << point.coordinates[0] << ", " << point.coordinates[1] << ") belongs to cluster " << point.persistentId << std::endl;
    }

    for (int i=0; i<4; i++){
        pillar_ave[i][0] = pillar_sum[i][0]/count[i];
        pillar_ave[i][1] = pillar_sum[i][1]/count[i];
        // pillar_ave[i][2] = pillar_sum[i][2]/count[i];
        std::cout << pillar_ave[i][0] << "," << pillar_ave[i][1] << std::endl;
    }   
    
    for (size_t i = 0; i < 4; ++i) {
    points[i].color = clusterColors[points[i].clusterId];
     visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clusters";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置点的位置
    marker.pose.position.x = pillar_ave[i][0];
    marker.pose.position.y = pillar_ave[i][1];
    marker.pose.position.z = 1;
    
    // 设置点的颜色
    marker.color = points[i].color;
    
    // 设置点的尺寸
    marker.scale.x = marker.scale.y = marker.scale.z = 0.5; // Adjust the size as needed
    
    // 发布Marker消息
    marker_pub.publish(marker);
}
        printf("############################################### \n");

    }

int main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud_registered_effect_world",5000,fill_callback);
  marker_pub = nh.advertise<visualization_msgs::Marker>("cylinder_model",5000);

 
// Spin
  ros::spin ();
  return 0;
}
 