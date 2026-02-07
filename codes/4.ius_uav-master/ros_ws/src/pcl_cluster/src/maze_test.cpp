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
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <Eigen/Dense>

ros::Publisher marker_pub;
pcl::PCDWriter writer;
class CloudFilter {
public:
    CloudFilter(float x_min, float x_max,
                float y_min, float y_max,
                float z_min, float z_max,
                int downfrequency = 2) {
        x_min_ = x_min;
        x_max_ = x_max;
        y_min_ = y_min;
        y_max_ = y_max;
        z_min_ = z_min;
        z_max_ = z_max;
        downfrequency_ = downfrequency;
        reset();
    }
    void reset() {
        cloud_buffer_.points.clear();
        cloud_filtered_.points.clear();
        input_count_ = 0;
    }

    const pcl::PointCloud<pcl::PointXYZ>& 
    filter(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
        input_count_++;
        if (input_count_ % downfrequency_ == 0) {
            cloud_filtered_ = cloud_buffer_;
            cloud_filtered_.header = cloud_in.header;
            cloud_buffer_.points.clear();
        }
        for (int i = 0; i < cloud_in.points.size(); i++) {
            if (cloud_in.points[i].x > x_min_ && cloud_in.points[i].x < x_max_ &&
                cloud_in.points[i].y > y_min_ && cloud_in.points[i].y < y_max_ &&
                cloud_in.points[i].z > z_min_ && cloud_in.points[i].z < z_max_) {
                cloud_buffer_.points.push_back(cloud_in.points[i]);
            }
        }
        return cloud_filtered_;
    }

private:
    pcl::PointCloud<pcl::PointXYZ> cloud_buffer_;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_;
    int input_count_;
    int downfrequency_;
    float x_min_, x_max_;
    float y_min_, y_max_;
    float z_min_, z_max_;
};

struct Plane
{
    Plane() {
        inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    }
    pcl::ModelCoefficients coefficients;
    pcl::PointCloud<pcl::PointXYZ> points;
    pcl::PointIndices::Ptr inliers;
};

// 点云平面提取
std::vector<pcl::shared_ptr<Plane>> plane_extract(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                   std::vector<pcl::shared_ptr<Plane>>& planes) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_points = 
                    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_in);
    int plane_count = 0;
    //
    int gate_coef_min = 10;
    int gate_b = 100;
    pcl::shared_ptr<Plane> plane_gate = pcl::make_shared<Plane>();
    //
    //
    int flank_coef_min = 10;
    int flank_b = 100;
    pcl::shared_ptr<Plane> plane_flank = pcl::make_shared<Plane>();
    //
    while (remaining_points->size() > 5) // 迭代拟合多个平面
    {
        pcl::shared_ptr<Plane> plane = pcl::make_shared<Plane>();
        seg.setInputCloud(remaining_points);
        seg.segment(*(plane->inliers), plane->coefficients);

        if (plane->inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // 提取平面内点和非平面内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining_points);
        extract.setIndices(plane->inliers);
        extract.setNegative(false);
        extract.filter(plane->points);

        extract.setNegative(true);
        extract.filter(*remaining_points);

        // 输出平面模型系数
        // std::cout << "Plane " << plane_count + 1 << " coefficients: ";
        // for (size_t i = 0; i < plane_gate->coefficients.values.size(); ++i)
        // {
        //     std::cout << plane_gate->coefficients.values[i] << " ";
        // }
        // std::cout << std::endl;

        ++plane_count;
        if(abs(plane->coefficients.values[0]) + abs(plane->coefficients.values[2]) < gate_coef_min){
            if(abs(plane->coefficients.values[3] / plane->coefficients.values[1]) < gate_b){
                gate_coef_min = abs(plane->coefficients.values[0]) + abs(plane->coefficients.values[2]);
                gate_b = abs(plane->coefficients.values[3] / plane->coefficients.values[1]);
                plane_gate = plane;
            } 
        }
        if(abs(plane->coefficients.values[1]) + abs(plane->coefficients.values[2]) < flank_coef_min){
            if(abs(plane->coefficients.values[3] / plane->coefficients.values[0]) < flank_b){
                flank_coef_min = abs(plane->coefficients.values[1]) + abs(plane->coefficients.values[2]);
                flank_b = abs(plane->coefficients.values[3] / plane->coefficients.values[0]);
                plane_flank = plane;
            } 
        }
    }
    std::vector<pcl::shared_ptr<Plane>> res = {plane_gate, plane_flank};
    return res;
}

CloudFilter C_cloud_filter( 20.0,  30.0,
                         -15.0, -7.0,
                          0.3,  2.0,
                         1);

void taskC_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    // 点云空间过滤
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered = C_cloud_filter.filter(cloud_in);
    // 输出过滤后点云信息
    // ROS_INFO("Cloud filtered size: %d", cloud_filtered.points.size());
    // 发布过滤后点云
    // sensor_msgs::PointCloud2 cloud_filtered_msg;
    // pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    // filter_cloud_pub.publish(cloud_filtered_msg);
    // 平面提取
    std::vector<pcl::shared_ptr<Plane>> planes;
    pcl::shared_ptr<Plane> plane_gate = plane_extract(cloud_filtered, planes)[0];
    pcl::shared_ptr<Plane> plane_flank = plane_extract(cloud_filtered, planes)[1];
    for (size_t i = 0; i < plane_gate->coefficients.values.size(); ++i)
    {
        // std::cout << plane_gate->coefficients.values[i] << " ";
    }
    // std::cout << std::endl;
    //
    int point_gate_num = plane_gate->points.size();
    int point_flank_num = plane_flank->points.size();
    if(point_gate_num > 5 && point_flank_num > 5)
    {
        Eigen::Matrix2f A;  // 系数矩阵
        Eigen::Vector2f b;  // 右侧常数向量

        A << plane_gate->coefficients.values[0], plane_gate->coefficients.values[1],
            plane_flank->coefficients.values[0], plane_flank->coefficients.values[1];

        b << -plane_gate->coefficients.values[3] - plane_gate->coefficients.values[2] * 1,
            -plane_flank->coefficients.values[3] - plane_flank->coefficients.values[2] * 1;

        Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);

        float x_start_edge = x[0] + 0.4 + 3.7;
        float y_start_edge = x[1];

        float x_mid_edge = x[0] + 0.5;
        float y_mid_edge = x[1] - 2.5;

        float x_end_edge = x_start_edge;
        float y_end_edge = y_start_edge - 5;

        std::cout << "x:"<< x_start_edge << ", y:" << y_start_edge << std::endl;

        visualization_msgs::Marker marker;
        for(int i = 0; i < 3; i++)
        {
        marker.header.frame_id = "camera_init";
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // 设置点的位置
        if(i == 0)
        {
        marker.pose.position.x = x_start_edge;
        marker.pose.position.y = y_start_edge;
        marker.pose.position.z = 1;
        }
        
        if(i == 1)
        {
        marker.pose.position.x = x_mid_edge;
        marker.pose.position.y = y_mid_edge;
        marker.pose.position.z = 1;
        }

        if(i == 2)
        {
        marker.pose.position.x = x_end_edge;
        marker.pose.position.y = y_end_edge;
        marker.pose.position.z = 1;
        }

        // 设置点的颜色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // 设置点的尺寸
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2; // Adjust the size as needed
        
        // 发布Marker消息
        marker_pub.publish(marker);  
        }
        
    }
    // int point_num = plane_gate->points.size();
    // std::cout << point_num << std::endl;

    // if(point_num > 30)
    // {
    //     double x_all[point_num];
    //     for(size_t i = 0; i < point_num; i++)
    //     {
    //         x_all[i] = plane_gate->points[i].x;
    //     }

    //     double x_max = *std::max_element(x_all, x_all + point_num);
    //     double x_min = *std::min_element(x_all, x_all + point_num);

    //     double width = 0.8;
    //     double dx = 0.1;
    //     int dx_num = (int)((x_max - x_min)  / dx + 1);

    //     int min_index = 0;
    //     int min_num;
    //     int wid_num = 8;

    //     int slice_num[dx_num] = {0};

    //     for(int i=0; i<point_num; i++)  // 把平面划分并记录各区域点数
    //     {
    //         int index = (int)((x_all[i] - x_min) / dx);
    //         slice_num[index]++;
    //     }

    //     // for(int i=0; i<dx_num; i++)
    //     // {
    //     //     std::cout << i << "  " << slice_num[i] << std::endl;
    //     // }

    //     int init_num;
    //     for(int i=0; i<wid_num; i++)  // 计算初始窗口内的点数
    //     {
    //         init_num += slice_num[i];
    //     }
    //     min_num = init_num;

    //     for(int i=0; i<dx_num-wid_num; i++)  // 滑动窗口，更新最小点数和索引
    //     {
    //         init_num = init_num - slice_num[i] + slice_num[i + wid_num];
    //         if (init_num < min_num)
    //         {
    //             min_num = init_num;
    //             min_index = i + 1;
    //         }
    //     }

    //     double x_right = x_min + dx * min_index;
    //     double x_mid = x_right + 0.4;
    //     double z_mid = 1;
    //     double y_mid = -(plane_gate->coefficients.values[3] + plane_gate->coefficients.values[0] * x_mid + plane_gate->coefficients.values[2] * z_mid) / plane_gate->coefficients.values[1]; 
        
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "camera_init";
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::SPHERE;
    //     marker.action = visualization_msgs::Marker::ADD;
        
    //     // 设置点的位置
    //     marker.pose.position.x = x_mid;
    //     marker.pose.position.y = y_mid;
    //     marker.pose.position.z = z_mid;
        
    //     // 设置点的颜色
    //     marker.color.r = 1.0;
    //     marker.color.g = 0.0;
    //     marker.color.b = 0.0;
    //     marker.color.a = 1.0;
        
    //     // 设置点的尺寸
    //     marker.scale.x = marker.scale.y = marker.scale.z = 0.2; // Adjust the size as needed
        
    //     // 发布Marker消息
    //     marker_pub.publish(marker);
    // }
    
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*cloud_msg, cloud_in);
    // 输出原始点云信息
    ROS_INFO("Cloud size: %d", cloud_in.points.size());

    // 任务A：柱子识别
    // taskA_perception(cloud_in);

    // 任务B：隧道识别
    // taskB_perception(cloud_in);

    // 任务C：迷宫门口识别
    taskC_perception(cloud_in);

    // 任务D：静态圆环识别

    // 任务E：动态圆环识别
}

int main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud_registered_effect_world",50,cloud_cb);
  marker_pub = nh.advertise<visualization_msgs::Marker>("maze_model",50);

 
// Spin
  ros::spin ();
  return 0;
}