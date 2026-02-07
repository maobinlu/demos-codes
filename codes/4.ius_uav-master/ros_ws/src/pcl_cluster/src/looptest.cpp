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

ros::Publisher marker_pub;
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

CloudFilter D_cloud_filter( 17.0,  23.0,
                         -23.0, -17,
                          0.5,  4.0,
                         6);
void taskD_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
    // 点云空间过滤
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered = D_cloud_filter.filter(cloud_in);
    // 输出过滤后点云信息
    ROS_INFO("Cloud filtered size: %d", cloud_filtered.points.size());
    float sum_x,sum_y,ave_x,ave_y,sum_z,ave_z;
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    ave_x = 0;
    ave_y = 0;
    ave_z = 0;
    for(int i = 0;i < cloud_filtered.points.size();i++)
    {
       sum_x += cloud_filtered.points[i].x;
       sum_y += cloud_filtered.points[i].y;
       sum_z += cloud_filtered.points[i].z;
    }
    ave_x = sum_x/cloud_filtered.points.size();
    ave_y = sum_y/cloud_filtered.points.size();
    ave_z = sum_z/cloud_filtered.points.size();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置点的位置
    marker.pose.position.x = ave_x;
    marker.pose.position.y = ave_y;
    marker.pose.position.z = ave_z;
    
    // 设置点的颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    // 设置点的尺寸
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2; // Adjust the size as needed
    
    // 发布Marker消息
    marker_pub.publish(marker);
// }
      std::cout << "Point (" << ave_x << ", " << ave_y << ") " << std::endl;
        printf("############################################### \n");
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*cloud_msg, cloud_in);
    // 输出原始点云信息
    ROS_INFO("Cloud size: %d", cloud_in.points.size());

    // 任务A：柱子识别
    taskD_perception(cloud_in);

    // 任务B：隧道识别

    // 任务C：迷宫门口识别
    // taskC_perception(cloud_in);

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
  marker_pub = nh.advertise<visualization_msgs::Marker>("loop_model",50);

 
// Spin
  ros::spin ();
  return 0;
}