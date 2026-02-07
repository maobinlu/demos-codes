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

CloudFilter B_cloud_filter( 18.0,  25.0,
                         -8.0, 0.5,
                          1.0,  4.0,
                         6);
void taskB_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
pcl::PointCloud<pcl::PointXYZ> cloud_filtered = B_cloud_filter.filter(cloud_in);
    // 输出过滤后点云信息
    ROS_INFO("Cloud filtered size: %d", cloud_filtered.points.size());
    // if(cloud_filtered.points.size() != 0)
    // writer.write("/home/host/ius_uav/ros_ws/src/pcl_cluster/data/pass.pcd", cloud_filtered, false);
    //中心点
    double min_x[10] = {100,100,100,100,100,100,100,100,100,100};
    double y[10];
    double dz = 0.3;

    for(int i=0; i<cloud_filtered.points.size(); i++)
    {
        for(int a=0;a<10;a++)
        {
            if (1 + dz * a < cloud_filtered.points[i].z < dz*(a+1) + 1)
            {
                if (cloud_filtered.points[i].x < min_x[a])
                {
                    min_x[a] = cloud_filtered.points[i].x;
                    y[a] = cloud_filtered.points[i].y;
                }
            }
        }
    }

    sort(min_x, min_x+10);
    sort(y, y+10);
    auto ave_min_x = (min_x[4]+min_x[5]) / 2.0;
    auto ave_y = (y[4]+y[5]) / 2.0;


    double max_y[10] = {-100,-100,-100,-100,-100,-100,-100,-100,-100,-100};
    double x[10];
    for(int i=0; i<cloud_filtered.points.size(); i++)
    {
        for(int a=0;a<10;a++)
        {
            if (1 + dz * a < cloud_filtered.points[i].z < dz*(a+1) + 1)
            {
                if (cloud_filtered.points[i].y > max_y[a])
                {
                    max_y[a] = cloud_filtered.points[i].y;
                    x[a] = cloud_filtered.points[i].x;
                }
            }
        }
    }

    sort(max_y, max_y+10);
    sort(y, y+10);
    auto ave_x = (x[4]+x[5]) / 2.0;
    auto ave_max_y = (max_y[4]+max_y[5]) / 2.0;
    double phi;
    phi = atan2(ave_x-ave_min_x,ave_max_y-ave_y);
    float x_start,y_start,x_end,y_end;
    x_start = (ave_min_x + ave_x)/2.0;
    y_start = (ave_y + ave_max_y)/2.0;
    x_end = x_start + 2*cos(phi);
    y_end = y_start - 2*sin(phi);

    
for(int i = 0;i < 2;i++)
    {
        visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clusters";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置点的位置
    if(i == 0){
    marker.pose.position.x = x_start;
    marker.pose.position.y = y_start;
    marker.pose.position.z = 1.5;
    }
    
if(i == 1){
    marker.pose.position.x = x_end;
    marker.pose.position.y = y_end;
    marker.pose.position.z = 1.5;
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

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*cloud_msg, cloud_in);
    // 输出原始点云信息
    ROS_INFO("Cloud size: %d", cloud_in.points.size());

    // 任务A：柱子识别
    taskB_perception(cloud_in);

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
  marker_pub = nh.advertise<visualization_msgs::Marker>("tunnel_model",50);

 
// Spin
  ros::spin ();
  return 0;
}