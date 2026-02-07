#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Cluster.h"
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <algorithm>
#include "math.h"
Clustering clusteropt;
ros::Publisher marker_pub;
void scan_callback(const sensor_msgs::PointCloud2ConstPtr& input)   //读取话题中的点云信息
{
  //将 sensor_msgs/PointCloud2 数据类型转为 pcl/PointCloud 方便转成pcd文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *new_cloud);//cloud is the output
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_pass(new pcl::PointCloud<PointT>);
     pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PCDWriter writer;  
    pcl::PassThrough<PointT> pass;  

    pass.setInputCloud(new_cloud);           //将无人机模型从点云数据中过滤掉
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.1,0.1);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.1,0.1);
    pass.setFilterLimitsNegative (true);   //过滤掉范围内的点云
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1,4);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud_pass);

    pass.setInputCloud(cloud_pass);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5,0);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud);
    writer.write("/home/host/ius_uav/ros_ws/src/pcl_cluster/data/pass.pcd", *cloud, false);
    double min_x[10] = {10,10,10,10,10,10,10,10,10,10};
    double y[10];
    double dz = 0.2;

    for(int i=0; i<cloud->points.size(); i++)
    {
        for(int a=0;a<10;a++)
        {
            if (dz * a < cloud->points[i].z < dz*(a+1))
            {
                if (cloud->points[i].x < min_x[a])
                {
                    min_x[a] = cloud->points[i].x;
                    y[a] = cloud->points[i].y;
                }
            }
        }
    }

    sort(min_x, min_x+10);
    sort(y, y+10);
    auto ave_min_x = (min_x[4]+min_x[5]) / 2.0;
    auto ave_y = (y[4]+y[5]) / 2.0;


    double max_y[10] = {-10,-10,-10,-10,-10,-10,-10,-10,-10,-10};
    double x[10];
    for(int i=0; i<cloud->points.size(); i++)
    {
        for(int a=0;a<10;a++)
        {
            if (dz * a < cloud->points[i].z < dz*(a+1))
            {
                if (cloud->points[i].y > max_y[a])
                {
                    max_y[a] = cloud->points[i].y;
                    x[a] = cloud->points[i].x;
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
    x_end = x_start + 1*cos(phi);
    y_end = y_start - 1*sin(phi);
    for(int i = 0;i < 2;i++)
    {
        visualization_msgs::Marker marker;
    marker.header.frame_id = "mid360";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clusters";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置点的位置
    if(i == 0){
    marker.pose.position.x = x_start;
    marker.pose.position.y = y_start;
    marker.pose.position.z = 1;
    }
    
if(i == 1){
    marker.pose.position.x = x_end;
    marker.pose.position.y = y_end;
    marker.pose.position.z = 1;
    }

    // 设置点的颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    // 设置点的尺寸
    marker.scale.x = marker.scale.y = marker.scale.z = 0.5; // Adjust the size as needed
    
    // 发布Marker消息
    marker_pub.publish(marker);
    }
     
}

int main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/mid360_scan2",10,scan_callback);
  marker_pub = nh.advertise<visualization_msgs::Marker>("cylinder_model",10);
 
// Spin
  ros::spin ();
  return 0;
}
 