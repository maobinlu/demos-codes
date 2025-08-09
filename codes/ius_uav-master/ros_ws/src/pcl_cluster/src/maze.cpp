#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Cluster.h"
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <algorithm>

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

    pass.setInputCloud(new_cloud);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1,3);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("x");
    pass.setFilterLimits(20,30);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud_pass);

    pass.setInputCloud(cloud_pass);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-15,-7);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud);
    double max_x[10] = {0};
    double y[10];
    double dz = 0.2;

    for(int i=0; i<cloud->points.size(); i++)
    {
        for(int a=0; a<10; a++)
        {
            if (dz * a + 1.0 < cloud->points[i].z < dz*(a+1) + 1.0)
            {
                if (cloud->points[i].x > max_x[a])
                {
                    max_x[a] = cloud->points[i].x;
                    y[a] = cloud->points[i].y;
                }
            }
        }
    }

    sort(max_x, max_x+10);
    sort(y, y+10);
    auto ave_x = (max_x[4]+max_x[5]) / 2.0;
    auto ave_y = (y[4]+y[5]) / 2.0;

    ave_x = ave_x - 0.9;

     visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clusters";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置点的位置
    marker.pose.position.x = ave_x;
    marker.pose.position.y = ave_y;
    marker.pose.position.z = 1;
    
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

int main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud_registered_effect_world",10,scan_callback);
  marker_pub = nh.advertise<visualization_msgs::Marker>("cylinder_model",10);
 
// Spin
  ros::spin ();
  return 0;
}
 