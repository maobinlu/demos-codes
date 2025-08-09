#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Cluster.h"
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <fstream>

Clustering clusteropt;
ros::Publisher pub_cylinder;

void pub_cylinder_model(pcl::ModelCoefficients::Ptr cloud, int i)   //点云在rviz中可视化
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "mid360";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cylinder";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
     // POINTS markers use x and y scale for width/height respectively
    marker.pose.position.x = cloud->values[0];
    marker.pose.position.y = cloud->values[1];
    marker.pose.position.z = cloud->values[2];

    marker.pose.orientation.x = cloud->values[3];
    marker.pose.orientation.y = cloud->values[4];
    marker.pose.orientation.z = cloud->values[5];
    marker.pose.orientation.w = cloud->values[6];

    marker.scale.x = 0.5; // diameter
    marker.scale.y = 0.5; // diameter
    marker.scale.z = 1;     // height

    // Set the color of the marker
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    pub_cylinder.publish(marker);
 
}

void scan_callback(const sensor_msgs::PointCloud2ConstPtr& input)   //读取话题中的点云信息
{
  //将 sensor_msgs/PointCloud2 数据类型转为 pcl/PointCloud 方便转成pcd文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);//cloud is the output
    clusteropt.cluster(cloud);
    pcl::io::savePCDFileASCII ("/home/host/ius_uav/ros_ws/src/pcl_cluster/data/origin.pcd", *cloud);
}

void Clustering::cluster(pcl::PointCloud<PointT>::Ptr cloud)     //对点云进行圆柱体分割（聚类）
{
    printf("############# start clustering ############# \n");
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_pass(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    pass.setInputCloud(cloud);           //将无人机模型从点云数据中过滤掉
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.1,0.1);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.1,0.1);
    pass.setFilterLimitsNegative (true);   //过滤掉范围内的点云
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);   //缩小待识别的范围，提高精确程度
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0,4);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2,2);
    pass.setFilterLimitsNegative (false);
    pass.filter(*cloud_pass);
    std::cerr << "PointCloud after filtering has: " << cloud_pass->points.size() << " data points." << std::endl;
    writer.write("/home/host/ius_uav/ros_ws/src/pcl_cluster/data/after_pass.pcd", *cloud_pass, false);

    // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_pass);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    //  //Create the segmentation object for the planar model and set all the parameters
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setNormalDistanceWeight(0.1);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);
    // seg.setDistanceThreshold(0.03);
    // seg.setInputCloud(cloud_pass);
    // seg.setInputNormals(cloud_normals);
    // // //获取平面模型的系数和处在平面的内点
    // seg.segment(*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // //从点云中抽取分割的处在平面上的点集
    // extract.setInputCloud(cloud_pass);
    // extract.setIndices(inliers_plane);
    // extract.setNegative(false);
    
    // //存储分割得到的平面上的点到点云文件
    // pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    // extract.filter(*cloud_plane);
    // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    // //移除平面并保留平面以外的点
    // extract.setNegative(true);
    // extract.filter(*cloud_filtered2);
    // extract_normals.setNegative(true);
    // extract_normals.setInputCloud(cloud_normals);
    // extract_normals.setIndices(inliers_plane);
    // extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);        //设置对估计模型优化
    seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱形
    seg.setMethodType(pcl::SAC_RANSAC);       //参数估计方法
    seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
    seg.setMaxIterations(10000);              //设置迭代的最大次数10000
    seg.setDistanceThreshold(0.005);           //设置内点到模型的距离允许最大值
    seg.setRadiusLimits(0, 0.1);              //设置估计出的圆柱模型的半径的范围

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cylinder_clouds;
    std::vector<pcl::ModelCoefficients::Ptr> cylinder_coefficients;
   
    // int i = 0;
    // int num_iterations = 0;
    // int max_iterations = 100;
    // while (cloud_pass->points.size() > 100 && num_iterations < max_iterations) {
        seg.setInputCloud(cloud_pass);
        seg.setInputNormals(cloud_normals);
        
        // Obtain the cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        extract.setInputCloud(cloud_pass);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*cloud_cylinder);
        
        if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
        else{
            std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
            writer.write("/home/host/ius_uav/ros_ws/src/pcl_cluster/data/cylinder.pcd", *cloud_cylinder, false);

            if (abs(coefficients_cylinder->values[0]) < 1 && abs(coefficients_cylinder->values[1]) < 1 && 0 < coefficients_cylinder->values[2] && coefficients_cylinder->values[2] < 1.2) {
                pub_cylinder_model(coefficients_cylinder);
            }
    printf("############# end clustering ############# \n");
        }

        // extract.setNegative(true);
        // extract.filter(*cloud_pass);
        // extract_normals.setNegative(true);
        // extract_normals.setInputCloud(cloud_normals);
        // extract_normals.setIndices(inliers_cylinder);
        // extract_normals.filter(*cloud_normals);
        // num_iterations++;
        // printf("############# end phase ############# \n");
    // }
   


}


int main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/mid360_scan2",10,scan_callback);
  pub_cylinder = nh.advertise<visualization_msgs::Marker>("cylinder_model",10);
 
// Spin
  ros::spin ();
  return 0;
}
 