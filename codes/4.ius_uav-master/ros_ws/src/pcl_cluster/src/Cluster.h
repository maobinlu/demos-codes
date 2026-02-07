#pragma once
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>     //实现直通滤波

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>  //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>  //基于采样一致性分割的类的头文件
#include <pcl/visualization/pcl_visualizer.h>   // 可视化

using namespace std;
typedef pcl::PointXYZ PointT;

class Clustering
{
    public:
    void cluster(pcl::PointCloud<PointT>::Ptr cloud);
    pcl::PCDReader reader;                                    //PCD文件读取对象
    pcl::PassThrough<PointT> pass;                            //直通滤波对象
    pcl::NormalEstimation<PointT, pcl::Normal> ne;            //法线估计对象
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //分割对象
    pcl::PCDWriter writer;                                    //PCD文件读取对象
    pcl::ExtractIndices<PointT> extract;                      //点提取对象
    pcl::ExtractIndices<pcl::Normal> extract_normals;         ///点提取对象
};