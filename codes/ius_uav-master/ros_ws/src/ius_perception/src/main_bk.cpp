#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ius_msgs/PillarPos.h>
#include <ius_msgs/TunnelPos.h>
#include <ius_msgs/LoopPos.h>

ros::Subscriber cloud_sub;
ros::Publisher filter_cloud_pub;
ros::Publisher pillar_pos_pub;
//
ros::Publisher tunnel_pos_pub;
ros::Publisher loop_pos_pub;
// 点云空间过滤器，过滤一定范围内的点云
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

// 空间点流聚类滤波器
class CloudClusterFilter {
public:
    struct ClusterPoint {
        float x;
        float y;
        int points_num;
    };
    
    CloudClusterFilter(float radius, int min_points) {
        radius_ = radius;
        min_points_ = min_points;
    }
    
    void reset() {
        cluster_points_.clear();
    }

    void cluster(double x, double y) {
        for (int i = 0; i < cluster_points_.size(); i++) {
            if (distance(x, y, cluster_points_[i].x, cluster_points_[i].y) < radius_) {
                // cluster_points_[i].x = (cluster_points_[i].x * cluster_points_[i].points_num + x) / (cluster_points_[i].points_num + 1);
                // cluster_points_[i].y = (cluster_points_[i].y * cluster_points_[i].points_num + y) / (cluster_points_[i].points_num + 1);
                cluster_points_[i].x = (cluster_points_[i].x * 0.99 + x * 0.01);
                cluster_points_[i].y = (cluster_points_[i].y * 0.99 + y * 0.01);
                cluster_points_[i].points_num++;
                return;
            }
        }
        ClusterPoint cluster_point;
        cluster_point.x = x;
        cluster_point.y = y;
        cluster_point.points_num = 1;
        cluster_points_.push_back(cluster_point);
    }

    void get_first_n_cluster_points(int n, std::vector<ClusterPoint>& cluster_points) {
        cluster_points.clear();
        for (int i = 0; i < cluster_points_.size(); i++) {
            if (cluster_points_[i].points_num >= min_points_) {
                cluster_points.push_back(cluster_points_[i]);
            }
        }
        if (cluster_points.size() > n) {
            std::sort(cluster_points.begin(), cluster_points.end(), [](const ClusterPoint& a, const ClusterPoint& b) {
                return a.points_num > b.points_num;
            });
            cluster_points.resize(n);
        }
    }
    
private:
    float radius_;
    int min_points_;

    std::vector<ClusterPoint> cluster_points_;
    
    double inline distance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }
};

CloudFilter A_cloud_filter( 3.0,  13.0,
                         -3.0, 0.5,
                          0.0,  4.0,
                         1);
CloudClusterFilter cloud_cluster_filter(0.5, 5);

void taskA_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    // 点云空间过滤
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered = A_cloud_filter.filter(cloud_in);
    // 输出过滤后点云信息
    ROS_INFO("Cloud filtered size: %d", cloud_filtered.points.size());
    // 发布过滤后点云
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    filter_cloud_pub.publish(cloud_filtered_msg);
    // 空间点流聚类
    // cloud_cluster_filter.reset();
    for (int i = 0; i < cloud_filtered.points.size(); i++) {
        cloud_cluster_filter.cluster(cloud_filtered.points[i].x, cloud_filtered.points[i].y);
    }
    std::vector<CloudClusterFilter::ClusterPoint> cluster_points;
    cloud_cluster_filter.get_first_n_cluster_points(4, cluster_points);
    for (int i = 0; i < cluster_points.size(); i++) {
        ROS_INFO("Cluster point %d: (%f, %f), points num: %d", i, cluster_points[i].x, cluster_points[i].y, cluster_points[i].points_num);
    }
    
    // 按照x坐标排序
    std::sort(cluster_points.begin(), cluster_points.end(), [](const CloudClusterFilter::ClusterPoint& a, const CloudClusterFilter::ClusterPoint& b) {
        return a.x < b.x;
    });
    
    // 发布柱子位置
    ius_msgs::PillarPos pillar_pos_msg;
    pillar_pos_msg.header.frame_id = "world";
    pillar_pos_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < cluster_points.size(); i++) {
        pillar_pos_msg.pillar_id = i + 1;
        pillar_pos_msg.pillar_pos.x = cluster_points[i].x;
        pillar_pos_msg.pillar_pos.y = cluster_points[i].y;
        pillar_pos_msg.pillar_pos.z = 1;
        pillar_pos_pub.publish(pillar_pos_msg);
    }
}

CloudFilter B_cloud_filter( 15.0,  25.0,
                         -6.0, 0.5,
                          1.0,  4.0,
                         3);
void taskB_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
pcl::PointCloud<pcl::PointXYZ> cloud_filtered = B_cloud_filter.filter(cloud_in);
    // 输出过滤后点云信息
    ROS_INFO("Cloud filtered size: %d", cloud_filtered.points.size());
    // 发布过滤后点云
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    filter_cloud_pub.publish(cloud_filtered_msg);
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

    std::sort(min_x, min_x + 10);
    std::sort(y, y + 10);
    auto ave_min_x = (min_x[4] + min_x[5]) / 2.0;
    auto ave_y = (y[4] + y[5]) / 2.0;


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

    std::sort(max_y, max_y+10);
    std::sort(y, y+10);
    auto ave_x = (x[4]+x[5]) / 2.0;
    auto ave_max_y = (max_y[4]+max_y[5]) / 2.0;
    double phi;
    phi = atan2(ave_x-ave_min_x,ave_max_y-ave_y);
    float x_start,y_start,x_end,y_end;
    x_start = (ave_min_x + ave_x)/2.0;
    y_start = (ave_y + ave_max_y)/2.0;
    x_end = x_start + 2*cos(phi);
    y_end = y_start - 2*sin(phi);

    //发布隧道的前后中心位置
    ius_msgs::TunnelPos tunnel_pos_msg;
    tunnel_pos_msg.header.frame_id = "world";
    tunnel_pos_msg.header.stamp = ros::Time::now();
    
        tunnel_pos_msg.tunnel_pos_in.x = x_start;
        tunnel_pos_msg.tunnel_pos_in.y = y_start;
        tunnel_pos_msg.tunnel_pos_in.z = 1.5;

        tunnel_pos_msg.tunnel_pos_out.x = x_end;
        tunnel_pos_msg.tunnel_pos_out.y = y_end;
        tunnel_pos_msg.tunnel_pos_out.z = 1.5;
        
        tunnel_pos_pub.publish(tunnel_pos_msg);
}

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
void plane_extract(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                   std::vector<pcl::shared_ptr<Plane>>& planes) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_points = 
                    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_in);
    int plane_count = 0;

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
        std::cout << "Plane " << plane_count + 1 << " coefficients: ";
        for (size_t i = 0; i < plane->coefficients.values.size(); ++i)
        {
            std::cout << plane->coefficients.values[i] << " ";
        }
        std::cout << std::endl;

        ++plane_count;
    }
}

CloudFilter C_cloud_filter( 20.0,  30.0,
                         -15.0, -7.0,
                          0.0,  4.0,
                         1);

void taskC_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    // 点云空间过滤
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered = C_cloud_filter.filter(cloud_in);
    // 输出过滤后点云信息
    ROS_INFO("Cloud filtered size: %d", cloud_filtered.points.size());
    // 发布过滤后点云
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    filter_cloud_pub.publish(cloud_filtered_msg);
    // 平面提取
    std::vector<pcl::shared_ptr<Plane>> planes;
    plane_extract(cloud_filtered, planes);
}

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
    
    // 发布圆环位置
    ius_msgs::LoopPos loop_pos_msg;
    loop_pos_msg.header.frame_id = "world";
    loop_pos_msg.header.stamp = ros::Time::now();
    
    loop_pos_msg.loop_id = 1;
    loop_pos_msg.loop_pos.x = ave_x;
    loop_pos_msg.loop_pos.y = ave_y;
    loop_pos_msg.loop_pos.z = 1;
    loop_pos_pub.publish(loop_pos_msg);
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
    // taskA_perception(cloud_in);

    // 任务B：隧道识别
    taskB_perception(cloud_in);

    // 任务C：迷宫门口识别
    // taskC_perception(cloud_in);

    // 任务D：静态圆环识别

    // 任务E：动态圆环识别
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ius_perception");
    ros::NodeHandle nh;
    
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 1, cloud_cb);
    filter_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
    pillar_pos_pub = nh.advertise<ius_msgs::PillarPos>("/ius_uav/pillar_pos", 50);
    tunnel_pos_pub = nh.advertise<ius_msgs::TunnelPos>("/ius_uav/tunnel_pos", 50);
    loop_pos_pub = nh.advertise<ius_msgs::LoopPos>("/ius_uav/loop_pos", 50);


    ros::spin();
    return 0;
}
