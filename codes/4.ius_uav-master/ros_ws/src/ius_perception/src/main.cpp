#include <vector>
#include <deque>
#include <limits>
#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <ius_msgs/PillarPos.h>
#include <ius_msgs/TunnelPos.h>
#include <ius_msgs/LoopPos.h>
#include <ius_msgs/MazePos.h>

#include "ius_perception/mission_param.h"

ros::Subscriber cloud_sub;
ros::Publisher filter_cloud_pub;
ros::Publisher mission_A_cloud_pub;
ros::Publisher mission_B_cloud_pub;
ros::Publisher mission_C_cloud_pub;
ros::Publisher mission_D_cloud_pub;
ros::Publisher mission_E_cloud_pub;
//
ros::Publisher pillar_pos_pub;
ros::Publisher tunnel_pos_pub;
ros::Publisher maze_pos_pub;
ros::Publisher loop_pos_pub;

MissionParam mission_param;

// 任务区点云分离与缓存
std::deque<pcl::PointCloud<pcl::PointXYZ>> mission_A_cloud_buffer;
size_t mission_A_buffer_size = 1;
size_t mission_A_cloud_points_num = 0;

std::deque<pcl::PointCloud<pcl::PointXYZ>> mission_B_task_cloud_buffer;
size_t mission_B_buffer_size = 100;
size_t mission_B_cloud_points_num = 0;

std::deque<pcl::PointCloud<pcl::PointXYZ>> mission_C_cloud_buffer;
size_t mission_C_buffer_size = 100;
size_t mission_C_cloud_points_num = 0;

std::deque<pcl::PointCloud<pcl::PointXYZ>> mission_D_cloud_buffer;
size_t mission_D_buffer_size = 100;
size_t mission_D_cloud_points_num = 0;

std::deque<pcl::PointCloud<pcl::PointXYZ>> mission_E_cloud_buffer;
size_t mission_E_buffer_size = 5;
size_t mission_E_cloud_points_num = 0;

pcl::VoxelGrid<pcl::PointXYZ> mission_B_voxel_grid_filter;
pcl::VoxelGrid<pcl::PointXYZ> mission_C_voxel_grid_filter;
pcl::VoxelGrid<pcl::PointXYZ> mission_D_voxel_grid_filter;
pcl::VoxelGrid<pcl::PointXYZ> mission_E_voxel_grid_filter;
pcl::PointCloud<pcl::PointXYZ> cloud_B_filtered;
pcl::PointCloud<pcl::PointXYZ> cloud_C_filtered;
pcl::PointCloud<pcl::PointXYZ> cloud_D_filtered;
pcl::PointCloud<pcl::PointXYZ> cloud_E_filtered;

inline bool is_point_in_range(const pcl::PointXYZ& point,
                              double x_min, double x_max,
                              double y_min, double y_max) {
    return point.x > x_min &&
           point.x < x_max && 
           point.y > y_min && 
           point.y < y_max &&
           point.z > 0.3 &&
           point.z < 4.0;
}

void cloud_mission_dispatch(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    pcl::PointCloud<pcl::PointXYZ> mission_A_cloud;
    pcl::PointCloud<pcl::PointXYZ> mission_B_cloud;
    pcl::PointCloud<pcl::PointXYZ> mission_C_cloud;
    pcl::PointCloud<pcl::PointXYZ> mission_D_cloud;
    pcl::PointCloud<pcl::PointXYZ> mission_E_cloud;
    for (int i = 0; i < cloud_in.points.size(); i++) {
        if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_pillar_x_down_, mission_param.mission_pillar_x_up_,
                              mission_param.mission_pillar_y_right_, mission_param.mission_pillar_y_left_)) {
            mission_A_cloud.points.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_tunnel_x_down_, mission_param.mission_tunnel_x_up_,
                              mission_param.mission_tunnel_y_right_, mission_param.mission_tunnel_y_left_)) {
            mission_B_cloud.points.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_maze_x_down_, mission_param.mission_maze_x_up_,
                              mission_param.mission_maze_y_right_, mission_param.mission_maze_y_left_)) {
            mission_C_cloud.points.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_loop_x_down_, mission_param.mission_loop_x_up_,
                              mission_param.mission_loop_y_right_, mission_param.mission_loop_y_left_)) {
            mission_D_cloud.points.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_m_loop_x_down_, mission_param.mission_m_loop_x_up_,
                              mission_param.mission_m_loop_y_right_, mission_param.mission_m_loop_y_left_)) {
            mission_E_cloud.points.push_back(cloud_in.points[i]);
        }
    }

    // 任务B点云滤波
    mission_B_cloud += cloud_B_filtered;
    mission_B_voxel_grid_filter.setInputCloud(mission_B_cloud.makeShared());
    mission_B_voxel_grid_filter.filter(cloud_B_filtered);
    // 任务C点云滤波
    mission_C_cloud += cloud_C_filtered;
    mission_C_voxel_grid_filter.setInputCloud(mission_C_cloud.makeShared());
    mission_C_voxel_grid_filter.filter(cloud_C_filtered);
    // 任务D点云滤波
    mission_D_cloud += cloud_D_filtered;
    mission_D_voxel_grid_filter.setInputCloud(mission_D_cloud.makeShared());
    mission_D_voxel_grid_filter.filter(cloud_D_filtered);
    // 任务E点云滤波
    // mission_E_cloud += cloud_E_filtered;
    // mission_E_voxel_grid_filter.setInputCloud(mission_E_cloud.makeShared());
    // mission_E_voxel_grid_filter.filter(cloud_E_filtered);

    if (!mission_A_cloud.empty()) {
        // ROS_INFO("Mission A frame cloud size: %d", mission_A_cloud.size());
        mission_A_cloud_points_num += mission_A_cloud.size();
        mission_A_cloud_buffer.push_back(std::move(mission_A_cloud));
    }
    if (!mission_B_cloud.empty()) {
        // ROS_INFO("Mission B frame cloud size: %d", mission_B_cloud.size());
        mission_B_cloud_points_num += mission_B_cloud.size();
        mission_B_task_cloud_buffer.push_back(std::move(mission_B_cloud));
    }
    if (!mission_C_cloud.empty()) {
        // ROS_INFO("Mission C frame cloud size: %d", mission_C_cloud.size());
        mission_C_cloud_points_num += mission_C_cloud.size();
        mission_C_cloud_buffer.push_back(std::move(mission_C_cloud));
    }
    if (!mission_D_cloud.empty()) {
        // ROS_INFO("Mission D frame cloud size: %d", mission_D_cloud.size());
        mission_D_cloud_points_num += mission_D_cloud.size();
        mission_D_cloud_buffer.push_back(std::move(mission_D_cloud));
    }
    if (!mission_E_cloud.empty()) {
        // ROS_INFO("Mission E frame cloud size: %d", mission_E_cloud.size());
        mission_E_cloud_points_num += mission_E_cloud.size();
        mission_E_cloud_buffer.push_back(std::move(mission_E_cloud));
    }

    while (mission_A_cloud_buffer.size() > mission_A_buffer_size ||
           mission_A_cloud_points_num > 100) {
           mission_A_cloud_points_num -= mission_A_cloud_buffer.front().size();
           mission_A_cloud_buffer.pop_front();
    }
    while (mission_B_task_cloud_buffer.size() > mission_B_buffer_size ||
           mission_B_cloud_points_num > 150) {
           mission_B_cloud_points_num -= mission_B_task_cloud_buffer.front().size();
           mission_B_task_cloud_buffer.pop_front();
    }
    while (mission_C_cloud_buffer.size() > mission_C_buffer_size ||
           mission_C_cloud_points_num > 200) {
           mission_C_cloud_points_num -= mission_C_cloud_buffer.front().size();
           mission_C_cloud_buffer.pop_front();
    }
    while (mission_D_cloud_buffer.size() > mission_D_buffer_size ||
           mission_D_cloud_points_num > 250) {
           mission_D_cloud_points_num -= mission_D_cloud_buffer.front().size();
           mission_D_cloud_buffer.pop_front();
    }
    while (mission_E_cloud_buffer.size() > mission_C_buffer_size ||
           mission_E_cloud_points_num > 100) {
           mission_E_cloud_points_num -= mission_E_cloud_buffer.front().size();
           mission_E_cloud_buffer.pop_front();
    }
}

// 点云缓存整合
void cloud_mission_buffer_merge(const std::deque<pcl::PointCloud<pcl::PointXYZ>>& cloud_buff,
                                pcl::PointCloud<pcl::PointXYZ>::VectorType& cloud_merge) {
    cloud_merge.clear();
    for (int i = 0; i < cloud_buff.size(); i++) {
        cloud_merge.insert(cloud_merge.end(), cloud_buff[i].begin(), cloud_buff[i].end());
    }
}

// 点云预处理
void cloud_preprocess(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_A,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_B,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_C,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_D,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_E) {
    cloud_A.points.clear();
    cloud_B.points.clear();
    cloud_C.points.clear();
    cloud_D.points.clear();
    cloud_E.points.clear();
    cloud_mission_dispatch(cloud_in);
    cloud_mission_buffer_merge(mission_A_cloud_buffer, cloud_A.points);
    cloud_mission_buffer_merge(mission_B_task_cloud_buffer, cloud_B.points);
    cloud_mission_buffer_merge(mission_C_cloud_buffer, cloud_C.points);
    cloud_mission_buffer_merge(mission_D_cloud_buffer, cloud_D.points);
    cloud_mission_buffer_merge(mission_E_cloud_buffer, cloud_E.points);

    ROS_INFO("Mission A buff cloud size: %d", cloud_A.points.size());
    ROS_INFO("Mission B buff cloud size: %d", cloud_B.points.size());
    ROS_INFO("Mission C buff cloud size: %d", cloud_C.points.size());
    ROS_INFO("Mission D buff cloud size: %d", cloud_D.points.size());
    ROS_INFO("Mission E buff cloud size: %d", cloud_E.points.size());

    // 发布任务区点云
    sensor_msgs::PointCloud2 cloud_A_msg;
    pcl::toROSMsg(cloud_A, cloud_A_msg);
    cloud_A_msg.header.frame_id = "world";
    cloud_A_msg.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 cloud_B_msg;
    pcl::toROSMsg(cloud_B_filtered, cloud_B_msg);
    cloud_B_msg.header.frame_id = "world";
    cloud_B_msg.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 cloud_C_msg;
    pcl::toROSMsg(cloud_C_filtered, cloud_C_msg);
    cloud_C_msg.header.frame_id = "world";
    cloud_C_msg.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 cloud_D_msg;
    pcl::toROSMsg(cloud_D_filtered, cloud_D_msg);
    cloud_D_msg.header.frame_id = "world";
    cloud_D_msg.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 cloud_E_msg;
    pcl::toROSMsg(cloud_E, cloud_E_msg);
    cloud_E_msg.header.frame_id = "world";
    cloud_E_msg.header.stamp = ros::Time::now();
    mission_A_cloud_pub.publish(cloud_A_msg);
    mission_B_cloud_pub.publish(cloud_B_msg);
    mission_C_cloud_pub.publish(cloud_C_msg);
    mission_D_cloud_pub.publish(cloud_D_msg);
    mission_E_cloud_pub.publish(cloud_E_msg);
}

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

    double inline distance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }
    
private:
    float radius_;
    int min_points_;

    std::vector<ClusterPoint> cluster_points_;
};

CloudClusterFilter cloud_cluster_filter(0.5, 5);

void taskA_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    // 空间点流聚类
    // cloud_cluster_filter.reset();
    for (int i = 0; i < cloud_in.points.size(); i++) {
        cloud_cluster_filter.cluster(cloud_in.points[i].x, cloud_in.points[i].y);
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

struct Value {
    double value = std::numeric_limits<double>::max();
    int index = -1;
};
// 寻找垂直的边界xy坐标
void find_xy_boundary_points(const pcl::PointCloud<pcl::PointXYZ>::VectorType& cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>::VectorType& boundary_points,
                             const std::function<double(double, double)>& value_func) {
    // 沿z轴切片[0.5 : 0.05 : 3.5) = int [0 : 59]
    std::vector<Value> min_x_values(60, Value());
    for (int i = 0; i < cloud_in.size(); i++) {
        int index = (int)((cloud_in[i].z - 0.5) / 0.05);
        if (index < 0 || index >= 60) {
            continue;
        }
        double value = value_func(cloud_in[i].x, cloud_in[i].y);
        if (value < min_x_values[index].value) {
            min_x_values[index].value = value;
            min_x_values[index].index = i;
        }
    }
    for (auto& value : min_x_values) {
        if (value.index != -1) {
            boundary_points.push_back(cloud_in[value.index]);
        }
    }
}

// 寻找水平面上的z的最大点
void find_max_z_points(const pcl::PointCloud<pcl::PointXYZ>::VectorType& cloud_in,
                       double x_min, double x_max, double y_min, double y_max,
                       std::vector<double>& max_z_points) {
    max_z_points.clear();
    // 沿水平面切片
    double delta = 0.3;
    int x_num = (x_max - x_min) / delta;
    int y_num = (y_max - y_min) / delta;
    std::unordered_map<int, Value> max_z_values;
    for (int i = 0; i < cloud_in.size(); i++) {
        int x_index = (int)((cloud_in[i].x - x_min) / delta);
        int y_index = (int)((cloud_in[i].y - y_min) / delta);
        int index = x_index * y_num + y_index;
        if (index < 0 || index >= x_num * y_num) {
            continue;
        }
        if (max_z_values.find(index) == max_z_values.end() ||
            cloud_in[i].z > max_z_values[index].value) {
            max_z_values[index].value = cloud_in[i].z;
            max_z_values[index].index = i;
        }
    }
    for (auto& value : max_z_values) {
        if (value.second.index != -1) {
            max_z_points.push_back(cloud_in[value.second.index].z);
        }
    }
}

CloudClusterFilter B_boundary1_filter(0.2, 5);
CloudClusterFilter B_boundary2_filter(0.2, 5);
CloudClusterFilter B_zz_filter(0.2, 5);
void taskB_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
    pcl::PointCloud<pcl::PointXYZ>::VectorType boundary1_points; // x + y
    pcl::PointCloud<pcl::PointXYZ>::VectorType boundary2_points; // -x + y
    std::vector<double> max_z_points;
    find_xy_boundary_points(cloud_in.points, boundary1_points,
                            [](double x, double y) {
                                return x + y;
                            });
    find_xy_boundary_points(cloud_in.points, boundary2_points,
                            [](double x, double y) {
                                return -x + y;
                            });
    find_max_z_points(cloud_in.points,
                      mission_param.mission_tunnel_x_down_, mission_param.mission_tunnel_x_up_,
                      mission_param.mission_tunnel_y_right_, mission_param.mission_tunnel_y_left_,
                      max_z_points);
    // 空间点流聚类
    B_boundary1_filter.reset();
    B_boundary2_filter.reset();
    B_zz_filter.reset();
    for (int i = 0; i < boundary1_points.size(); i++) {
        B_boundary1_filter.cluster(boundary1_points[i].x, boundary1_points[i].y);
    }
    for (int i = 0; i < boundary2_points.size(); i++) {
        B_boundary2_filter.cluster(boundary2_points[i].x, boundary2_points[i].y);
    }
    for (int i = 0; i < max_z_points.size(); i++) {
        B_zz_filter.cluster(max_z_points[i], max_z_points[i]);
    }
    std::vector<CloudClusterFilter::ClusterPoint> cluster1_points;
    std::vector<CloudClusterFilter::ClusterPoint> cluster2_points;
    std::vector<CloudClusterFilter::ClusterPoint> zz_points;
    B_boundary1_filter.get_first_n_cluster_points(1, cluster1_points);
    B_boundary2_filter.get_first_n_cluster_points(1, cluster2_points);
    B_zz_filter.get_first_n_cluster_points(1, zz_points);
    if (cluster1_points.empty() || cluster2_points.empty() || zz_points.empty()) {
        return;
    }
    double dx = cluster2_points[0].x - cluster1_points[0].x;
    double dy = cluster2_points[0].y - cluster1_points[0].y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < 0.5) {
        return;
    }

    //发布隧道的前后中心位置
    double tunnel_in_x, tunnel_in_y;
    double tunnel_out_x, tunnel_out_y;
    const double tunnel_width = 0.65;
    double tunnel_height = zz_points[0].x - 0.45;
    tunnel_in_x = cluster1_points[0].x - dy / distance * tunnel_width;
    tunnel_in_y = cluster1_points[0].y + dx / distance * tunnel_width;
    tunnel_out_x = cluster2_points[0].x - dy / distance * tunnel_width;
    tunnel_out_y = cluster2_points[0].y + dx / distance * tunnel_width;
    // 发布柱子位置
    ius_msgs::TunnelPos tunnel_pos_msg;
    tunnel_pos_msg.header.frame_id = "world";
    tunnel_pos_msg.header.stamp = ros::Time::now();
    tunnel_pos_msg.tunnel_pos_in.x = tunnel_in_x;
    tunnel_pos_msg.tunnel_pos_in.y = tunnel_in_y;
    tunnel_pos_msg.tunnel_pos_in.z = tunnel_height;
    tunnel_pos_msg.tunnel_pos_out.x = tunnel_out_x;
    tunnel_pos_msg.tunnel_pos_out.y = tunnel_out_y;
    tunnel_pos_msg.tunnel_pos_out.z = tunnel_height;
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

CloudClusterFilter C_boundary1_filter(0.2, 5);
CloudClusterFilter C_boundary2_filter(0.2, 5);
CloudClusterFilter C_zz_filter(0.2, 5);
void taskC_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    pcl::PointCloud<pcl::PointXYZ>::VectorType boundary1_points; // x - y
    pcl::PointCloud<pcl::PointXYZ>::VectorType boundary2_points; // x + y
    std::vector<double> max_z_points;
    find_xy_boundary_points(cloud_in.points, boundary1_points,
                            [](double x, double y) {
                                return x - y;
                            });
    find_xy_boundary_points(cloud_in.points, boundary2_points,
                            [](double x, double y) {
                                return x + y;
                            });
    find_max_z_points(cloud_in.points,
                      mission_param.mission_maze_x_down_, mission_param.mission_maze_x_up_,
                      mission_param.mission_maze_y_right_, mission_param.mission_maze_y_left_,
                      max_z_points);
    // 空间点流聚类
    C_boundary1_filter.reset();
    C_boundary2_filter.reset();
    C_zz_filter.reset();
    for (int i = 0; i < boundary1_points.size(); i++) {
        C_boundary1_filter.cluster(boundary1_points[i].x, boundary1_points[i].y);
    }
    for (int i = 0; i < boundary2_points.size(); i++) {
        C_boundary2_filter.cluster(boundary2_points[i].x, boundary2_points[i].y);
    }
    for (int i = 0; i < max_z_points.size(); i++) {
        C_zz_filter.cluster(max_z_points[i], max_z_points[i]);
    }
    std::vector<CloudClusterFilter::ClusterPoint> cluster1_points;
    std::vector<CloudClusterFilter::ClusterPoint> cluster2_points;
    std::vector<CloudClusterFilter::ClusterPoint> zz_points;
    C_boundary1_filter.get_first_n_cluster_points(1, cluster1_points);
    C_boundary2_filter.get_first_n_cluster_points(1, cluster2_points);
    C_zz_filter.get_first_n_cluster_points(1, zz_points);
    if (cluster1_points.empty() || cluster2_points.empty() || zz_points.empty()) {
        return;
    }
    double dx = cluster2_points[0].x - cluster1_points[0].x;
    double dy = cluster2_points[0].y - cluster1_points[0].y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < 0.5) {
        return;
    }

    //发布迷宫门口的前后中心位置
    double maze_in_x, maze_in_y;
    double maze_mid_x, maze_mid_y;
    double maze_out_x, maze_out_y;
    const double maze_in_dist = 4.1;
    const double maze_mid_dist = 0.9;
    const double maze_out_dist = 4.1;
    double maze_height = zz_points[0].x - 2.0;
    maze_in_x = cluster1_points[0].x - dy / distance * maze_in_dist;
    maze_in_y = cluster1_points[0].y + dx / distance * maze_in_dist;
    maze_out_x = cluster2_points[0].x - dy / distance * maze_out_dist;
    maze_out_y = cluster2_points[0].y + dx / distance * maze_out_dist;
    // maze_mid_x = (maze_in_x + maze_out_x) / 2;
    // maze_mid_y = (maze_in_y + maze_out_y) / 2;
    double mid_x = (cluster1_points[0].x + cluster2_points[0].x) / 2.0;
    double mid_y = (cluster1_points[0].y + cluster2_points[0].y) / 2.0;
    maze_mid_x = mid_x - dy / distance * maze_mid_dist;
    maze_mid_y = mid_y + dx / distance * maze_mid_dist;
    double maze_mid_height = zz_points[0].x - 1.3;

    ius_msgs::MazePos maze_pos_msg;
    maze_pos_msg.header.frame_id = "world";
    maze_pos_msg.header.stamp = ros::Time::now();
    maze_pos_msg.maze_pos_in.x = maze_in_x;
    maze_pos_msg.maze_pos_in.y = maze_in_y;
    maze_pos_msg.maze_pos_in.z = maze_height;
    maze_pos_msg.maze_pos_mid.x = maze_mid_x;
    maze_pos_msg.maze_pos_mid.y = maze_mid_y;
    maze_pos_msg.maze_pos_mid.z = maze_mid_height;
    maze_pos_msg.maze_pos_out.x = maze_out_x;
    maze_pos_msg.maze_pos_out.y = maze_out_y;
    maze_pos_msg.maze_pos_out.z = maze_height;
    maze_pos_pub.publish(maze_pos_msg);
}

double loop_height = 0;
void taskD_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
    if (cloud_in.points.size() < 30) {
        return;
    }
    float sum_x,sum_y,ave_x,ave_y,sum_z,ave_z;
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    ave_x = 0;
    ave_y = 0;
    ave_z = 0;
    for(int i = 0;i < cloud_in.points.size();i++)
    {
       sum_x += cloud_in.points[i].x;
       sum_y += cloud_in.points[i].y;
       sum_z += cloud_in.points[i].z;
    }
    ave_x = sum_x/cloud_in.points.size();
    ave_y = sum_y/cloud_in.points.size();
    ave_z = sum_z/cloud_in.points.size();
    
    // 发布圆环位置
    ius_msgs::LoopPos loop_pos_msg;
    loop_pos_msg.header.frame_id = "world";
    loop_pos_msg.header.stamp = ros::Time::now();
    
    loop_pos_msg.loop_id = 1;
    loop_pos_msg.loop_pos.x = ave_x;
    loop_pos_msg.loop_pos.y = ave_y;
    loop_pos_msg.loop_pos.z = ave_z + 0.25;
    loop_pos_pub.publish(loop_pos_msg);
    loop_height = ave_z;
}

void taskE_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
    if (cloud_in.points.size() < 30) {
        return;
    }
    float sum_x,sum_y,ave_x,ave_y,sum_z,ave_z;
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    ave_x = 0;
    ave_y = 0;
    ave_z = 0;
    for(int i = 0;i < cloud_in.points.size();i++)
    {
       sum_x += cloud_in.points[i].x;
       sum_y += cloud_in.points[i].y;
       sum_z += cloud_in.points[i].z;
    }
    ave_x = sum_x/cloud_in.points.size();
    ave_y = sum_y/cloud_in.points.size();
    ave_z = sum_z/cloud_in.points.size();
    
    // 发布圆环位置
    ius_msgs::LoopPos loop_pos_msg;
    loop_pos_msg.header.frame_id = "world";
    loop_pos_msg.header.stamp = ros::Time::now();
    
    loop_pos_msg.loop_id = 2;
    loop_pos_msg.loop_pos.x = ave_x;
    loop_pos_msg.loop_pos.y = ave_y;
    loop_pos_msg.loop_pos.z = loop_height + 0.35;
    loop_pos_pub.publish(loop_pos_msg);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_A;
    pcl::PointCloud<pcl::PointXYZ> cloud_B;
    pcl::PointCloud<pcl::PointXYZ> cloud_C;
    pcl::PointCloud<pcl::PointXYZ> cloud_D;
    pcl::PointCloud<pcl::PointXYZ> cloud_E;
    // ros msg to pcl pointcloud
    pcl::fromROSMsg(*cloud_msg, cloud_in);
    // 输出原始点云信息
    ROS_INFO("Cloud size: %d", cloud_in.points.size());

    // 点云预处理
    cloud_preprocess(cloud_in, cloud_A, cloud_B, cloud_C, cloud_D, cloud_E);

    // 任务A：柱子识别
    taskA_perception(cloud_A);

    // 任务B：隧道识别
    taskB_perception(cloud_B_filtered);

    // 任务C：迷宫门口识别
    taskC_perception(cloud_C_filtered);

    // 任务D：静态圆环识别
    taskD_perception(cloud_D_filtered);

    // 任务E：动态圆环识别
    taskE_perception(cloud_E);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ius_perception");
    ros::NodeHandle nh("~");
    mission_param.load_param(nh);

    mission_B_voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);
    mission_C_voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);
    mission_D_voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);
    mission_E_voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);

    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 1, cloud_cb);
    filter_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
    pillar_pos_pub = nh.advertise<ius_msgs::PillarPos>("/ius_uav/pillar_pos", 50);
    tunnel_pos_pub = nh.advertise<ius_msgs::TunnelPos>("/ius_uav/tunnel_pos", 50);
    maze_pos_pub = nh.advertise<ius_msgs::MazePos>("/ius_uav/maze_pos", 50);
    loop_pos_pub = nh.advertise<ius_msgs::LoopPos>("/ius_uav/loop_pos", 50);
    mission_A_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_A_cloud", 1);
    mission_B_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_B_cloud", 1);
    mission_C_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_C_cloud", 1);
    mission_D_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_D_cloud", 1);
    mission_E_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_E_cloud", 1);

    ros::spin();
    return 0;
}
