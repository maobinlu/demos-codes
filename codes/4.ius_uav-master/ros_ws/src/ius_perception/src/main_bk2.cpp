#include <vector>
#include <deque>

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

#include <ius_perception/mission_param.h>

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
ros::Publisher loop_pos_pub;

MissionParam mission_param;

// 任务区点云分离与缓存
std::deque<std::vector<pcl::PointXYZ>> mission_A_cloud_buffer;
size_t mission_A_buffer_size = 1;
size_t mission_A_cloud_points_num = 0;

std::deque<std::vector<pcl::PointXYZ>> mission_B_task_cloud_buffer;
size_t mission_B_buffer_size = 100;
size_t mission_B_cloud_points_num = 0;

std::deque<std::vector<pcl::PointXYZ>> mission_C_cloud_buffer;
size_t mission_C_buffer_size = 100;
size_t mission_C_cloud_points_num = 0;

std::deque<std::vector<pcl::PointXYZ>> mission_D_cloud_buffer;
size_t mission_D_buffer_size = 100;
size_t mission_D_cloud_points_num = 0;

std::deque<std::vector<pcl::PointXYZ>> mission_E_cloud_buffer;
size_t mission_E_buffer_size = 100;
size_t mission_E_cloud_points_num = 0;

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
    std::vector<pcl::PointXYZ> mission_A_cloud;
    std::vector<pcl::PointXYZ> mission_B_cloud;
    std::vector<pcl::PointXYZ> mission_C_cloud;
    std::vector<pcl::PointXYZ> mission_D_cloud;
    std::vector<pcl::PointXYZ> mission_E_cloud;
    for (int i = 0; i < cloud_in.points.size(); i++) {
        if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_pillar_x_down_, mission_param.mission_pillar_x_up_,
                              mission_param.mission_pillar_y_right_, mission_param.mission_pillar_y_left_)) {
            mission_A_cloud.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_tunnel_x_down_, mission_param.mission_tunnel_x_up_,
                              mission_param.mission_tunnel_y_right_, mission_param.mission_tunnel_y_left_)) {
            mission_B_cloud.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_maze_x_down_, mission_param.mission_maze_x_up_,
                              mission_param.mission_maze_y_right_, mission_param.mission_maze_y_left_)) {
            mission_C_cloud.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_loop_x_down_, mission_param.mission_loop_x_up_,
                              mission_param.mission_loop_y_right_, mission_param.mission_loop_y_left_)) {
            mission_D_cloud.push_back(cloud_in.points[i]);
        } else if (is_point_in_range(cloud_in.points[i], 
                              mission_param.mission_m_loop_x_down_, mission_param.mission_m_loop_x_up_,
                              mission_param.mission_m_loop_y_right_, mission_param.mission_m_loop_y_left_)) {
            mission_E_cloud.push_back(cloud_in.points[i]);
        }
    }
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
           mission_D_cloud_points_num > 100) {
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
void cloud_mission_buffer_merge(const std::deque<std::vector<pcl::PointXYZ>>& cloud_buff,
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
    pcl::toROSMsg(cloud_B, cloud_B_msg);
    cloud_B_msg.header.frame_id = "world";
    cloud_B_msg.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 cloud_C_msg;
    pcl::toROSMsg(cloud_C, cloud_C_msg);
    cloud_C_msg.header.frame_id = "world";
    cloud_C_msg.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 cloud_D_msg;
    pcl::toROSMsg(cloud_D, cloud_D_msg);
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
    
private:
    float radius_;
    int min_points_;

    std::vector<ClusterPoint> cluster_points_;
    
    double inline distance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }
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

void taskB_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
    //中心点
    double min_x[10] = {100,100,100,100,100,100,100,100,100,100};
    double y[10];
    double dz = 0.3;

    for(int i=0; i<cloud_in.points.size(); i++)
    {
        for(int a=0;a<10;a++)
        {
            if (1 + dz * a < cloud_in.points[i].z < dz*(a+1) + 1)
            {
                if (cloud_in.points[i].x < min_x[a])
                {
                    min_x[a] = cloud_in.points[i].x;
                    y[a] = cloud_in.points[i].y;
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
    for(int i=0; i<cloud_in.points.size(); i++)
    {
        for(int a=0;a<10;a++)
        {
            if (1 + dz * a < cloud_in.points[i].z < dz*(a+1) + 1)
            {
                if (cloud_in.points[i].y > max_y[a])
                {
                    max_y[a] = cloud_in.points[i].y;
                    x[a] = cloud_in.points[i].x;
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
    // 发布柱子位置
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

void taskC_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in) {
    // 平面提取
    std::vector<pcl::shared_ptr<Plane>> planes;
    plane_extract(cloud_in, planes);
}

void taskD_perception(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
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
    loop_pos_msg.loop_pos.z = 1;
    loop_pos_pub.publish(loop_pos_msg);
// }
      std::cout << "Point (" << ave_x << ", " << ave_y << ") " << std::endl;
        printf("############################################### \n");
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
    taskB_perception(cloud_B);

    // 任务C：迷宫门口识别
    // taskC_perception(cloud_C);

    // 任务D：静态圆环识别
    taskD_perception(cloud_D);

    // 任务E：动态圆环识别
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ius_perception");
    ros::NodeHandle nh("~");
    mission_param.load_param(nh);

    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 1, cloud_cb);
    filter_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
    pillar_pos_pub = nh.advertise<ius_msgs::PillarPos>("/ius_uav/pillar_pos", 50);
    tunnel_pos_pub = nh.advertise<ius_msgs::TunnelPos>("/ius_uav/tunnel_pos", 50);
    loop_pos_pub = nh.advertise<ius_msgs::LoopPos>("/ius_uav/loop_pos", 50);
    mission_A_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_A_cloud", 1);
    mission_B_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_B_cloud", 1);
    mission_C_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_C_cloud", 1);
    mission_D_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_D_cloud", 1);
    mission_E_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ius_uav/mission_E_cloud", 1);

    ros::spin();
    return 0;
}
