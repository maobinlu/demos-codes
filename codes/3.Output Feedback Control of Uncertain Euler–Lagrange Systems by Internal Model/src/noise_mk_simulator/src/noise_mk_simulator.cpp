#include <ros/ros.h>
#include <random>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
double LAZ, X0, Y0, Z0;

// 生成不超过1米误差的三维正态分布的函数
void generateNormalDistribution3D(double& x, double& y, double& z) {
    // 创建一个伪随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(0.0, 0.3);  // 均值为0，标准差为0.3

    // 生成三个正态分布的随机数
    x = distribution(gen);
    y = distribution(gen);
    z = distribution(gen);

    // 归一化到1米误差范围内
    double norm = sqrt(x * x + y * y + z * z);
    x /= norm;
    y /= norm;
    z /= norm;
}

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Republish the received data to the output topic


    if (LAZ < 0)
    {
        X0 = msg->pose.position.x;
        Y0 = msg->pose.position.y;
        Z0 = msg->pose.position.z;

        LAZ = 1.00;

    }
    geometry_msgs::PoseStamped msg_xyz;
    double x, y, z;
    generateNormalDistribution3D(x, y, z);
    msg_xyz = *msg;
    std::cout << "RDF" << x << "S" << y << z << std::endl;
    // 将生成的随机数加入到消息中
    msg_xyz.pose.position.x = msg->pose.position.x - X0 + x;
    msg_xyz.pose.position.y = msg->pose.position.y - Y0 + y;
    msg_xyz.pose.position.z = msg->pose.position.z - Z0 + z;
    pub.publish(msg_xyz);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_republisher");
    ros::NodeHandle nh;

    // Create a subscriber to the input topic
    LAZ = -1.00;
    ros::Subscriber sub = nh.subscribe("/vrpn_client_node/robofly/pose", 1000, &callback);

    // Create a publisher for the output topic
    pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/RB1/SIM_POse", 1000);

    // Spin forever
    ros::spin();
}

