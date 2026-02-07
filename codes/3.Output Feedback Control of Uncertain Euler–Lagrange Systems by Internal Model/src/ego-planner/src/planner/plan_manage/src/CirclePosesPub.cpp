#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ius_msgs/CirclePoses.h"
#include "ius_msgs/SingleCirclePos.h"
#include <vector>
using namespace ros;
int	main(int argc, char **argv)
{
    init(argc, argv, "CirclePoses_pub");
    NodeHandle nh;
    Publisher CirclePoses_puber = nh.advertise<ius_msgs::CirclePoses>("/CirclePoses", 100);
    ius_msgs::CirclePoses cps;
    ius_msgs::SingleCirclePos Scps[5];

    for (int i = 0; i < 5; i++)
    {
        Scps[i].index = i;
        if (i == 3)
        {
            Scps[i].yaw = 90.0;
        }
        else
        {
            Scps[i].yaw = 0.0;
        }
    }

    Scps[0].position.x = 3.8;
    Scps[0].position.y = 2.2;
    Scps[0].position.z = 1.5;

    Scps[1].position.x = 7.4;
    Scps[1].position.y = 2.6;
    Scps[1].position.z = 1.8;

    Scps[2].position.x = 18.1;
    Scps[2].position.y = 2.2;
    Scps[2].position.z = 3.5;

    Scps[3].position.x = 23.0;
    Scps[3].position.y = 5.0;
    Scps[3].position.z = 3.5;

    Scps[4].position.x = 18.1;
    Scps[4].position.y = 7.8;
    Scps[4].position.z = 3.5;
    
    for (int i = 0; i < 5; i++)
    {
        cps.poses.push_back(Scps[i]);
    }

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        // cps.poses[2].position.y += 0.3;
        // cps.poses[1].position.y -= 0.5;
        CirclePoses_puber.publish(cps);
        loop_rate.sleep();
        spinOnce();
    }
    
    
    
}