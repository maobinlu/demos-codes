/*
 * @Author: yxz 2410391147@qq.com
 * @Date: 2023-03-08 21:26:49
 * @LastEditors: yxz 2410391147@qq.com
 * @LastEditTime: 2023-03-14 13:54:51
 * @FilePath: /arm807_hand_ws/src/real_arm_hand/include/real_arm_hand/plan_graspsv1.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
# define M_PI		3.14159265358979323846	/* pi */

    ros::Subscriber grasps_sub_;
    rs_yolo::Info object_position;



    tf::Vector3 obj_camera_frame, obj_robot_frame;

   
    //相机坐标与/link6的偏移参数,待设置
    double camera_shifting[3][1]={0,0,0};

    std::vector<moveit_msgs::Grasp> grasps;