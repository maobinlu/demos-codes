/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-02-11 17:28:39
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-02-16 21:52:47
 * @FilePath: /arm807_hand_ws/src/real_arm_hand/include/real_arm_hand/PickAndPlace.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "common.h"
#include "RealRobot.h"

#define NUM_OF_JOINT_AND_TOOL 7
#define HOME_POSE   1
#define DEMO_START  2
#define DEMO_STOP   3

typedef struct _ArMarker
{
  uint32_t id;
  double position[3];
} ArMarker;

typedef struct _F2dObject
{
  uint32_t id;
  double position[3];
} F2dObject;

enum class CurrentMode
{
	idle=0, //待机
    init=1,
    waitObject=2,
    move_to_pick=3,
    close_object=4,
    move_to_place=5
};

class PickAndPlace
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  uint8_t demo_count_;
  uint8_t pick_ar_id_;
  uint8_t avg_size;


public:
    PickAndPlace();
    ~PickAndPlace();

 
}

