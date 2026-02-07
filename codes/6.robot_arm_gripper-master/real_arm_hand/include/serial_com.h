/*
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2022-08-26 16:49:09
 * @LastEditors: xbw-ubuntu 15116921911@example.com
 * @LastEditTime: 2023-03-09 17:31:57
 * @FilePath: /catkin_ws/src/real_arm/include/real_arm/common.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef SERIALCOMMON_H
#define SERIALCOMMON_H

#include "common.h"

//--------------参数----------------
extern int32_t cmd_loc[6];
extern int32_t cmd_spd[6];
extern int32_t cmd_hand_mode;
extern int32_t feedback_loc[6];
extern int32_t feedback_spd[6];
extern int32_t feedback_hand_mode;

//--------------串口配置----------------
static serial::Serial sp;  // 创建一个serial类
static serial::Timeout to; // 创建timeout

//-------------串口接收参数-------------
static std::string str;
static std::string cjson_str_get;
static cJSON *cjson_obj_get;
static cJSON *cjson_data_get;
static cJSON *cjson_item_get;
// int32_t master[12]; // master to host :realloc,realspd
static int skill_array_size;
static bool cjson_start_get;

//-------------串口发送参数--------------
static std::string cjson_str_send;
static cJSON *cjson_obj_send;
static cJSON *cjson_data_send;
// int32_t host[12]; // host to master :camdloc,camdspd

//---------------函数------------------
void serial_init();
void set_cmd_loc();
void set_cmd_spd();
void set_cmd_hand_mode();
void json_out();
void json_input();

#endif // __COMMON__