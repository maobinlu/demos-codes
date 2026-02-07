/*
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2023-03-07 16:47:27
 * @LastEditors: xbw-ubuntu 15116921911@example.com
 * @LastEditTime: 2025-03-05 20:55:31
 * @FilePath: /catkin_ws/src/arm807_9/hard_arm/src/serial_com.cpp
 * @Description:将串口通信部分函数独立提取出来，使得两个serve端都可以调用与下位机交互
 * @note:将会设置一系列变量，串口输出输入只会更改这些变量，而两个serve端的cb函数都赋值更改此变量
 * */
#include "serial_com.h"

// 初始化变量
int32_t cmd_loc[6] = {0};
int32_t cmd_spd[6] = {0};
int32_t cmd_hand_mode = 0;
int32_t feedback_loc[6] = {0};
int32_t feedback_spd[6] = {0};
int32_t feedback_hand_mode = 0;

/**
 * @brief 串口部分初始化
 *
 */
void serial_init()
{
    to = serial::Timeout::simpleTimeout(100); // 创建timeout
    sp.setPort("/dev/ttyUSB0");               // 设置要打开的串口名称
    sp.setBaudrate(115200);                   // 设置串口通信的波特率
    sp.setTimeout(to);                        // 串口设置timeout
                                              // 开启串口
    serial::bytesize_t data_bits = serial::eightbits;
    serial::parity_t parity = serial::parity_none;
    serial::stopbits_t stop_bits = serial::stopbits_one;

    sp.setBytesize(data_bits);
    sp.setParity(parity);
    sp.setStopbits(stop_bits);
    try
    {
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
    if (sp.isOpen())
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");

    //--------串口接收参数初始化----------
    str = "";
    cjson_str_get = "";
    cjson_obj_get = NULL;
    cjson_data_get = NULL;
    cjson_item_get = NULL;
    skill_array_size = 0;
    cjson_start_get = 0;

    //---------串口发送参数初始化----------
    cjson_str_send = "";
    cjson_obj_send = cJSON_CreateObject();
    cjson_data_send = cJSON_CreateArray();

    // 生成结构
    //{"host:[loc1,loc2,loc3,lo4,lo5,lo6,spd1,spd2,spd3,spd4,spd5,spd6,hand]"}
    for (int i = 0; i < 13; i++)
    {
        // 这里是初始化的参数，之后再设定。
        cJSON_AddItemToArray(cjson_data_send, cJSON_CreateNumber(0));
    }
    cJSON_AddItemToObject(cjson_obj_send, "host", cjson_data_send);
}

/**
 * @brief 赋值loc指令变量 cmd_loc
 */
void set_cmd_loc()
{
    for (int i = 0; i < 6; i++)
    {
        cJSON_ReplaceItemInArray(cjson_data_send, i, cJSON_CreateNumber(cmd_loc[i]));
    }
}

/**
 * @brief 赋值spd指令变量 cmd_spd
 */
void set_cmd_spd()
{
    for (int i = 6; i < 12; i++)
    {
        cJSON_ReplaceItemInArray(cjson_data_send, i, cJSON_CreateNumber(cmd_spd[i - 6]));
    }
}

/**
 * @brief 赋值hand变量 将cmd_hand_mode
 */
void set_cmd_hand_mode()
{
    cout<<"set hand(0开 1关):"<<cmd_hand_mode<<endl;
    cJSON_ReplaceItemInArray(cjson_data_send, 12, cJSON_CreateNumber(cmd_hand_mode));
}

/**
 * @brief json发送
 *
 */
void json_out()
{
    // cout << "updata ok" << endl;
    cjson_str_send = cJSON_Print(cjson_obj_send);
    // cout << "generate str ok" << endl;
    sp.write(cjson_str_send);
    // cout << cjson_str_send << endl;
}

void json_input()
{
    static std::string cache; // 缓存字符串，用于存储上一次读取到的剩余数据
    size_t n = sp.available();
    if (n == 0) {
        return;
    }

    // 读取可用数据到buf缓存中，避免越界
    char* buf = new char[n];
    sp.read((uint8_t*)buf, n); // 将数据读取到buf缓存中

    std::string s(buf, n);
    delete[] buf;

    // 将上一次读取剩余的数据与本次读取到的数据拼接在一起，组成完整的数据字符串
    std::string data = cache + s;
    size_t len = data.length();

    // cout<< data<<endl;

    // 处理完整的数据包
    int start_idx = -1;
    int end_idx = -1;
    for (int i = 0; i < len; i++) {
        if (data[i] == '{') {
            start_idx = i;
        }
        else if (data[i] == '}'&& start_idx!=-1) {
            end_idx = i;
            break;
        }
    }


    if (start_idx != -1 && end_idx != -1) {
        // cout<< "test"<<endl;

        std::string cjson_str_get = data.substr(start_idx, end_idx - start_idx + 1);
        // cout<< cjson_str_get<<endl;

        // 对读取的数据进行cjson解码
        const char *str2 = cjson_str_get.c_str();
        cjson_obj_get = cJSON_Parse(str2);
        
        if (cjson_obj_get != NULL) {
            cjson_data_get = cJSON_GetObjectItem(cjson_obj_get, "master");
            int cjson_size = cJSON_GetArraySize(cjson_data_get);
            if (cjson_size == 13) {
                for (int i = 0; i < 6; i++) {
                    feedback_loc[i] = cJSON_GetArrayItem(cjson_data_get, i)->valueint;
                    feedback_spd[i] = cJSON_GetArrayItem(cjson_data_get, i + 6)->valueint;
                }
                feedback_hand_mode = cJSON_GetArrayItem(cjson_data_get, 12)->valueint;
            }
            else {
                printf("Invalid JSON object size.\n");
            }
        }
        else {
            printf("JSON parse failed.\n");
        }

        // 将剩余数据缓存到下一次处理
        cache = data.substr(end_idx + 1);
    }
    else {
        // 如果读取的数据没有完整的数据包，则将数据放入缓存，待下次处理
        cache = data;
    }
}
/**
 * @brief json输入，获得数据。
 * 数据格式："{master:[loc1,loc2,loc3,loc4,loc5,loc6,spd1,spd2,spd3,spd4,spd5,spd6,hand]}"
 * loc--> feedback_loc; spd--> feedback_spd; hand--> feedback_hand
//  */
// void json_input()
// {
//     size_t n = sp.available();
//     if (n != 0)
//     {
//         // 读取全部数据，效果要更好一点
//         std::string s = sp.read(n);
//         for (int i = 0; i < n; i++)
//         {
//             std::string f(1, s[i]); // 将s[i]的char字符强制转换为string类型
//             if (f == "{")
//             {
//                 str = ""; // 检测为{，清空数据，开始下个结构体
//                 cjson_start_get = 1;
//             }
//             else if (f == "}")
//             {
//                 cjson_str_get = str + "}"; // 检测为}，结构体读取完毕，对格式进行补足{}
//                 cjson_start_get = 0;
//             }
//             if (cjson_start_get)
//                 str = str + f; // 将字符链接为内容；
//         }
//         // std::cout << s ;
//     }
//     // std::cout << n << std::endl;

//     // 对读取的数据进行cjson解码
//     const char *str2 = cjson_str_get.c_str();
//     // std::cout << cjson_str_get << std::endl;
//     // printf("%s", str2);
//     cjson_obj_get = cJSON_Parse(str2);
//     if (cjson_obj_get == NULL && n>0)
//     {
//         printf("parse fail.\n");
//     }
//     else
//     {
//         cjson_data_get = cJSON_GetObjectItem(cjson_obj_get, "master");
//         int cjson_size = cJSON_GetArraySize(cjson_data_get);
//         // std::cout << cjson_size<< std::endl;
//         if (cjson_size == 13)
//         {
//             for (int i = 0; i < 6; i++)
//             {
//                 feedback_loc[i] = cJSON_GetArrayItem(cjson_data_get, i)->valueint;
//                 feedback_spd[i] = cJSON_GetArrayItem(cjson_data_get, i + 6)->valueint;

//                 // master[i] = cJSON_GetArrayItem(cjson_data_get, i)->valueint;
//                 // std::cout << i <<": "<< master[i] << std::endl;
//             }
//             feedback_hand_mode = cJSON_GetArrayItem(cjson_data_get, 12)->valueint;
//         }
//         // printf('motor2loc = %d',master[1]);
//     }
// }