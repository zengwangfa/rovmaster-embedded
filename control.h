/*
 * @Author: your name
 * @Date: 2020-06-25 14:43:23
 * @LastEditTime: 2020-06-30 17:12:21
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: \rov-master-master\user\control.h
 */ 
#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "config.h"

typedef struct
{
    int16_t fx;    // 摇杆 X轴动力 (X正方向为机头方向)
    int16_t fy;    // 摇杆 Y轴动力
    int16_t fz;    // 摇杆 Z轴动力
    int16_t yaw;   // 偏航角度
    int16_t force; // 合力大小

    float deg;     // 合力角度值
    float rad;     // 合力弧度角
    float percent; // 动力百分比

} rockerInfo_t; /* 摇杆信息 */

typedef struct 
{
    float speech_value_now  ;           //当前速度
    float speech_value_last ;           //初速度速度
    float displacement      ;           //位移
}speech_t;                        //偏差



void sixAixs_control(rockerInfo_t *rc,propellerPower_t *propeller); //六轴控制函数 
void Speed_Buffer(short *now_value,short *last_value,short range);  //缓冲函数

#endif
