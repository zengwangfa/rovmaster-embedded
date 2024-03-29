/*
 * @Author: your name
 * @Date: 2020-06-25 14:43:23
 * @LastEditTime: 2020-07-28 16:56:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \rov-master-master\user\control.h
 */
#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "../applications/pwmDevices.h"
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
    float speech_value_now;  //当前速度
    float speech_value_last; //初速度速度
    float displacement;      //位移
    int power_conpensation;  //动力补偿系数
} speech_t;                  //偏差

typedef struct
{
    speech_t X;
    speech_t Y;
} powerconpensation_t; //动力补偿

void sixAixs_get_rocker_params(rockerInfo_t *rc, cmd_t *cmd); // 获取摇杆参数值

void rov_depth_control(rockerInfo_t *rc, propellerPower_t *propeller);

void Speed_Buffer(short *now_value, short *last_value, short range);//缓冲器

void rov_yaw_control(rockerInfo_t *rc);    //横向角锁定；

void sixAixs_horizontal_control(rockerInfo_t *rc, propellerPower_t *propeller); //六轴控制函数

void propeller_conposent(propellerPower_t *PropellerBuffer ,propellerPower_t *Propellerconposent) ; //水平推进器死区补偿

void propeller_output(propellerPower_t *propeller) ;//动力输出

void location_keep_control(powerconpensation_t *powerconpensation,rockerInfo_t *rc);//抗流

#endif
