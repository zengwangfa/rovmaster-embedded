/**
 * @desc: 基本控制程序
 */
#define LOG_TAG "ctrl"

#include "../applications/PID.h"
#include "../applications/data.h"
#include "../applications/pwmDevices.h"

#include "control.h"
#include "datatype.h"

#include <elog.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void fourAixs_get_rocker_params(rockerInfo_t *rc, cmd_t *cmd) // 获取摇杆参数值
{
    // 摇杆值居中为 127，范围为 0~255，因此需要转换为 -127 ~ +127
    rc->fx = cmd->move_back - 126;  // X轴摇杆值
    rc->fy = cmd->left_right - 128; // Y轴摇杆值
    rc->fz = cmd->up_down - 127;    // Z轴摇杆值(当大于128时上浮，小于128时下潜，差值越大，速度越快)
    rc->yaw = cmd->rotate - 128;    // 偏航摇杆值

    rc->fx += (rc->yaw / 2); // 4推ROV，偏航摇杆值叠加在 X轴摇杆轴上(除以2是为了减小 偏航摇杆值的影响)

    /* 垂直方向 */
    if (abs(rc->fz) < 10) // Z轴摇杆值较小时不进行计算，防止过度累加
        rc->fz = 0;

    /* 水平方向 */
    rc->force = sqrt(rc->fx * rc->fx + rc->fy * rc->fy); // 求取合力斜边大小
    rc->deg = Rad2Deg(atan2(rc->fx, rc->fy));            // 求取合力角度：180 ~ -180

    if (rc->deg < 0) // 角度变换 以极坐标定义 角度顺序 0~360°
        rc->deg += 360;

    rc->rad = Deg2Rad(rc->deg); // 转换弧度角 0~2PI

    rc->percent = cmd->power / 100; // power最大为255 计算动力百分比
}

/**
 * @brief  4推 水平方向控制
 * @param  rocker_t 摇杆结构体
 */
void fourAixs_horizontal_control(rockerInfo_t *rc, propellerPower_t *propeller)
{
    float left_force;  // 左推力
    float right_force; // 右推力

    /* 直线前进/后退角度锁定  90°±12°  270°±12° 
	 * 当摇杆 较为归中时，忽略Y轴摇杆值(即仅为前进/后退)
	 */
    if ((rc->deg >= 78 && rc->deg <= 102) || (rc->deg >= 258 && rc->deg <= 282))
    {
        left_force = abs(rc->fx) * sin(rc->rad);
        right_force = abs(rc->fx) * sin(rc->rad);
    }
    else
    {
        /* 左右推进器 运动控制核心公式 */
        left_force = abs(rc->fx) * sin(rc->rad) + abs(rc->fy) * cos(rc->rad); // 解算摇杆获取
        right_force = abs(rc->fx) * sin(rc->rad) - abs(rc->fy) * cos(rc->rad);
    }

    /* 推力公式 = 方向系数*(动力百分比*摇杆对应的推力值+偏差值) */
    // TODO 推进器偏差值  推进器方向
    propeller->leftDown = rc->percent * left_force;
    propeller->rightDown = rc->percent * right_force;
}

/**
 * @brief  深度控制
 * @param  rocker_t 摇杆结构体
 */
void rov_depth_control(rockerInfo_t *rc, propellerPower_t *propeller)
{
    static float expect_depth;
    static float vertical_force; // 垂直方向上的推力输出大小

    if (rc->fz < 10) // 当摇杆不动时，PID进行定深
    {
        Total_Controller.High_Position_Control.Expect = expect_depth;                // 期望深度(为上次摇杆调节到的深度)
        Total_Controller.High_Position_Control.FeedBack = rovInfo.depthSensor.depth; // 深度反馈(为传感器当前深度)
        vertical_force = PID_Control(&Total_Controller.High_Position_Control);       // 获取 高度位置PID控制器 输出的控制量

        propeller->leftMiddle = vertical_force;   // 正反桨
        propeller->rightMiddle = -vertical_force; // 输出为负值
    }
    else // 当摇杆动时
    {
        expect_depth = rovInfo.depthSensor.depth; // 期望深度为当前深度
        propeller->leftMiddle = rc->fz * 2;       // 正反桨
        propeller->rightMiddle = -rc->fz * 2;     // 输出为负值
    }
}

/**
 * @brief  航向角控制
 * @param  rocker_t 摇杆结构体
 */
void rov_yaw_control(rockerInfo_t *rc, propellerPower_t *propeller)
{
    static float expect_yaw;
    static float force; // 推力输出大小

    Total_Controller.High_Position_Control.Expect = expect_yaw;          // 期望航向角
    Total_Controller.High_Position_Control.FeedBack = rovInfo.jy901.yaw; // 当前航向角
    force = PID_Control(&Total_Controller.Yaw_Angle_Control);            // 获取 角度PID控制器 输出的控制量

    propeller->leftDown = force;   // 正反桨
    propeller->rightDown = -force; // 输出为负值
}

void sixAixs_get_rocker_params(rockerInfo_t *rc, cmd_t *cmd) // 获取摇杆参数值
{
    // 摇杆值居中为 127，范围为 0~255，因此需要转换为 -127 ~ +127
    rc->fx = cmd->move_back - 126;  // X轴摇杆值
    rc->fy = cmd->left_right - 128; // Y轴摇杆值
    rc->fz = cmd->up_down - 127;    // 当大于128时上浮，小于128时下潜，差值越大，速度越快
    rc->yaw = cmd->rotate - 128;    // 偏航摇杆值

    /* 垂直方向 */
    if (abs(rc->fz) < 10) // Z轴摇杆值较小时不进行计算，防止过度累加
        rc->fz = 0;

    /* 水平方向 */
    rc->force = sqrt(rc->fx * rc->fx + rc->fy * rc->fy); // 求取合力斜边大小
    rc->deg = Rad2Deg(atan2(rc->fx, rc->fy));            // 求取合力角度：180 ~ -180

    if (rc->deg < 0) // 角度变换 以极坐标定义 角度顺序 0~360°
        rc->deg += 360;

    rc->rad = Deg2Rad(rc->deg); // 转换弧度角 0~2PI

    rc->percent = cmd->power / 100; // power最大为255 计算动力百分比
}
