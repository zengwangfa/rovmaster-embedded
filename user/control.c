/**
 * @desc: 基本控制算法
 */
#define LOG_TAG "ctrl"

#include "../applications/PID.h"
#include "../applications/data.h"
#include "../applications/pwmDevices.h"
#include "../applications/sensor.h"
#include "../drivers/jy901.h"

#include "control.h"
#include "datatype.h"

#include <elog.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define STEP_VLAUE 2

propellerPower_t PropellerBuffer = {0, 0, 0, 0, 0, 0}; //推进器缓冲区；
propellerPower_t Propellerconposent = {0, 0, 0, 0, 0, 0};  //推进器补偿
int location_keep_timer = 0;

void fourAixs_get_rocker_params(rockerInfo_t *rc, cmd_t *cmd) // 获取摇杆参数值
{
    // 摇杆值居中为 127，范围为 0~255，因此需要转换为 -127 ~ +127
    rc->fx = cmd->move_back - 127;  // X轴摇杆值
    rc->fy = cmd->left_right - 127; // Y轴摇杆值
    rc->fz = cmd->up_down - 127;    // Z轴摇杆值(当大于128时上浮，小于128时下潜，差值越大，速度越快)
    rc->yaw = cmd->rotate - 127;    // 偏航摇杆值

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

    /* 直线前进/后退角度锁定  90°±10°  270°±10°
	 * 当摇杆 较为归中时，忽略Y轴摇杆值(即仅为前进/后退)
	 */
    if ((rc->deg >= 80 && rc->deg <= 100) || (rc->deg >= 260 && rc->deg <= 280))
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

    if (abs(rc->fz) < 10) // 当摇杆接近不动时，PID进行定深
    {
        Total_Controller.High_Position_Control.Expect = expect_depth;                // 期望深度(为上次摇杆调节到的深度)
        Total_Controller.High_Position_Control.FeedBack = rovInfo.depthSensor.depth; // 深度反馈(为传感器当前深度)
        vertical_force = PID_Control(&Total_Controller.High_Position_Control);       // 获取 高度位置PID控制器 输出的控制量

        propeller->leftMiddle  =  vertical_force;   // 正反桨
        propeller->rightMiddle = -vertical_force; // 输出为负值
    }
    else // 当摇杆动时
    {
        expect_depth = rovInfo.depthSensor.depth; // 期望深度为当前深度
        PropellerBuffer.leftMiddle  =  rc->fz * 2;       // 正反桨
        PropellerBuffer.rightMiddle = -rc->fz * 2;     // 输出为负值
        Speed_Buffer(&propeller->leftMiddle ,   &PropellerBuffer.leftMiddle,    4);
        Speed_Buffer(&propeller->rightMiddle,   &PropellerBuffer.rightMiddle,   4);
    }

}

/**
 * @brief  航向角控制
 * @param  rocker_t 摇杆结构体
 */
void rov_yaw_control(rockerInfo_t *rc, propellerPower_t *propeller)
{
    //static float expect_yaw;
    static float force; // 推力输出大小

    if(abs(rc->yaw) > 5 )        Total_Controller.High_Position_Control.Expect   = rovInfo.jy901.yaw;          // 期望航向角

    Total_Controller.High_Position_Control.FeedBack = rovInfo.jy901.yaw; // 当前航向角
    force = PID_Control(&Total_Controller.Yaw_Angle_Control);            // 获取 角度PID控制器 输出的控制量

    propeller->leftDown  =  force;   // 正反桨
    propeller->rightDown = -force; // 输出为负值
}

void sixAixs_get_rocker_params(rockerInfo_t *rc, cmd_t *cmd) // 获取摇杆参数值
{
    // 摇杆值居中为 127，范围为 0~255，因此需要转换为 -127 ~ +127
    rc->fx  = cmd->move_back  - 127;  // X轴摇杆值
    rc->fy  = cmd->left_right - 127; // Y轴摇杆值
    rc->fz  = cmd->up_down    - 127;    // 当大于128时上浮，小于128时下潜，差值越大，速度越快
    rc->yaw = cmd->rotate     - 127;    // 偏航摇杆值

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
 * @brief  推进器缓冲
 * @param  last_value 变化数值指针，缓冲数值指针，变化幅度
 */
void Speed_Buffer(short *now_value, short *last_value, short range)
{
    static uint16_t diff_value = 0;
    diff_value = abs((*last_value) - (*now_value)); //暂存差值的绝对值

    if (diff_value >= range ) //微分大于预设值，启动缓冲
    {
        if (*now_value <= *last_value)
        {
            *now_value = *now_value + STEP_VLAUE;
        }
        else
        {
            *now_value = *now_value - STEP_VLAUE;
        }
        *last_value = *now_value;
    }
    else
    {
       *now_value = *last_value;
    }
    
}

/**
 * @brief  六轴推进器水平控制
 * @param  rocker_t 摇杆结构体
 */
void sixAixs_horizontal_control(rockerInfo_t *rc, propellerPower_t *propeller)
{
    static int propeller_Value = 1; //推进器比例数值

    static int propeller_Direction_Down_X =  1; //推进器参数方向
    static int propeller_Direction_Up_X   = -1;

    static int propeller_Direction_Down_Y = -1;
    static int propeller_Direction_Up_Y   =  1;

    if ((abs(rc->fx) < 10 && abs(rc->fy) < 10))      //当遥感值较小或锁定时，推进器全部失能，后期可以修改为自稳
    {
        PropellerBuffer.leftDown  = 0;
        PropellerBuffer.rightDown = 0;
        PropellerBuffer.leftUp    = 0;
        PropellerBuffer.rightUp   = 0;
    }
    // else if(abs(rc->yaw)>5)
    // {
    //     PropellerBuffer.leftDown  =  propeller_Value * rc->yaw;     //根据遥感获取动力0~254
    //     PropellerBuffer.rightDown = -propeller_Value * rc->yaw;
    //     PropellerBuffer.leftUp    = -propeller_Value * rc->yaw;
    //     PropellerBuffer.rightUp   =  propeller_Value * rc->yaw;
    // }
    else
    {
        PropellerBuffer.leftDown  = propeller_Value * (rc->fx * propeller_Direction_Down_X + rc->fy * propeller_Direction_Down_Y + Propellerconposent.leftUp);     //根据遥感获取动力0~254
        PropellerBuffer.rightDown = propeller_Value * (rc->fx * propeller_Direction_Down_X + rc->fy * propeller_Direction_Up_Y   + Propellerconposent.rightDown);
        PropellerBuffer.leftUp    = propeller_Value * (rc->fx * propeller_Direction_Up_X   + rc->fy * propeller_Direction_Down_Y + Propellerconposent.leftUp);
        PropellerBuffer.rightUp   = propeller_Value * (rc->fx * propeller_Direction_Up_X   + rc->fy * propeller_Direction_Up_Y   + Propellerconposent.rightUp);
    }

    Speed_Buffer(&propeller->leftDown,  &PropellerBuffer.leftDown,  4);    //缓冲器
    Speed_Buffer(&propeller->rightDown, &PropellerBuffer.rightDown, 4);
    Speed_Buffer(&propeller->leftUp,    &PropellerBuffer.leftUp,    4);
    Speed_Buffer(&propeller->rightUp,   &PropellerBuffer.rightUp,   4);

}

void propeller_conposent_horizontal(rockerInfo_t *rc, propellerPower_t *Propellerconposent)    //推进器死区补偿
{
    if(rc->fx >abs(rc->fy))
    {
        Propellerconposent->leftUp    = -34;
        Propellerconposent->rightUp   = -33;
        Propellerconposent->leftDown  =  25;
        Propellerconposent->rightDown =  33;
    }
    else if(rc->fx < abs(rc->fy))
    {
        Propellerconposent->leftUp    =  34;
        Propellerconposent->rightUp   =  20;
        Propellerconposent->leftDown  = -17;
        Propellerconposent->rightDown = -35;
    }
    else if(rc->fy > abs(rc->fx))
    {
        Propellerconposent->leftUp    = -34;
        Propellerconposent->rightUp   =  20;
        Propellerconposent->leftDown  = -17;
        Propellerconposent->rightDown =  33;

    }
    else if(rc->fy < abs(rc->fx))
    {
        Propellerconposent->leftUp    =  34;
        Propellerconposent->rightUp   = -33;
        Propellerconposent->leftDown  =  25;
        Propellerconposent->rightDown = -35;

    }

}

void propeller_conposent_depth1(rockerInfo_t *rc, propellerPower_t *Propellerconposent)
{
    if(rc->fy >= 2)
    {
        Propellerconposent->leftMiddle    =  20;
        Propellerconposent->rightMiddle   = -40;

    }
    else if(rc->fy <=-2)
    {
        Propellerconposent->leftMiddle    = -40;
        Propellerconposent->rightMiddle   =  40;
    }

}

/**
 * @brief  动力补偿
 * @param  speech
 */
void location_keep_control(powerconpensation_t *powerconpensation)
{

    powerconpensation->X.speech_value_now = powerconpensation->X.speech_value_last + rovInfo.jy901.acc.x;                        //计算速度
    powerconpensation->X.displacement = (powerconpensation->X.speech_value_now + powerconpensation->X.speech_value_last) * 0.5f; //计算位移
    powerconpensation->Y.speech_value_now = powerconpensation->Y.speech_value_last + rovInfo.jy901.acc.y;
    powerconpensation->Y.displacement = (powerconpensation->Y.speech_value_now + powerconpensation->Y.speech_value_last) * 0.5f;

    Total_Controller.Location_X_Control.Expect = 0; // 定位原点
    Total_Controller.Location_Y_Control.Expect = 0;

    Total_Controller.Location_X_Control.FeedBack = powerconpensation->X.displacement; // X加速度反馈(为x轴当前加速度)
    Total_Controller.Location_Y_Control.FeedBack = powerconpensation->Y.displacement; // Y加速度反馈(为y轴当前加速度)

    powerconpensation->X.power_conpensation = PID_Control(&Total_Controller.Location_X_Control); // 获取 X轴动力补偿PID控制器输出的控制量
    powerconpensation->Y.power_conpensation = PID_Control(&Total_Controller.Location_Y_Control); // 获取 y轴动力补偿PID控制器 输出的控制量

    if (powerconpensation->X.power_conpensation > 50)                                   //限制补偿动力
        powerconpensation->X.power_conpensation = 50;
    else if (powerconpensation->X.power_conpensation < -50)
        powerconpensation->X.power_conpensation = -50;

    if (powerconpensation->Y.power_conpensation > 50)
        powerconpensation->Y.power_conpensation = 50;
    else if (powerconpensation->Y.power_conpensation < -50)
        powerconpensation->Y.power_conpensation = -50;
}





