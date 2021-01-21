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


#define SPEED_BUFF 20
#define SPEED_RANGE 30

propellerPower_t PropellerBuffer = {0, 0, 0, 0, 0, 0}; //推进器缓冲区；
propellerPower_t Propellerconposent = {0, 0, 0, 0, 0, 0};  //推进器补偿
powerconpensation_t Propeller_resist_flow;
rockerInfo_t rocker;                                    //手柄
int yaw_conposent;                                       //偏航角推进器补偿
int roll_conposent;                                      //横滚角推进器补偿
extern cmd_t cmd_data;



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
    static float vertical_force; // 垂直方向上的推力输出大小
    if (abs(rc->fz) > 5 && CONTROL_UNLOCK == cmd_data.all_lock) // 当摇杆动时且处于解锁状态
    {
        Total_Controller.High_Position_Control.Expect = rovInfo.depthSensor.depth;                  //将当前深度作为期望深度
        /*----------------推进器动力缓冲区---------------------------------  */
        PropellerBuffer.leftMiddle  = -rc->fz * 2 + roll_conposent ;       // 正反桨动力 =  手柄数值*3 + 姿态自稳调节
        PropellerBuffer.rightMiddle =  rc->fz * 2 - roll_conposent;       // 输出为负值
    }
    else // 当摇杆接近不动时，PID进行定深
    {
        
        Total_Controller.High_Position_Control.FeedBack = rovInfo.depthSensor.depth; // 深度反馈(为传感器当前深度)
        vertical_force = PID_Control(&Total_Controller.High_Position_Control);       // 获取 高度位置PID控制器 输出的控制量

        PropellerBuffer.leftMiddle  =  -vertical_force + roll_conposent ;   // 正反桨
        PropellerBuffer.rightMiddle =   vertical_force - roll_conposent; // 输出为负值
    }

}
/**
 * @description: 横滚姿态调整
 * @param {*}九轴横滚角
 * @return {*}无
 */
void rov_roll_control(rockerInfo_t *rc)
{
     static int roll_flag = CONTROL_UNLOCK;                                 
    // if (CONTROL_UNLOCK == roll_flag && CONTROL_UNLOCK == cmd_data.all_lock)    //解锁时，启动自稳
    // {
    //     roll_flag = CONTROL_LOCK;
    //     Total_Controller.Roll_Angle_Control.Expect   = rovInfo.jy901.roll;                // 解锁时以当前角度为水平期望角度。
    // }
    if(roll_flag)
    {
        roll_flag = 0;
        Total_Controller.Roll_Angle_Control.Expect   = rovInfo.jy901.roll;                // 解锁时以当前角度为水平期望角度。
    }
    //    Total_Controller.Roll_Angle_Control.Expect   = rovInfo.jy901.roll;                // 解锁时以当前角度为水平期望角度。
        Total_Controller.Roll_Angle_Control.FeedBack = rovInfo.jy901.roll;                //当前值传入PID
        roll_conposent = PID_Control_Yaw(&Total_Controller.Roll_Angle_Control);           //获取 角度PID控制器 输出的控制量



}

/**
 * @brief  航向角控制
 * @param  rocker_t 摇杆结构体
 */
void rov_yaw_control(rockerInfo_t *rc)
{
    static int yaw_flag = CONTROL_UNLOCK;                                 
    if (CONTROL_UNLOCK == yaw_flag && CONTROL_UNLOCK == cmd_data.all_lock)    //解锁时，启动自稳
    {
        yaw_flag = CONTROL_LOCK;
        Total_Controller.Yaw_Angle_Control.Expect   = rovInfo.jy901.yaw; 
    }
    
    if(abs(rc->yaw) > 10 )                                                           //手柄
    {
         Total_Controller.Yaw_Angle_Control.Expect   = rovInfo.jy901.yaw;           // 期望航向角
        
    }   
    Total_Controller.Yaw_Angle_Control.FeedBack = rovInfo.jy901.yaw;                //当前值传入PID

    if (0 == Total_Controller.Yaw_Angle_Control.Expect )                            //如果开机没有解锁则不启动自稳
    {  
        yaw_conposent = 0;                                                             
    }
    else
    {
        yaw_conposent = PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);       // 获取 角度PID控制器 输出的控制量
    } 
   
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
void Speed_Buffer(short *outupt, short *rebuff, short range)
{
    static uint16_t diff_value = 0;
    diff_value = abs((*rebuff) - (*outupt)); //暂存差值的绝对值

    if (diff_value >= range ) //微分大于预设值，启动缓冲
    {
        if (*outupt <= *rebuff)
        {
            *outupt = *outupt + SPEED_BUFF;
        }
        else
        {
            *outupt = *outupt - SPEED_BUFF;
        }
        *rebuff = *outupt;
    }
    else
    {
       *outupt = *rebuff;
    }
    
}

/**
 * @brief  六轴推进器水平控制
 * @param  rocker_t 摇杆结构体
 */
void sixAixs_horizontal_control(rockerInfo_t *rc, propellerPower_t *propeller)
{
    static float propeller_Value = 1.8f; //推进器比例数值

    static int propeller_Direction_Down_X = 1; //推进器参数方向
    static int propeller_Direction_Up_X   = 1;

    static int propeller_Direction_Down_Y = -1;
    static int propeller_Direction_Up_Y   = 1;

    

    if ((abs(rc->fx) > 5 || abs(rc->fy) > 5 || abs(rc->yaw)>5)&& CONTROL_UNLOCK == cmd_data.all_lock )      //当遥感值较小或锁定时，推进器全部失能，后期可以修改为自稳 && CONTROL_UNLOCK == cmd_data.all_lock
    {
       if(abs(rc->fx) <30) rc->fx = 0;
       if(abs(rc->fy) <30) rc->fy = 0;
        /*-推进器东路 = 倍数调整（手柄XY的值（最大为255）+ 自旋值（最大为255） ）+ 横向角稳定（最大为120）----------------------*/
        PropellerBuffer.leftDown  = propeller_Value * (rc->fx * propeller_Direction_Down_X   + rc->fy * propeller_Direction_Down_Y)+ rc->yaw-yaw_conposent;     //根据遥感获取动力0~254
        PropellerBuffer.rightDown = propeller_Value * (rc->fx * propeller_Direction_Down_X   + rc->fy * propeller_Direction_Up_Y  )- rc->yaw+yaw_conposent;
        PropellerBuffer.leftUp    = propeller_Value * (rc->fx * propeller_Direction_Up_X     + rc->fy * propeller_Direction_Up_Y)+ rc->yaw-yaw_conposent;
        PropellerBuffer.rightUp   = propeller_Value * (rc->fx * propeller_Direction_Up_X     + rc->fy * propeller_Direction_Down_Y )- rc->yaw+yaw_conposent;
    
    
    }
    else
    {
        
        /* -----------自稳时，横向角锁定------------*/
        PropellerBuffer.leftDown  = -yaw_conposent;
        PropellerBuffer.rightDown =  yaw_conposent;
        PropellerBuffer.leftUp    = -yaw_conposent;
        PropellerBuffer.rightUp   =  yaw_conposent;
    }

   
}

void propeller_output(propellerPower_t *propeller)
{   
    propellerPower_t Propeller_sum;
    propeller_conposent(&PropellerBuffer, &Propellerconposent);  //死区补偿

    Propeller_sum.leftDown    =  PropellerBuffer.leftDown     +  Propellerconposent.leftDown;     //根据遥感获取动力0~254
    Propeller_sum.rightDown   = PropellerBuffer.rightDown     +  Propellerconposent.rightDown;    //推进器动力缓冲区=缓冲区+推进器死区补偿
    Propeller_sum.leftUp      = PropellerBuffer.leftUp        +  Propellerconposent.leftUp; 
    Propeller_sum.rightUp     = PropellerBuffer.rightUp       +  Propellerconposent.rightUp; 
    Propeller_sum.leftMiddle  = PropellerBuffer.leftMiddle    +  Propellerconposent.leftMiddle; 
    Propeller_sum.rightMiddle =PropellerBuffer.rightMiddle    +  Propellerconposent.rightMiddle; 
/*-----------------------动力缓冲器 让动力变化慢一些--------------------------------------------------*/
    Speed_Buffer(&propeller->leftDown,      &Propeller_sum.leftDown,  SPEED_RANGE);    //范围为10，递减系数为8的动力缓冲器
    Speed_Buffer(&propeller->rightDown,     &Propeller_sum.rightDown, SPEED_RANGE);
    Speed_Buffer(&propeller->leftUp,        &Propeller_sum.leftUp,    SPEED_RANGE);
    Speed_Buffer(&propeller->rightUp,       &Propeller_sum.rightUp,   SPEED_RANGE);
    Speed_Buffer(&propeller->leftMiddle ,   &Propeller_sum.leftMiddle,    SPEED_RANGE);
    Speed_Buffer(&propeller->rightMiddle,   &Propeller_sum.rightMiddle,   SPEED_RANGE);

}

void propeller_conposent(propellerPower_t *PropellerBuffer ,propellerPower_t *Propellerconposent)    //推进器死区补偿
{
    if(PropellerBuffer->leftDown > 5)       Propellerconposent->leftDown    =  28;
    else if(PropellerBuffer->leftDown < -5) Propellerconposent->leftDown    = -30;
    else Propellerconposent->leftDown    =  0;
    
    if(PropellerBuffer->rightDown > 5)       Propellerconposent->rightDown    =  20;
    else if(PropellerBuffer->rightDown < -5) Propellerconposent->rightDown    = -30;
    else Propellerconposent->rightDown    =  0;

    if(PropellerBuffer->leftUp > 5)       Propellerconposent->leftUp    =  33;
    else if(PropellerBuffer->leftUp < -5) Propellerconposent->leftUp    = -30;
    else Propellerconposent->leftUp    =  0;

    if(PropellerBuffer->rightUp > 5)       Propellerconposent->rightUp    =  20;
    else if(PropellerBuffer->rightUp < -5) Propellerconposent->rightUp    =  -30;
    else Propellerconposent->rightUp    =  0;

    if(PropellerBuffer->leftMiddle > 5)       Propellerconposent->leftMiddle    =  25;
    else if(PropellerBuffer->leftMiddle < -5) Propellerconposent->leftMiddle    =  -30;
    else Propellerconposent->leftMiddle    =  0;

    if(PropellerBuffer->rightMiddle > 5)       Propellerconposent->rightMiddle    =  25;
    else if(PropellerBuffer->rightMiddle < -5) Propellerconposent->rightMiddle    =  -30;
    else Propellerconposent->rightMiddle    =  0;
    

}

/**
 * @brief  动力补偿
 * @param  speech
 */
void location_keep_control(powerconpensation_t *powerconpensation,rockerInfo_t *rc)
{

    powerconpensation->X.speech_value_now = powerconpensation->X.speech_value_last + rovInfo.jy901.acc.x;                        //计算速度
    powerconpensation->X.displacement = (powerconpensation->X.speech_value_now + powerconpensation->X.speech_value_last) * 0.5f; //计算位移
    powerconpensation->Y.speech_value_now = powerconpensation->Y.speech_value_last + rovInfo.jy901.acc.y;
    powerconpensation->Y.displacement = (powerconpensation->Y.speech_value_now + powerconpensation->Y.speech_value_last) * 0.5f;

    if ((abs(rc->fx) > 10 || abs(rc->fy) > 10) && CONTROL_UNLOCK == cmd_data.all_lock)
    {
        Total_Controller.Location_X_Control.Expect = 0; // 定位原点
        Total_Controller.Location_Y_Control.Expect = 0;
    }

    Total_Controller.Location_X_Control.FeedBack = powerconpensation->X.displacement; // X加速度反馈(为x轴当前加速度)
    Total_Controller.Location_Y_Control.FeedBack = powerconpensation->Y.displacement; // Y加速度反馈(为y轴当前加速度)

    powerconpensation->X.power_conpensation = PID_Control(&Total_Controller.Location_X_Control); // 获取 X轴动力补偿PID控制器输出的控制量
    powerconpensation->Y.power_conpensation = PID_Control(&Total_Controller.Location_Y_Control); // 获取 y轴动力补偿PID控制器 输出的控制量

    if (powerconpensation->X.power_conpensation > 100)                                   //限制补偿动力
        powerconpensation->X.power_conpensation = 100;
    else if (powerconpensation->X.power_conpensation < -100)
        powerconpensation->X.power_conpensation = -100;

    if (powerconpensation->Y.power_conpensation > 100)
        powerconpensation->Y.power_conpensation = 100;
    else if (powerconpensation->Y.power_conpensation < -100)
        powerconpensation->Y.power_conpensation = -100;
}






