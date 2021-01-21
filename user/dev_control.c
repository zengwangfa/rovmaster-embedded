/*
 * @Author: your name
 * @Date: 2020-07-03 15:23:51
 * @LastEditTime: 2021-01-15 16:21:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \rovmaster-embedded\user\dev_control.c
 */ 
/**
 * @desc: 设备控制程序
 */
#define LOG_TAG "dev_ctrl"

#include "../applications/data.h"
#include "../applications/PID.h"

#include "control.h"
#include "datatype.h"
#include "focus.h"
#include "data.h"
#include <elog.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

extern rockerInfo_t rocker;
extern int yaw_conposent;     //测试
extern powerconpensation_t Propeller_resist_flow;
extern uint8_t camera_control_data[6];
extern propellerPower_t PropellerBuffer;

void *propeller_thread(void *arg)
{
    while (1)
    {
        sixAixs_get_rocker_params(&rocker, &cmd_data);      // 获取摇杆参数
        rov_depth_control(&rocker, &rovdev.propellerPower); //深度控制
        rov_yaw_control(&rocker);                                      //横向角锁定
       // rov_roll_control(&rocker);                                     //横滚角锁定
        sixAixs_horizontal_control(&rocker, &rovdev.propellerPower);   //6轴运动控制
      //  location_keep_control(&Propeller_resist_flow,&rocker);//抗流
        
        propeller_output(&rovdev.propellerPower);   //动力输出
        delay(20);
    }
}

void *test_printf(void *arg)
{
    while (1)
    {
        printf("propeller->leftUp %d \n",rovdev.propellerPower.leftUp);
        printf("propeller->rightUp %d \n",rovdev.propellerPower.rightUp);
        printf("propeller->leftDown %d \n",rovdev.propellerPower.leftDown);
        printf("propeller->rightDown %d \n",rovdev.propellerPower.rightDown);
        //printf("propeller->leftMiddle %d \n",rovdev.propellerPower.leftMiddle);
        // printf("propeller->rightMiddle %d \n",rovdev.propellerPower.rightMiddle); 
         printf("fx %d \n",rocker.fx);
         printf("fy %d \n",rocker.fy);
        // // printf("up_down %d \n",cmd_data.up_down);
        // printf("rc-yaw %d \n",rocker.yaw);
        // printf("Expect %f \n",Total_Controller.Yaw_Angle_Control.Expect);
         printf("FeedBack %f \n",Total_Controller.Yaw_Angle_Control.FeedBack);
        // printf("Kd %f \n",Total_Controller.Yaw_Angle_Control.Kp);
        // printf("Kd %f \n",Total_Controller.Yaw_Angle_Control.Ki);
        // printf("Kd %f \n",Total_Controller.Yaw_Angle_Control.Kd);

        sleep(1);
    }
        
    return 0;
}

int propeller_thread_init(void)
{
    pthread_t propeller_tid;
    pthread_t propeller_tid1;

    pthread_create(&propeller_tid, NULL, &propeller_thread, NULL);
    pthread_detach(propeller_tid);

    pthread_create(&propeller_tid1, NULL, &test_printf, NULL);
    pthread_detach(propeller_tid1);

    log_i("propeller controller init");
    return 0;
}
