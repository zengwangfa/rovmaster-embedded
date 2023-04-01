/*
 * @Author: your name
 * @Date: 2020-07-03 15:23:51
 * @LastEditTime: 2020-11-01 12:35:02
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

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

extern rockerInfo_t rocker;
extern int yaw_conposent;     //测试
extern powerconpensation_t Propeller_resist_flow;
extern uint8_t camera_control_data[6];

void *propeller_thread(void *arg)
{
    while (1)
    {
        sixAixs_get_rocker_params(&rocker, &cmd_data);      // 获取摇杆参数
        rov_depth_control(&rocker, &rovdev.propellerPower); //深度控制
        rov_yaw_control(&rocker);                                      //横向角锁定
        sixAixs_horizontal_control(&rocker, &rovdev.propellerPower);   //6轴运动控制
        location_keep_control(&Propeller_resist_flow,&rocker);//抗流
        
        propeller_output(&rovdev.propellerPower);   //动力输出
        delay(20);
    }
}

void *test_printf(void *arg)
{
    while (1)
    {
        // static int i;
        // printf("propeller->leftUp %d \n",rovdev.propellerPower.leftUp);
        // printf("propeller->rightUp %d \n",rovdev.propellerPower.rightUp);
        // printf("propeller->leftDown %d \n",rovdev.propellerPower.leftDown);
        // printf("propeller->rightDown %d \n",rovdev.propellerPower.rightDown);
        // printf("propeller->leftMiddle %d \n",rovdev.propellerPower.leftMiddle);
        // printf("propeller->rightMiddle %d \n",rovdev.propellerPower.rightMiddle); 
        // printf("Control_OutPut %f \n",Total_Controller.Yaw_Angle_Control.Control_OutPut);
        // printf("Expect %f \n",Total_Controller.Yaw_Angle_Control.Expect);
        // printf("FeedBack %f \n",Total_Controller.Yaw_Angle_Control.FeedBack);
        // printf("roll %f \n",rovInfo.jy901.roll);
        // printf("pitch %f \n",rovInfo.jy901.pitch);
        // for ( i = 0; i < 6; i++)
        // {
        //     printf("control_data[%d] %d \n",i,camera_control_data[i]);
        // }
        
        printf("camera %d \n",cmd_data.camera);
        printf("ms5837 %lf \n",rovInfo.depthSensor.depth);
        // printf("depth %f \n",rovInfo.depthSensor.depth);
        // printf("acc %f \n",rovInfo.jy901.acc.x);
        // printf("Expect %f \n",Total_Controller.Location_X_Control.Expect);
        // printf("displacement %f \n",Propeller_resist_flow.X.displacement);
        // printf("Err %f \n",Total_Controller.Location_X_Control.Err);
        // printf("FeedBack %f \n",Total_Controller.Location_X_Control.FeedBack);
        // printf("Err_Limit_Flag %d \n",Total_Controller.Location_X_Control.Err_Limit_Flag);
        // printf("Control_OutPut %f \n",Total_Controller.Location_X_Control.Control_OutPut);
        // printf("power_conpensation %d \n",Propeller_resist_flow.X.power_conpensation);

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

    printf("propeller controller init");
    return 0;
}
