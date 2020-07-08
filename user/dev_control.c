/*
 * @Author: your name
 * @Date: 2020-07-03 15:23:51
 * @LastEditTime: 2020-07-08 19:35:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \rovmaster-embedded\user\dev_control.c
 */ 
/**
 * @desc: 设备控制程序
 */
#define LOG_TAG "dev_ctrl"

#include "../applications/data.h"


#include "control.h"
#include "datatype.h"

#include <elog.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

rockerInfo_t rocker;
extern propellerPower_t Propellerconposent;

void *propeller_thread(void *arg)
{
    while (1)
    {
        sixAixs_get_rocker_params(&rocker, &cmd_data);      // 获取摇杆参数
        rov_depth_control(&rocker, &rovdev.propellerPower); //深度控制
        propeller_conposent_horizontal(&rocker, &Propellerconposent);
        sixAixs_horizontal_control(&rocker, &rovdev.propellerPower);   //6轴运动控制
        delay(20);
    }
}

int propeller_thread_init(void)
{
    pthread_t propeller_tid;

    pthread_create(&propeller_tid, NULL, &propeller_thread, NULL);
    pthread_detach(propeller_tid);

    log_i("propeller controller init");
    return 0;
}
