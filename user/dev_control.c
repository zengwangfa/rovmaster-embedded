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

void *propeller_thread(void *arg)
{
    while (1)
    {
        sixAixs_get_rocker_params(&rocker, &cmd_data);      // 获取摇杆参数
        rov_depth_control(&rocker, &rovdev.propellerPower); //深度控制
        sixAixs_horizontal_control(&rocker, &rovdev.propellerPower);
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
