/**
 * @desc: 初始化程序
 */
#define LOG_TAG "init"

#include "init.h"

#include "../applications/data.h"
#include "../applications/display.h"
#include "../applications/ioDevices.h"
#include "../applications/pwmDevices.h"
#include "../applications/sensor.h"
#include "../applications/server.h"
#include "../applications/system.h"

#include "datatype.h"
#include "debug.h"

#include <elog.h>
#include <stdio.h>
#include <string.h>

#include <wiringPi.h>

// 全局ROV信息
rovInfo_t rovInfo;

// 全局ROV设备
rovDev_t rovdev = {
    .light = {
        // 探照灯
        .name = "light    ",
        .pMax = 20 * 1000, // 单位us
        .nMax = 0,
        .speed = 1000,
        .channel = LIGHT_CHANNEL,
    },

    .yuntai = {
        // 云台
        .name = "yuntai   ",
        .pMax = 5000,
        .med = 1500,
        .nMax = 1000,
        .speed = 50,
        .channel = YUNTAI_CHANNEL,
    },

    .robot_arm = {
        // 机械臂
        .name = "robot-arm",
        .cur = 1500,
        .pMax = 1550, // 参数来自ROVMaker机械手
        .med = 1500,
        .nMax = 1450,
        .speed = 50,
        .channel = ROBOT_ARM_CHANNEL,
    },
};

int system_init(void)
{
    // easylogger日志系统 初始化
    easylogger_init();

    if (wiringPiSetup() < 0)
    {
        log_e("Unable to start wiringPi: %s", strerror(errno));
        return -1;
    }

    // TODO ROV模式获取 4推 还是 6推
    log_i("Welcome to ROV Master V%s\n", ROV_MASTER_VERSION);

    rov_all_params_init(); // ROV参数初始化

    control_server_thread_init(); // 上位机服务器线程 初始化

    anoUdp_server_thread_init(); // ANO服务器线程 初始化

    sensor_thread_init(); // 传感器线程 初始化

    pwmDevs_thread_init(); // PWM设备线程 初始化

    ioDevs_thread_init(); // IO设备线程 初始化

    system_status_thread_init(); // 获取系统状态线程 初始化

    display_thread_init(); // 显示模块线程 初始化

    return 0;
}