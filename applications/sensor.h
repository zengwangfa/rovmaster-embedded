#ifndef __SENSOR_H_
#define __SENSOR_H_

#include "../user/config.h"

/* 深度传感器设备描述符 */
typedef struct
{
    int pin;               // wiringpi node的编号
    char *name;            // 深度传感器名称
    float depth;           // 实际深度 (单位 cm)
    float temperature;     // 水温     (单位 ℃)
    int32_t pressure;      // 压力值
    int32_t last_pressure; // 上次压力值
    int32_t init_pressure; // 初始化压力值

} depthSensor_t;

/* 电源 设备描述符 */
typedef struct
{
    float percent;  // 电量百分比
    float quantity; // 电池容量
    float voltage;  // 电压(A)
    float current;  // 电流(V)

} powerSource_t;

/* --------------------------------------------------------------------------------------------------- */

// 传感器线程初始化
int sensor_thread_init(void);

#endif
