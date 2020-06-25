/*
 * @Description: SPL1301 深度传感器驱动程序
 */

#ifndef SPL1301_H
#define SPL1301_H

#include "../user/config.h"

#define SPL1301_I2C_DEV "/dev/i2c-1" // SPL1301 使用的 I2C设备
#define SPL1301_I2C_ADDR 0x77        // SPL1301 I2C 地址 (datasheet P9)

/* 产品与版本ID 寄存器地址 */
#define PRODUCT_REVISION_ID 0x0D

/* 命令 */
#define CONTINUOUS_PRESSURE 1
#define CONTINUOUS_TEMPERATURE 2
#define CONTINUOUS_P_AND_T 3

/* 传感器标号 */
#define PRESSURE_SENSOR 0
#define TEMPERATURE_SENSOR 1

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl1301_calib_param_t; // spl1301 出厂校准参数

typedef struct
{
    spl1301_calib_param_t calib_param; /* calibration data */
    int8_t product_id;                 // 产品ID
    int8_t revision_id;                // 修订ID

    int32_t i32rawPressure;    // 原始压力值
    int32_t i32rawTemperature; // 原始温度值
    int32_t i32kP;             // 压力系数
    int32_t i32kT;             // 温度系数

    float pressure;    // 实际压力值
    float temperature; // 实际温度值
} spl1301_t;

// SPL1301 初始化
int spl1301Setup(const int pinBase);
// 获取厂家ID与版本ID
int spl1301_get_id(int fd);
// 设置特定传感器的采样率和每秒过采样率
void spl1301_rateset(int fd, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl);
// 开始一次温度测量
void spl1301_start_temperature(int fd);
// 开始一次压力测量
void spl1301_start_pressure(int fd);
// 选择连续测量模式
void spl1301_start_continuous(int fd, uint8_t mode);
// 获取原始温度值并将其转换为32位整数
void spl1301_get_raw_temp(int fd);
// 调用该句转换数据,获取原始压力值并将其转换为32位整数
void spl1301_get_raw_pressure(int fd);
// 根据原始值返回校准温度值
float get_spl1301_temperature(int fd);
// 根据原值返回校准压力值
float get_spl1301_pressure(int fd);

#endif
