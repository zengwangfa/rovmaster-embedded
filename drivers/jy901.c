/*
 * @Description: JY901 九轴数据解析程序
 */

#define LOG_TAG "jy901"

#include "jy901.h"

#include <elog.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>

static uint8_t jy901_reset_cmd[5] = {0xFF, 0xAA, 0x00, 0x01, 0x00}; // 0x00-设置保存  0x01-恢复出厂设置并保存

// JY901 原始数据
static jy901_raw_t jy901_raw;

// 恢复出厂设置并保存
void jy901Reset(int fd)
{
    write(fd, jy901_reset_cmd, 5);
}

/**
  *  @brief  JY901数据转换
  */
static void jy901_convert(uint8_t which, jy901_t *jy901)
{
    // 选择数据包
    switch (which)
    {
    case 0x51:
    {
        jy901->acc.x = (float)jy901_raw.stcAcc.a[0] / 2048; // 32768*16
        jy901->acc.y = (float)jy901_raw.stcAcc.a[1] / 2048;
        jy901->acc.z = (float)jy901_raw.stcAcc.a[2] / 2048;
        jy901->temperature = (float)jy901_raw.stcAcc.T / 100;
        //printf("acc %0.2f %0.2f %0.2f  %0.2fC\n", jy901->acc.x, jy901->acc.y, jy901->acc.z, jy901->temperature);
    }
    break;
    case 0x52:
    {
        jy901->gyro.x = (float)jy901_raw.stcGyro.w[0] / 2048 * 125; // 32768*2000
        jy901->gyro.y = (float)jy901_raw.stcGyro.w[1] / 2048 * 125;
        jy901->gyro.z = (float)jy901_raw.stcGyro.w[2] / 2048 * 125;
        //printf("gyro %0.2f %0.2f %0.2f\n", jy901->gyro.x, jy901->gyro.y, jy901->gyro.z);
    }
    break;
    case 0x53:
    {
        jy901->roll = (float)jy901_raw.stcAngle.angle[0] / 8192 * 45; // 32768*180;
        jy901->pitch = (float)jy901_raw.stcAngle.angle[1] / 8192 * 45;
        jy901->yaw = (float)jy901_raw.stcAngle.angle[2] / 8192 * 45;
        //printf("angle %0.2f %0.2f %0.2f\n", jy901->roll, jy901->pitch, jy901->yaw);
    }
    break;
    case 0x54:
    {
        jy901->mag.x = jy901_raw.stcMag.h[0];
        jy901->mag.y = jy901_raw.stcMag.h[1];
        jy901->mag.z = jy901_raw.stcMag.h[2];
        //printf("mag %d %d %d\n", jy901->mag.x, jy901->mag.y, jy901->mag.z);
    }
    break;
    case 0x56: // 气压值
    {
        jy901->pressure = jy901_raw.stcPress.lPressure;
        jy901->altitude = jy901_raw.stcPress.lAltitude;
        //printf("mag %d %d %d\n", jy901->mag.x, jy901->mag.y, jy901->mag.z);
    }
    break;
    default:
        break;
    }
}

/**
  * @brief  copeJY901_data为串口中断调用函数，串口每收到一个数据，调用一次这个函数
  *         判断数据是哪种数据，然后将其拷贝到对应的结构体中
  */
void copeJY901_data(uint8_t data, jy901_t *jy901)
{
    static uint8_t i;
    static uint8_t rxBuffer[20] = {0}; // 数据包
    static uint8_t rxCheck = 0;        // 尾校验字
    static uint8_t rxCount = 0;        // 接收计数

    rxBuffer[rxCount++] = data; // 将收到的数据存入缓冲区中
    if (rxBuffer[0] != 0x55)
    {
        // 数据头不对，则重新开始寻找0x55数据头
        rxCount = 0; // 清空缓存区
        return;
    }
    if (rxCount < JY901_PACKET_LENGTH)
        return; // 数据不满11个，则返回

    /*********** 只有接收满11个字节数据 才会进入以下程序 ************/
    for (i = 0; i < 10; i++)
        rxCheck += rxBuffer[i]; //校验位累加

    if (rxCheck == rxBuffer[JY901_PACKET_LENGTH - 1]) // 判断数据包校验是否正确
    {
        // 判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
        switch (rxBuffer[1])
        {
        case 0x50: // 拷贝数据 舍去包头与数据长度位
            memcpy(&jy901_raw.stcTime, &rxBuffer[2], 8);
            break;
        case 0x51:
            memcpy(&jy901_raw.stcAcc, &rxBuffer[2], 8);
            break;
        case 0x52:
            memcpy(&jy901_raw.stcGyro, &rxBuffer[2], 8);
            break;
        case 0x53:
            memcpy(&jy901_raw.stcAngle, &rxBuffer[2], 8);
            break;
        case 0x54:
            memcpy(&jy901_raw.stcMag, &rxBuffer[2], 8);
            break;
        case 0x55:
            memcpy(&jy901_raw.stcDStatus, &rxBuffer[2], 8);
            break;
        case 0x56:
            memcpy(&jy901_raw.stcPress, &rxBuffer[2], 8);
            break;
        case 0x57:
            memcpy(&jy901_raw.stcLonLat, &rxBuffer[2], 8);
            break;
        case 0x58:
            memcpy(&jy901_raw.stcGPSV, &rxBuffer[2], 8);
            break;
        case 0x59:
            memcpy(&jy901_raw.stcQ, &rxBuffer[2], 8);
            break;
        }
        /* JY901 数据转换 */
        jy901_convert(rxBuffer[1], jy901);
    }
    rxCount = 0; // 清空缓存区
    rxCheck = 0; // 校验位清零
}

/**
  * @brief  打开对应 JY901 串口设备
  */
int jy901Setup(void)
{
    int fd;
    // 小于0代表无法找到该uart接口，输入命令 sudo npi-config 使能该uart接口
    fd = serialOpen(JY901_UART_DEV, UART_BAUD_9600);
    if (fd < 0)
        return -1;

    // jy901Reset(fd); // 恢复出厂设置
    return fd;
}
