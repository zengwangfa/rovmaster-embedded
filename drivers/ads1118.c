
/*
 * @description: ads1118 ADC 驱动程序
 *
 * Copyright (c) 2019-2020, Ian, <zengwangfa@outlook.com>
 */

#define LOG_TAG "ads1118"

#include "ads1118.h"

#include <elog.h>
#include <stdio.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

static ads1118_t ads1118;

/**
  * @brief  ads1118 根据通道获取数据
  */
static uint16_t ads1118_transmit(int fd, int channel)
{
    uint8_t spiData[4] = {0};

    ads1118.config &= ~CONFIG_MUX_MASK; // 通道寄存器位 清0
    switch (channel)
    {
    case 0:
        ads1118.config |= (MUX_S0 << 12);
        break;
    case 1:
        ads1118.config |= (MUX_S1 << 12);
        break;
    case 2:
        ads1118.config |= (MUX_S2 << 12);
        break;
    case 3:
        ads1118.config |= (MUX_S3 << 12);
        break;
    default:
        log_e("ads1118 channel range in [0, 3]");
    }

    spiData[0] = (ads1118.config >> 8) & 0xff; // 写入配置寄存器 (datasheet P24)
    spiData[1] = (ads1118.config & 0xff);
    spiData[2] = (ads1118.config >> 8) & 0xff;
    spiData[3] = (ads1118.config & 0xff);
    //printf("%x %x %x %x\n",spiData[0],spiData[1],spiData[2],spiData[3]);
    wiringPiSPIDataRW(fd, spiData, 4);
    delay(1000); // 等待数据转换完毕
    //printf("%x %x %x %x\n",spiData[0],spiData[1],spiData[2],spiData[3]);
    return (spiData[0] << 8) | spiData[1]; // SPI传输高位在前
}

//------------------------------------------------------------------------------------------------------------------
//
//	用于 WiringPi functions
//
//------------------------------------------------------------------------------------------------------------------

/**
  * @brief  ads1118 根据增益转换实际电压值
  */
static int myAnalogRead(struct wiringPiNodeStruct *node, int pin)
{

    int fd = node->fd;
    // 获取是 ads1118 的第几通道
    int channel = pin - node->pinBase;

    ads1118.adcVal = ads1118_transmit(fd, channel);

    return ads1118.adcVal;
}

/**
 * @brief  初始化并设置 ads1118
 * @param 
 *  int pinBase  pinBase > 64
 *  int spiChannel  使用的是哪一个spi设备(/dev/spidev1.0 == 1)
 *
 *  可修改相关配置: 增益放大、采样率...
 */
int ads1118Setup(const int pinBase)
{
    int fd;
    struct wiringPiNodeStruct *node = NULL; // 指针初始化为NULL，以免产生段错误

    /* 注意ads1118配置SPI接口为 SPI mode 1  */
    uint8_t mode = 1;   // mode 1 (CPOL = 0, CPHA = 1)  (datasheet P31)
    int spiChannel = 1; // spiChannel = 1代表使用 /dev/spidev1.0

    // 小于0代表无法找到该spi接口，输入命令 sudo npi-config 使能该spi接口
    fd = wiringPiSPISetupMode(spiChannel, ADS1118_OSC_CLK, mode); // (/dev/spidev1.0   1MHz  mode1)
    if (fd < 0)
        return -1;

    ads1118.config = 0;
    /* 设置配置寄存器 448Bh */
    ads1118.config |= (SS_NONE << 15);       // 不启动单发转换
    ads1118.config |= (MUX_S0 << 12);        // 通道 AIN0
    ads1118.config |= (PGA_2_048 << 9);      // 可编程增益放大 ±2.048v
    ads1118.config |= (MODE_CONTINOUS << 8); // 连续转换模式
    ads1118.config |= (DR_128_SPS << 5);     // 每秒采样率 128
    ads1118.config |= (TS_MODE_ADC << 4);    // ADC 模式
    ads1118.config |= (PULL_UP_EN_ON << 3);  // 数据引脚上拉使能
    ads1118.config |= (NOP_DATA_VALID << 1); // 有效数据,更新配置寄存器
    ads1118.config |= (RESERVED_BIT);        // 保留位

    /* 检测是否存在 ads1118 器件
     * 读写数据得到数据为0，代表无数据，即未接入 ads1118，或者spi接口错误
    */
    if ((0 == ads1118_transmit(spiChannel, 0)) && (0 == ads1118_transmit(spiChannel, 1))) // 0代表 AIN0；1代表 AIN1
        return -2;

    // 创建节点加入链表 4 pins 共4个通道
    node = wiringPiNewNode(pinBase, 4);

    if (!node)
        return -4;

    node->fd = spiChannel;
    node->analogRead = myAnalogRead;

    return 0;
}
