/*
 * @Description: 获取电压、电流
 * @Author: chenxi
 * @Date: 2020-02-09 17:25:35
 * @LastEditTime : 2020-02-10 17:09:56
 * @LastEditors  : chenxi
 */

#ifndef __ADC_CONVERT_H_
#define __ADC_CONVERT_H_

#include "../user/config.h"

/* ads1118 内部晶振频率 */
#define ADS1118_OSC_CLK 1000000
#define CONFIG_MUX_MASK (0x7000)

/* 单次转换 */
#define SS_NONE 0b0
#define SS_START 0b1

/* 差分输入 */
#define MUX_D0_1 0b000
#define MUX_D0_3 0b001
#define MUX_D1_3 0b010
#define MUX_D2_3 0b011

/* 单端输入 */
#define MUX_S0 0b100
#define MUX_S1 0b101
#define MUX_S2 0b110
#define MUX_S3 0b111

/* 增益放大 */
#define PGA_6_144 0b000
#define PGA_4_096 0b001
#define PGA_2_048 0b010
#define PGA_1_024 0b011
#define PGA_0_512 0b100
#define PGA_0_256 0b101

/* 工作模式 */
#define MODE_CONTINOUS 0b0
#define MODE_SINGLE 0b1

/* 采样率 */
#define DR_8_SPS 0b000
#define DR_16_SPS 0b001
#define DR_32_SPS 0b010
#define DR_64_SPS 0b011
#define DR_128_SPS 0b100
#define DR_250_SPS 0b101
#define DR_475_SPS 0b110
#define DR_860_SPS 0b111

/* 温度传感器模式 */
#define TS_MODE_ADC 0b0
#define TS_MODE_TEMP 0b1

/* 上拉使能 */
#define PULL_UP_EN_OFF 0b0
#define PULL_UP_EN_ON 0b1

/* 无操作 */
#define NOP_DATA_INVALID 0b00
#define NOP_DATA_VALID 0b01

/* 保留位 */
#define RESERVED_BIT 0b1

typedef struct
{
    uint16_t config; // 配置寄存器
    uint16_t adcVal; // ADC数据

} ads1118_t;

// ADS1118初始化
int ads1118Setup(const int pinBase);

#endif
