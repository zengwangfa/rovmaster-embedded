#ifndef __MS5837_H_
#define __MS5837_H_

#include "../user/config.h"

#define MS5837_I2C_DEV "/dev/i2c-1" // MS5837 使用的 I2C设备
/* MS5837-30BA address is 1110110x (write: x=0, read: x=1). */
#define MS583703BA_I2C_ADDR 0x76 // MS5387 I2C 地址 (datasheet P9)

#define MS583703BA_RESET 0x1E
#define MS583703BA_ADC_RD 0x00
#define MS583703BA_PROM_RD 0xA0

/* 转换命令：压力 (datasheet P9) */
#define MS583703BA_D1_OSR_256 0x40
#define MS583703BA_D1_OSR_512 0x42
#define MS583703BA_D1_OSR_1024 0x44
#define MS583703BA_D1_OSR_2048 0x46
#define MS583703BA_D1_OSR_4096 0x48
#define MS583703BA_D1_OSR_8192 0x4A

/* 转换命令：温度 (datasheet P9) */
#define MS583703BA_D2_OSR_256 0x50
#define MS583703BA_D2_OSR_512 0x52
#define MS583703BA_D2_OSR_1024 0x54
#define MS583703BA_D2_OSR_2048 0x56
#define MS583703BA_D2_OSR_4096 0x58
#define MS583703BA_D2_OSR_8192 0x5A

/* 传感器标号 */
#define PRESSURE_SENSOR 0
#define TEMPERATURE_SENSOR 1

typedef struct
{
    /** 出厂校准参数 calib_param[7] (datasheet P12)
    *
    * C0    CRC校验 bit[15,12]  工厂参数bit[11,0] 
    * C1    压力灵敏度 SENS|T1
    * C2    压力补偿   OFF|T1
    * C3	温度压力灵敏度系数 TCS
    * C4	温度系数的压力补偿 TCO
    * C5	参考温度 T|REF
    * C6 	温度系数的温度 TEMPSENS
    */
    uint16_t c[7];      // prom中的出厂校准参数 calib_param[7]
    uint8_t crc;        // crc校验
    uint8_t factory_id; // 出厂参数 (为 c[0]的前12bit)

    uint32_t D1_Pres; // 原始压力数字量
    uint32_t D2_Temp; // 原始温度数字量

    int32_t dT;   // 实际温度与参考温度之差
    int32_t TEMP; // 实际的温度

    int64_t OFF;  // 实际温度偏移
    int64_t SENS; // 实际温度灵敏度
    int32_t P;    // 温度补偿的压力

    float pressure;    // 实际压力值
    float temperature; // 实际温度值

} ms5837_t; /* 存放ms5837相关参数 */

// MS5837初始化
int ms5837Setup(const int pinBase);

#endif
