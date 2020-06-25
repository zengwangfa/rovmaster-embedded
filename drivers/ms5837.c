/**
 * @desc: MS5837 深度传感器驱动
 *
 *       Notes: 水深传感器设备驱动
 *   Attention: SCL (黑色)   
 *				SDA (黄色)   
 */
#define LOG_TAG "ms5837"

#include "ms5837.h"

#include <elog.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

static ms5837_t ms5837;

/**
  * @brief  MS5837 PROM校准参数，crc4校验函数 (datasheet P12)
  * @param  数组
  * @retval 返回crc 4bit校验
  */
uint8_t ms5837_crc4(uint16_t *n_prom)
{
    int32_t cnt;
    uint32_t n_rem = 0; // crc 余数
    uint8_t n_bit;

    n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7] = 0;                      // Subsidiary value, set to 0
    for (cnt = 0; cnt < 16; cnt++)      // operation is performed on bytes
    {
        // choose LSB or MSB
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000f); // final 4-bit remainder is CRC code
    return (n_rem ^ 0x00);
}

/**
  * @brief  ms5837 复位
  */
int ms5837_reset(int fd)
{
    return wiringPiI2CWrite(fd, MS583703BA_RESET);
}

/**
  * @brief  ms5837获取出厂标定参数
  * @param  None
  * @retval 返回出厂标定参数 crc校验 是否成功标志：1成功，-1失败
  *  若成功表示为ms5837传感器
  *  若失败表示为其他类型传感器或无
  */
int ms5837_get_calib_param(int fd)
{
    int i;

    for (i = 0; i <= 6; i++)
    {
        // 读取prom中的出厂标定参数
        ms5837.c[i] =
            wiringPiI2CReadReg16(fd, MS583703BA_PROM_RD + (i * 2));

        /* 高8位 与 低8位互换，由于i2c读取先读取 MSB */
        ms5837.c[i] = (ms5837.c[i] << 8) | (ms5837.c[i] >> 8);
    }

    // crc校验为 C[0]的 bit[15,12]
    ms5837.crc = (uint8_t)(ms5837.c[0] >> 12);

    // 工厂定义参数为 c[0] 的bit[14,0]
    ms5837.factory_id = (uint8_t)(ms5837.c[0] & 0x0fff);
    /* 
	 * crc校验为用于判断 ms5837 是否初始化成功 
	 * 即为了检测接入的设备是否是 ms5837传感器
	*/
    if (ms5837.crc == ms5837_crc4(ms5837.c))
        return 1; // 校验成功，返回1

    return -1;
}

/**
 * @brief  ms5837 获取转换数据
 * @param 
 *  uint8_t command  带精度温度命令  带精度温度压力(见头文件)
 * @retval
 *  uint32_t 数据结果
 */
uint32_t ms5837_get_conversion(int fd, uint8_t command)
{
    uint8_t temp[3];

    // 1.先写入转换命令(即指定转换传感器及精度) (datasheet P11)
    wiringPiI2CWrite(fd, command);

    /* 2.延时等待转换完成  
	 * eg.读取8196精度时，等待时间必须大于 datasheet P2页中的18.08毫秒，否则无法获取数据
	 */
    delay(30);

    // 3.写入 ADC read读取命令(让器件准备数据)
    wiringPiI2CWrite(fd, MS583703BA_ADC_RD);

    // 4.读取 24bit(3个字节)的转换数据 高位在前
    read(fd, temp, 3); // 由于wiringpi没有提供I2C读取多个字节，因此使用read代替

    return ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | ((uint32_t)temp[2]);
}

/**
 * @brief  获取并计算温度值
 *  此时的温度值还没经过补偿，并不准确
 */
void ms5837_cal_raw_temperature(int fd)
{
    // 获取原始温度数字量
    ms5837.D2_Temp = ms5837_get_conversion(fd, MS583703BA_D2_OSR_8192);
    // 实际温度与参考温度之差 (公式见datasheet P7)
    ms5837.dT = (int32_t)ms5837.D2_Temp - ((int32_t)ms5837.c[5] << 8);
    // 实际的温度
    ms5837.TEMP = 2000 + (((int64_t)ms5837.dT * (int64_t)ms5837.c[6]) >> 23);
}

/**
 * @brief  计算温度补偿后的压力值与二阶修正后温度值
 *   除法采用移位为了提高计算效率
 *   计算中的(int64_t)强制类型转换为了防止计算数据溢出
 */
void ms5837_cal_pressure_and_temp(int fd)
{
    int64_t Ti, OFFi, SENSi;
    int64_t dT_squ; // dT的乘方
    uint32_t temp_minus_squ, temp_plus_squ;

    // 获取原始压力数字量
    ms5837.D1_Pres = ms5837_get_conversion(fd, MS583703BA_D1_OSR_8192);
    // 实际温度偏移
    ms5837.OFF = ((int64_t)ms5837.c[2] << 16) + (((int64_t)(ms5837.c[4] * ms5837.dT)) >> 7);
    // 实际温度灵敏度
    ms5837.SENS = ((int64_t)ms5837.c[1] << 15) + (((int64_t)(ms5837.c[3] * ms5837.dT)) >> 8);

    dT_squ = ((int64_t)ms5837.dT * (int64_t)ms5837.dT);           // dT的2次方
    temp_minus_squ = (2000 - ms5837.TEMP) * (2000 - ms5837.TEMP); // 温度差的2次方

    /* 对温度和压力进行二阶修正 (datasheet P8) */
    if (ms5837.TEMP < 2000) // 低温情况:低于20℃时
    {
        Ti = (3 * dT_squ) >> 33;
        OFFi = 3 * temp_minus_squ / 2;
        SENSi = 5 * temp_minus_squ / 8;

        if (ms5837.TEMP < -1500) // 超低温情况:低于-15℃时
        {
            temp_plus_squ = (ms5837.TEMP + 1500) * (ms5837.TEMP + 1500); // 温度和的2次方
            OFFi += 7 * temp_plus_squ;
            SENSi += 4 * temp_plus_squ;
        }
    }
    else // 高温情况:高于20℃时
    {
        Ti = (2 * dT_squ) >> 37;
        OFFi = temp_minus_squ >> 4;
        SENSi = 0;
    }
    ms5837.OFF -= OFFi;
    ms5837.SENS -= SENSi;

    // 温度补偿后的压力值
    ms5837.P = ((((int64_t)ms5837.D1_Pres * ms5837.SENS) >> 21) - ms5837.OFF) >> 13;
    // 实际温度值
    ms5837.temperature = (ms5837.TEMP - Ti) / 100.0f;
    // 实际压力值
    ms5837.pressure = ms5837.P / 10.0f;
}

//------------------------------------------------------------------------------------------------------------------
//
//	用于 WiringPi functions
//
//------------------------------------------------------------------------------------------------------------------

/**
  * @brief  ms5837 根据引脚转换为通道获取相应数值
  *  返回的压力值单位为 Pa
  *  返回的温度值单位为 摄氏度的100倍
  */
static int myDigitalRead(struct wiringPiNodeStruct *node, int pin)
{
    /* 0为压力通道，1为温度通道 */
    int channel = pin - node->pinBase;
    int fd = node->fd;

    // 因为程序调用时，应当先获取压力值，获取完压力值后(经过二阶修正)，温度值才是准确的
    // 因此，获取温度时，不再进行数据计算，直接返回温度数据，避免浪费计算资源
    if (TEMPERATURE_SENSOR == channel)
    {
        return (int)(ms5837.temperature * 100); // 扩大100倍，方便int类型传输
    }
    /* 先获取温度数据，因为需要进行温度补偿 
	 * 在计算压力函数中，会计算温度二阶，使得温度更加准确
     * 因此调用 ms5837_cal_pressure_and_temp 函数后，温度才是准确值
	*/
    ms5837_cal_raw_temperature(fd);   // 计算原始温度数据
    ms5837_cal_pressure_and_temp(fd); // 计算温度补偿后的压力值与二阶修正后温度值

    if (PRESSURE_SENSOR == channel)
    {
        // 由于MS5837读取到的压力值单位为mbar (1mbar = 100Pa，因此*100)
        return (int)(ms5837.pressure * 100); // 转换单位为Pa
    }

    log_e("ms5837 channel range in [0, 1]");
    return -1;
}

/**
 * @brief  初始化并设置 ms5837
 * @param 
 *  int pinBase  pinBase > 64
 */
int ms5837Setup(const int pinBase)
{
    int fd;
    struct wiringPiNodeStruct *node = NULL; // 指针初始化为NULL，以免产生段错误

    // 小于0代表无法找到该i2c接口，输入命令 sudo npi-config 使能该i2c接口
    if ((fd = wiringPiI2CSetupInterface(MS5837_I2C_DEV, MS583703BA_I2C_ADDR)) < 0)
        return -1;

    delay(50);
    /* 检测是否存在 ms5837 器件
     * 写入复位，如果写入失败，代表不存在 MS5837，或者器件地址错误
	 * 复位的目的：复位才可读取校准数据 (datasheet P10) 
    */
    if (ms5837_reset(fd) < 0)
        return -2;
    delay(50);
    /* 获取校准参数，若获取的数据CRC校验失败，则判定 接入的不是MS5837，或未接入MS5837 */
    if (ms5837_get_calib_param(fd) < 0)
        return -3;

    // 创建节点，2个通道，一个为压力值，一个为温度值
    node = wiringPiNewNode(pinBase, 2);
    if (!node)
        return -4;

    // 注册方法
    node->fd = fd;
    node->digitalRead = myDigitalRead;

    return fd;
}
