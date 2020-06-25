/**
 * @desc: 各个传感器线程 (深度传感器、九轴、CPU 设备数据解析并获取)
 */

#define LOG_TAG "sensor"
#include "sensor.h"
#include "../drivers/ads1118.h"
#include "../drivers/jy901.h"
#include "../drivers/ms5837.h"
#include "../drivers/spl1301.h"
#include "../user/datatype.h"

#include <elog.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#define ADS1118_PIN_BASE 500 // wiringpi node编号
#define DEPTH_SENSOR_PIN_BASE 360

static jy901_t *jy901 = &rovInfo.jy901;
static powerSource_t *powerSource = &rovInfo.powerSource;
static depthSensor_t *depthSensor = &rovInfo.depthSensor;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int depthSensor_detect(int pin);
/*******************************************************************************************************************/
//
// 线程
//
/*******************************************************************************************************************/

void *adc_thread(void *arg)
{
    // TODO 待测试
    // 存放增益放大参数
    static float vref[6] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};

    while (1)
    {
        sleep(1);

        // 注意FS为 正负2.048，所以计算时为2.048/32768. (满量程是65535)
        // verf[PGA_2_048] 即为 FSR增益值 ±2.048

        powerSource->voltage = (float)analogRead(ADS1118_PIN_BASE) * (vref[2] / 32768);
        powerSource->current = (float)analogRead(ADS1118_PIN_BASE + 1) * (vref[2] / 32768);

        //printf("voltage %f  current %f\n", adc->voltage, adc->current);
    }

    return NULL;
}

void *jy901_thread(void *arg)
{
    static uint8_t data;
    // 文件描述符由 创建线程参数 arg
    int fd = *(int *)arg;
    while (1)
    {
        delay(10);
        while (serialDataAvail(fd))
        {
            data = (uint8_t)serialGetchar(fd);
            copeJY901_data(data, jy901);
        }
    }
    return NULL;
}

/**
 * @brief  深度传感器处理线程
 */
void *depthSensor_thread(void *arg)
{
    uint8_t flag = 0, cnt = 0;

    // 获取初始压力值(岸上大气压值) 如果初始压力值异常(比标准大气压很大或很小)，重新进行初始化
    depthSensor->init_pressure = digitalRead(depthSensor->pin);

    // 如果初始压力值与标准大气压相差 20kPa，重新初始化获取(3次重新初始化)     // 101325-20000 ≈ 80000(大约1800m)
    while ((abs(depthSensor->init_pressure - STANDARD_ATMOSPHERIC_PRESSURE) > 20000) && (cnt++ < 3))
    {
        log_w("depth sensor error, trying init it again...");
        depthSensor->pin += 2;                                      // 原始的pin_base编号舍去，使用+2
        depthSensor_detect(depthSensor->pin);                       // 重新初始化
        depthSensor->init_pressure = digitalRead(depthSensor->pin); // 重新获取初始压力
        if (cnt == 3)                                               // 如果第3次仍失败，则将初始压力值 = 标准大气压
            depthSensor->init_pressure = STANDARD_ATMOSPHERIC_PRESSURE;
    }
    while (1)
    {
        pthread_mutex_lock(&mutex);
        // 获取压力值 通道0
        depthSensor->pressure = digitalRead(depthSensor->pin);
        /* 如果压力值与上一次压力值相差 0.2个标准大气压，舍弃该数据 */
        if (flag && (abs(depthSensor->last_pressure - depthSensor->pressure) > (0.2 * STANDARD_ATMOSPHERIC_PRESSURE)))
        {
            log_e("%d", depthSensor->pressure);
            depthSensor->pressure = depthSensor->last_pressure; // 使用上一次的数据
        }
        flag = 1; // 第一次不进行比较上一次的值

        // 获取温度值 通道1
        depthSensor->temperature = digitalRead(depthSensor->pin + 1) / 100.0f; // 除以100转换为温度值

        // ((测得水深压力)-(岸上压力 101325))/(1000*9.8) 淡水1000  海水1030   p=ρgh
        depthSensor->depth =
            (depthSensor->pressure - depthSensor->init_pressure) / (9.8 * 1000);
        pthread_mutex_unlock(&mutex);

        /*printf("temp %.2f, init %d, pressure %d, depth %.2f\n",
               depthSensor->temperature, depthSensor->init_pressure, depthSensor->pressure, depthSensor->depth);*/

        depthSensor->last_pressure = depthSensor->pressure;
        delay(10);
    }
}

/**
 * @brief  深度传感器检测及初始化
 * @param  对应wiringpi DEPTH_SENSOR_PIN_BASE
 */
int depthSensor_detect(int pin)
{
    int fd;
    // 对应pin
    depthSensor->pin = pin;

    /* 依次初始化 spl1301、ms5837，根据获取到的数据来判断接入的是哪种传感器 */
    if ((fd = spl1301Setup(depthSensor->pin)) > 0)
        depthSensor->name = "spl1301";
    else if ((fd = ms5837Setup(depthSensor->pin)) > 0)
        depthSensor->name = "ms5837 ";
    else
        depthSensor->name = "null";

    return fd;
}

int sensor_thread_init(void)
{
    int fd;
    pthread_t adc_tid;
    pthread_t jy901_tid;
    pthread_t depth_tid;
    static uint8_t cnt = 2;

    // ADS1118 ADC 初始化
    fd = ads1118Setup(ADS1118_PIN_BASE);
    if (fd < 0) // 先判断设备是否存在，不存在不创建对应线程
        ERROR_LOG(fd, "ads1118");
    else
    {
        log_i("ads1118 init");
        pthread_create(&adc_tid, NULL, adc_thread, NULL);
        pthread_detach(adc_tid);
    }

    // JY901 九轴 初始化
    fd = jy901Setup();
    if (fd < 0)
        ERROR_LOG(fd, "jy901");
    else
    {
        log_i("jy901   init");
        pthread_create(&jy901_tid, NULL, jy901_thread, &fd);
        pthread_detach(jy901_tid);
    }

    // 深度传感器 初始化
    while (cnt--)
    {
        fd = depthSensor_detect(DEPTH_SENSOR_PIN_BASE);

        if (fd < 0 && cnt == 1) // 第1次无法检测到，打印警告信息
            log_w("depth sensor cannot detected, trying init it again...");
        else if (fd < 0 && cnt == 0) // 第2次无法检测到，打印错误信息
            ERROR_LOG(fd, "depth sersor");
        else
        {
            log_i("%s init", depthSensor->name);
            pthread_create(&depth_tid, NULL, depthSensor_thread, NULL);
            pthread_detach(depth_tid);
            break; // 初始化成功，则直接跳出
        }
    }

    return 0;
}
