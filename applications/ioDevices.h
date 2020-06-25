
#ifndef __IO_DEVICES_H__
#define __IO_DEVICES_H__

#include "../user/config.h"

#include <wiringPi.h>

// 蜂鸣器 wiringpi引脚号
#define BUZZER_PIN 12
// 按键 wiringpi引脚号
#define BUTTON_PIN 24

// RGB灯 wiringpi引脚号
#define LEDR_PIN 13
#define LEDG_PIN 14
#define LEDB_PIN 10

// RGB灯共阳极接到+3.3V，电平 0亮 1灭
#define LED_ON(pin) digitalWrite(pin, LOW)
#define LED_OFF(pin) digitalWrite(pin, HIGH)

#define IO_OUPUT_HIGH(pin) digitalWrite(pin, HIGH)
#define IO_OUPUT_LOW(pin) digitalWrite(pin, LOW)

/* 模拟PWM设备描述符 */
typedef struct
{
    char *name;      // 设备名称
    uint32_t cnt;    // 计数器
    uint32_t time;   // 持续时间
    uint32_t period; // 周期
    uint8_t duty;    // 占空比 (0~100)
    uint8_t pin;     // 引脚
    uint8_t flag;    // 模式标志位
    void (*off)(void);
} softPWM_t;

// IO口设备线程初始化
int ioDevs_thread_init(void);

#endif
