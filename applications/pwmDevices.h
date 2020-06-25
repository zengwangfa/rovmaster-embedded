

#ifndef __PWMDEVICES_H_
#define __PWMDEVICES_H_

#include "../user/config.h"

#define HERTZ 50             // PWM频率
#define MAX_PWM 4096         // 满占空比数值
#define PCA9685_PIN_BASE 300 // PCA9685虚拟引脚

#define PROPELLER_POWER_P_MAX 2000 // 正向最大值
#define PROPELLER_POWER_STOP 1500  // 停转信号值
#define PROPELLER_POWER_N_MAX 1000 // 反向最大值

/* 推进器参数 */
typedef struct
{
    uint16_t med;      // 中值 medium(单位 us)
    uint16_t pMax;     // 正向最大值 positive(单位 us)
    uint16_t nMax;     // 反向最大值 negative(单位 us)
    uint16_t deadband; //死区值

} propellerParam_t; //推进器参数结构体

/* 推进器动力控制器 */
typedef struct
{
    int16_t leftUp;
    int16_t rightUp;
    int16_t leftDown;
    int16_t rightDown;
    int16_t leftMiddle;
    int16_t rightMiddle;

} propellerPower_t; //各个推进器推力

/* 简单PWM设备描述符 (云台、机械臂、探照灯) */
typedef struct
{
    const char *name; // 设备 名称
    uint16_t speed;   // 速度值(即单位增减的幅度)
    uint16_t channel; // 对应 输出通道
    uint16_t pMax;    // 正向最大值 positive
    uint16_t nMax;    // 反向最大值 negative
    uint16_t med;     // 中值   medium
    uint16_t cur;     // 当前值 current

} easyPWM_dev_t;

// PWM设备线程初始化
int pwmDevs_thread_init(void);

#endif
